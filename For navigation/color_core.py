# -*- coding: utf-8 -*-
"""三岔路口色卡识别核心（PC 端）。

从 三岔路口.py 抽出检测/跟踪算法，去掉视频读取、窗口和 main()。
输入 BGR ndarray，输出：
  color: 0 无 / 1 红 / 2 黄 / 3 蓝（多色同时确认时取框面积最大的）
  trackers: 每种颜色当前的确认框，供调试绘制

坐标约定：对外返回的 rect 是原图坐标 (x, y, w, h)，原点在左上角。
"""

import cv2


# ==================== 可调参数 ====================
PROC_WIDTH = 640          # 检测宽度；0 表示不缩放
USE_GAUSSIAN = True
GAUSSIAN_KERNEL = (3, 3)
MORPH_KERNEL_SIZE = 3

WINDOW = 10               # 确认前的投票窗口帧数
MIN_VOTES = 4             # 窗口内至少命中几帧才确认
LOST_TOLERANCE = 5        # 已确认后连续丢帧几帧才取消

SMOOTH_ALPHA = 0.65       # 位置平滑
SMOOTH_BETA = 0.30        # 位置速度修正
SIZE_ALPHA = 0.30         # 宽高平滑

# 原实现 MIN_AREA=12500 是按 1280x720 缩到 640x360 标定的（约 1.36%）。
# 板端现在是 640x480，改成按帧面积比例，避免直接套绝对值后大量漏检。
MIN_AREA_RATIO = 0.0135   # 640x480 下约 4147 px
MIN_AREA_ABS = 0          # 需要绝对下限时填这里

COLOR_NONE = 0

# label, 颜色编号, HSV 下限, HSV 上限, BGR 绘制色
COLOR_DEFS = (
    ("RED", 1, (0, 50, 20), (5, 255, 255), (0, 0, 255)),
    ("YELLOW", 2, (18, 69, 74), (55, 255, 255), (0, 255, 255)),
    ("BLUE", 3, (100, 100, 70), (130, 255, 255), (255, 0, 0)),
)
# ==================================================


def _to_src(rect, invScale):
    """检测分辨率坐标 -> 原图坐标。"""
    x, y, w, h = rect
    return (
        int(round(x * invScale)),
        int(round(y * invScale)),
        max(1, int(round(w * invScale))),
        max(1, int(round(h * invScale))),
    )


def _confidence(votes):
    if WINDOW <= 0:
        return 0
    return min(100, max(0, int(round(100.0 * float(votes) / WINDOW))))


def _empty_result():
    return {
        "color": COLOR_NONE,
        "label": "NONE",
        "rect": None,
        "cx": 0,
        "cy": 0,
        "w": 0,
        "h": 0,
        "confidence": 0,
        "confirmed": False,
        "trackers": [],
        "candidate_count": 0,
        "fps": 0.0,
        "proc_ms": 0.0,
    }


def findDetect(view, lower, upper, minArea, kernel, offset=(0, 0)):
    """在 HSV 图像（或 ROI 子图）中找面积最大的色块，返回 (x, y, w, h)。

    view 为 ROI 子图时，offset 传 ROI 左上角坐标，返回值仍是整图坐标。
    """
    mask = cv2.inRange(view, lower, upper)

    # 开运算去小噪点 -> 闭运算填洞
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    best = None
    bestArea = minArea
    for c in contours:
        area = cv2.contourArea(c)
        if area > bestArea:
            bestArea = area
            best = cv2.boundingRect(c)

    if best is None:
        return None
    x, y, w, h = best
    return (x + offset[0], y + offset[1], w, h)


def drawDetect(label, rect, img, boxColor, scale=1.0):
    """画矩形框、中心点和标签（rect 为检测分辨率坐标，scale 映射回原图）。"""
    x = int(round(rect[0] * scale))
    y = int(round(rect[1] * scale))
    w = max(1, int(round(rect[2] * scale)))
    h = max(1, int(round(rect[3] * scale)))

    cv2.rectangle(img, (x, y), (x + w, y + h), boxColor, 2)

    center = (x + w // 2, y + h // 2)
    cv2.circle(img, center, 5, boxColor, -1)

    (textW, textH), baseline = cv2.getTextSize(
        label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
    )
    textOrg = (x, y - 5)
    if textOrg[1] - textH < 0:
        textOrg = (x, y + textH + 5)

    # 黑底白字，避免背景颜色干扰读数
    cv2.rectangle(
        img,
        (textOrg[0], textOrg[1] - textH - 4),
        (textOrg[0] + textW + 4, textOrg[1] + baseline),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        img,
        label,
        (textOrg[0] + 2, textOrg[1]),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )


def countVotes(history):
    """窗口内有效框数量（None 代表该帧未检出）。"""
    return sum(1 for r in history if r is not None and r[2] * r[3] > 0)


def voteRect(history):
    """取窗口内有效框的平均值，作为刚确认时的初始框。"""
    x = y = w = h = 0.0
    n = 0
    for r in history:
        if r is None or r[2] * r[3] <= 0:
            continue
        x += r[0]
        y += r[1]
        w += r[2]
        h += r[3]
        n += 1
    if n == 0:
        return None
    return (x / n, y / n, w / n, h / n)


class ColorTracker(object):
    """单色跟踪：多帧投票确认 + α-β 滤波速度预测。"""

    def __init__(self, label, colorId, lower, upper, color):
        self.label = label
        self.colorId = colorId
        self.lower = lower
        self.upper = upper
        self.color = color
        self.history = []
        self.confirmed = False
        self.miss = 0
        self.state = None
        self.velocity = [0.0, 0.0, 0.0, 0.0]

    def reset(self):
        self.history = []
        self.confirmed = False
        self.miss = 0
        self.state = None
        self.velocity = [0.0, 0.0, 0.0, 0.0]

    def votes(self):
        return countVotes(self.history)

    def update(self, rect):
        """rect：本帧检测结果（检测分辨率坐标），None 表示未检出。"""
        if len(self.history) >= WINDOW:
            self.history.pop(0)
        self.history.append(rect)

        self.miss = 0 if rect is not None else self.miss + 1

        if not self.confirmed:
            if rect is None or countVotes(self.history) < MIN_VOTES:
                return
            seed = voteRect(self.history)
            if seed is None:
                return
            self.state = [float(v) for v in seed]
            self.velocity = [0.0, 0.0, 0.0, 0.0]
            self.confirmed = True
            self.miss = 0
        elif self.miss >= LOST_TOLERANCE:
            self.reset()
            return

        if rect is None:
            # 短暂丢帧：按速度外推，速度衰减防止漂移
            self.state = [s + v for s, v in zip(self.state, self.velocity)]
            self.velocity = [v * 0.8 for v in self.velocity]
            return

        # α-β 滤波：先预测，再用本帧测量修正
        z = [float(v) for v in rect]
        pred = [s + v for s, v in zip(self.state, self.velocity)]
        err = [zi - pi for zi, pi in zip(z, pred)]
        alpha = (SMOOTH_ALPHA, SMOOTH_ALPHA, SIZE_ALPHA, SIZE_ALPHA)
        beta = (SMOOTH_BETA, SMOOTH_BETA, 0.0, 0.0)
        self.state = [p + a * e for p, a, e in zip(pred, alpha, err)]
        self.velocity = [v + b * e for v, b, e in zip(self.velocity, beta, err)]

    def roi(self, width, height):
        """根据滤波结果 + 速度预测生成下一帧的检测 ROI。"""
        if not self.confirmed or self.state is None:
            return None

        x, y, w, h = self.state
        cx = x + w * 0.5 + self.velocity[0] * 2.0
        cy = y + h * 0.5 + self.velocity[1] * 2.0
        halfW = w * 0.9 + 8
        halfH = h * 0.9 + 8

        x0 = max(0, int(cx - halfW))
        y0 = max(0, int(cy - halfH))
        x1 = min(width, int(cx + halfW))
        y1 = min(height, int(cy + halfH))

        if x1 - x0 < 16 or y1 - y0 < 16:
            return None
        return (x0, y0, x1 - x0, y1 - y0)

    def rect(self):
        if not self.confirmed or self.state is None:
            return None
        return (
            int(round(self.state[0])),
            int(round(self.state[1])),
            max(1, int(round(self.state[2]))),
            max(1, int(round(self.state[3]))),
        )


class ColorDetector(object):
    """一帧检测 + 多色跟踪；对外只给一个主颜色编号。"""

    def __init__(self):
        self.trackers = [
            ColorTracker(label, colorId, lower, upper, color)
            for label, colorId, lower, upper, color in COLOR_DEFS
        ]
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
        )

    def reset(self):
        for tracker in self.trackers:
            tracker.reset()

    def process(self, frame, collectDebug=False):
        """处理一帧 BGR 图像，返回 (result, debug)。"""
        if frame is None or getattr(frame, "ndim", 0) != 3 or frame.shape[2] != 3:
            return _empty_result(), {
                "roi": [],
                "candidates": [],
                "scale": 1.0,
                "proc_size": (0, 0),
            }

        srcH, srcW = frame.shape[0], frame.shape[1]
        if srcW <= 0 or srcH <= 0:
            return _empty_result(), {
                "roi": [],
                "candidates": [],
                "scale": 1.0,
                "proc_size": (0, 0),
            }

        if PROC_WIDTH and srcW > PROC_WIDTH:
            scale = float(PROC_WIDTH) / float(srcW)
            procW = max(1, int(round(srcW * scale)))
            procH = max(1, int(round(srcH * scale)))
            small = cv2.resize(
                frame, (procW, procH), interpolation=cv2.INTER_LINEAR
            )
        else:
            scale = 1.0
            procW, procH = srcW, srcH
            small = frame

        if USE_GAUSSIAN:
            blur = cv2.GaussianBlur(small, GAUSSIAN_KERNEL, 0)
        else:
            blur = small
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        minArea = max(MIN_AREA_ABS, int(procW * procH * MIN_AREA_RATIO))
        invScale = 1.0 / scale if scale else 1.0

        trackersOut = []
        candidatesOut = []
        rois = []
        detected = 0

        for tracker in self.trackers:
            roi = tracker.roi(procW, procH)
            if roi is None:
                rect = findDetect(
                    hsv, tracker.lower, tracker.upper, minArea, self.kernel
                )
            else:
                rois.append((tracker.label, roi))
                x0, y0, rw, rh = roi
                rect = findDetect(
                    hsv[y0 : y0 + rh, x0 : x0 + rw],
                    tracker.lower,
                    tracker.upper,
                    minArea,
                    self.kernel,
                    offset=(x0, y0),
                )
                if rect is None:
                    # 快速运动时目标可能跑出 ROI，立即全图兜底
                    rect = findDetect(
                        hsv, tracker.lower, tracker.upper, minArea, self.kernel
                    )

            if rect is not None:
                detected += 1
            tracker.update(rect)

            if collectDebug and rect is not None:
                candidatesOut.append(
                    {
                        "label": tracker.label,
                        "color": tracker.colorId,
                        "rect": _to_src(rect, invScale),
                        "confirmed": tracker.confirmed,
                    }
                )

            if tracker.confirmed and tracker.state is not None:
                r = tracker.rect()
                if r is not None:
                    votes = tracker.votes()
                    trackersOut.append(
                        {
                            "label": tracker.label,
                            "color": tracker.colorId,
                            "rect": _to_src(r, invScale),
                            "votes": votes,
                            "confidence": _confidence(votes),
                            "bgr": tracker.color,
                        }
                    )

        # 多色同时确认时，取框面积最大的作为主颜色
        primary = None
        primaryArea = -1
        for item in trackersOut:
            _x, _y, w, h = item["rect"]
            area = w * h
            if area > primaryArea:
                primaryArea = area
                primary = item

        if primary is None:
            result = _empty_result()
        else:
            x, y, w, h = primary["rect"]
            result = {
                "color": primary["color"],
                "label": primary["label"],
                "rect": primary["rect"],
                "cx": x + w // 2,
                "cy": y + h // 2,
                "w": w,
                "h": h,
                "confidence": primary["confidence"],
                "confirmed": True,
                "trackers": trackersOut,
                "candidate_count": detected,
                "fps": 0.0,
                "proc_ms": 0.0,
            }

        debug = {
            "roi": rois,
            "candidates": candidatesOut,
            "scale": scale,
            "proc_size": (procW, procH),
        }
        return result, debug


def drawResult(frame, result, debug=None, debugMode=False):
    """把颜色检测结果画到帧上。debugMode=True 时画 ROI 和未确认候选。"""
    if debug is None:
        debug = {}

    if debugMode:
        scale = debug.get("scale", 1.0)
        invScale = 1.0 / scale if scale else 1.0
        for _label, roi in debug.get("roi", []):
            x, y, w, h = _to_src(roi, invScale)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 1)

        for cand in debug.get("candidates", []):
            if cand.get("confirmed"):
                continue
            x, y, w, h = cand["rect"]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (128, 128, 128), 1)
            cv2.putText(
                frame,
                cand["label"],
                (x, max(12, y - 2)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (200, 200, 200),
                1,
            )

    for item in result.get("trackers", []):
        drawDetect(
            "%s %d%%" % (item["label"], item["confidence"]),
            item["rect"],
            frame,
            item["bgr"],
            1.0,
        )

    if not result.get("confirmed") and debugMode:
        cv2.putText(
            frame,
            "none",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
