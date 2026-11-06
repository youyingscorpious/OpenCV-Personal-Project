"""红色八角 STOP 牌识别共享核心。

PC（桌面 OpenCV + numpy）和庐山派 K230（CanMV cv2 绑定 + ulab.numpy）
共用本文件的算法与参数。入口文件只负责摄像头/显示/键盘/通信钩子。

坐标约定：所有 rect 都是 (x, y, w, h)，原点在画面左上角，单位为像素。
"""

import cv2

try:
    from ulab import numpy as np
except ImportError:
    import numpy as np


# ==================== 可调参数 ====================
PROC_WIDTH = 640
PROC_HEIGHT = 480

# 红色在 HSV 中跨越 0°，必须两段
RED_LOWER1 = (0, 90, 50)
RED_UPPER1 = (10, 255, 255)
RED_LOWER2 = (160, 90, 50)
RED_UPPER2 = (180, 255, 255)

# 板端传感器是 RGB888，但 CanMV 的 cv2 示例按 BGR 使用；若红色 mask 总是空的，
# 把这里改成 "RGB"（绘制颜色也会跟着交换）。
COLOR_ORDER = "BGR"

# 预处理
USE_GAUSSIAN = True
GAUSSIAN_KERNEL = (3, 3)
MORPH_KERNEL_SIZE = 3  # 噪声大时可改成 5；越小距离估计越准

# 候选筛选
MIN_AREA_RATIO = 0.0025  # 最小面积占整帧比例（640x480 下约 768 px）
MAX_AREA_RATIO = 0.45  # 未确认时限制大块红色；场景干净可放宽
MAX_AREA_RATIO_CONFIRMED = 0.75  # 已确认跟踪后允许牌子贴近、变大
MIN_ASPECT = 0.60
MAX_ASPECT = 1.65
MIN_EXTENT = 0.52  # 轮廓面积 / 外接矩形面积
MIN_SOLIDITY = 0.82  # 轮廓面积 / 凸包面积
APPROX_EPSILON = 0.025  # approxPolyDP 的 epsilon / 周长
MIN_VERTICES = 6
MAX_VERTICES = 10

# 白边/白字验证
WHITE_CHECK_HARD = True  # True：白区不足直接拒绝；False：只作为加分项
WHITE_MIN_RATIO = 0.015  # 白区像素 / 轮廓面积
WHITE_LOWER = (0, 0, 90)
WHITE_UPPER = (180, 110, 255)

# 多帧确认与跟踪
WINDOW = 10  # 投票窗口帧数
MIN_VOTES = 6  # 窗口内至少命中几帧才确认
LOST_TOLERANCE = 6  # 已确认后连续丢帧几帧才取消
SMOOTH_ALPHA = 0.65  # 中心位置平滑
SMOOTH_BETA = 0.30  # 中心速度修正
SIZE_ALPHA = 0.50  # 宽高平滑
SIZE_BETA = 0.15  # 宽高速度修正（车子接近时框会快速变大）

# ROI
ROI_SCALE = 1.8  # ROI 宽度约为框宽的 1.8 倍
ROI_MARGIN = 16
ROI_MIN_SIZE = 24
ROI_PREDICT = 2.0  # 用几帧的速度预测 ROI 中心

# 阈值数组只创建一次，避免每帧分配
RED_LO1 = np.array(RED_LOWER1, dtype=np.uint8)
RED_HI1 = np.array(RED_UPPER1, dtype=np.uint8)
RED_LO2 = np.array(RED_LOWER2, dtype=np.uint8)
RED_HI2 = np.array(RED_UPPER2, dtype=np.uint8)
WHITE_LO = np.array(WHITE_LOWER, dtype=np.uint8)
WHITE_HI = np.array(WHITE_UPPER, dtype=np.uint8)

KERNEL = cv2.getStructuringElement(
    cv2.MORPH_RECT, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
)


def _color(bgr):
    """绘制颜色：输入按 BGR 习惯写，RGB 模式下自动交换 R/B。"""
    if COLOR_ORDER == "RGB":
        return (bgr[2], bgr[1], bgr[0])
    return bgr


def redMask(view):
    """HSV 红色双段掩码。"""
    m1 = cv2.inRange(view, RED_LO1, RED_HI1)
    m2 = cv2.inRange(view, RED_LO2, RED_HI2)
    out = cv2.bitwise_or(m1, m2)
    # K230D 堆很小，显式释放两段掩码，降低峰值
    del m1, m2
    return out


def _shiftedContour(contour, dx, dy):
    """把轮廓平移到以 (dx,dy) 为原点的小图上。"""
    pts = []
    for p in contour:
        try:
            px, py = p[0][0], p[0][1]
        except (TypeError, IndexError):
            px, py = p[0], p[1]
        pts.append((int(px) - dx, int(py) - dy))
    return np.array(pts, dtype=np.int32).reshape((-1, 1, 2))


def whiteRatio(view, contour, rect, area):
    """红色八角形内部的白边/白字占比。

    view 是 HSV 图（可能是 ROI 子图），contour 的坐标也是该子图坐标。
    K230D 的 MicroPython 堆很小：mask 只按外接矩形分配，绝不按整帧分配。
    优先用 drawContours 的 offset 平移轮廓；CanMV 不支持时退回中心区域统计。
    """
    x, y, w, h = rect
    if w <= 0 or h <= 0 or area <= 0:
        return 0.0

    roi = view[y : y + h, x : x + w]
    if roi.shape[0] != h or roi.shape[1] != w:
        return 0.0

    white = None
    inside = None
    try:
        cmask = np.zeros((h, w), dtype=np.uint8)
        try:
            cv2.drawContours(cmask, [contour], -1, 255, -1, offset=(-x, -y))
        except Exception:
            # 部分 CanMV cv2 不支持 offset；手工平移后画到小 mask
            shifted = _shiftedContour(contour, x, y)
            cv2.drawContours(cmask, [shifted], -1, 255, -1)
        white = cv2.inRange(roi, WHITE_LO, WHITE_HI)
        inside = cv2.bitwise_and(white, cmask)
        count = cv2.countNonZero(inside)
    except Exception:
        # 最后兜底：只统计框中心 78% 区域，仍然只分配小数组
        mx = int(w * 0.11)
        my = int(h * 0.11)
        if w - 2 * mx < 4 or h - 2 * my < 4:
            return 0.0
        center = roi[my : h - my, mx : w - mx]
        white = cv2.inRange(center, WHITE_LO, WHITE_HI)
        count = cv2.countNonZero(white)
    return count / float(area)


def scoreCandidate(w, h, extent, solidity, vertices, white):
    """把形状指标压成一个 0~1 的启发式分数，用于选目标和显示置信度。"""
    aspect = w / float(h) if h > 0 else 0.0

    # 八边形最理想是 8 个顶点；模糊/透视下 6~10 都可接受
    vscore = 1.0 if vertices == 8 else max(0.35, 1.0 - abs(vertices - 8) * 0.18)
    ascore = max(0.0, 1.0 - abs(aspect - 1.0) / 0.65)
    escore = min(1.0, max(0.0, (extent - 0.50) / 0.33))
    sscore = min(1.0, max(0.0, (solidity - 0.80) / 0.20))
    wscore = min(1.0, white / 0.08)

    """加权算分，vscore/ascore/extent/solidity/white 分别占 30%/20%/15%/15%/20%。"""
    return 0.30 * vscore + 0.20 * ascore + 0.15 * escore + 0.15 * sscore + 0.20 * wscore


def findStopCandidates(
    view, fullArea, offset=(0, 0), collectDebug=False, maxAreaRatio=MAX_AREA_RATIO
):
    """在 HSV 图（或 ROI 子图）中找红色八角候选。

    返回 (candidates, mask, rejected)。
    candidates 里的 rect 已经加上 offset，是整图坐标。
    """
    mask = redMask(view)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not collectDebug:
        # 实时模式不再需要整帧 mask，尽早释放给后面的候选验证
        mask = None

    candidates = []
    rejected = []
    ox, oy = offset

    for c in contours:
        area = cv2.contourArea(c)
        if area < fullArea * MIN_AREA_RATIO or area > fullArea * maxAreaRatio:
            if collectDebug and len(rejected) < 8:
                x, y, w, h = cv2.boundingRect(c)
                reason = "small" if area < fullArea * MIN_AREA_RATIO else "area"
                rejected.append({"rect": (x + ox, y + oy, w, h), "reason": reason})
            continue

        x, y, w, h = cv2.boundingRect(c)
        if w <= 0 or h <= 0:
            continue

        reason = None
        aspect = w / float(h)
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            reason = "aspect"

        extent = area / float(w * h)
        if reason is None and extent < MIN_EXTENT:
            reason = "extent"

        hull = cv2.convexHull(c)
        hullArea = cv2.contourArea(hull)
        solidity = area / hullArea if hullArea > 0 else 0.0
        if reason is None and solidity < MIN_SOLIDITY:
            reason = "solidity"

        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, APPROX_EPSILON * peri, True)
        vcount = len(approx)
        if reason is None and (vcount < MIN_VERTICES or vcount > MAX_VERTICES):
            reason = "vertices"
        if reason is None and not cv2.isContourConvex(approx):
            reason = "convex"

        white = 0.0
        if reason is None:
            white = whiteRatio(view, c, (x, y, w, h), area)
            if WHITE_CHECK_HARD and white < WHITE_MIN_RATIO:
                reason = "white"

        if reason is not None:
            if collectDebug and len(rejected) < 8:
                rejected.append({"rect": (x + ox, y + oy, w, h), "reason": reason})
            continue

        score = scoreCandidate(w, h, extent, solidity, vcount, white)
        candidates.append(
            {
                "rect": (x + ox, y + oy, w, h),
                "score": score,
                "area": area,
                "vertices": vcount,
                "extent": extent,
                "solidity": solidity,
                "white": white,
                "aspect": aspect,
            }
        )

    return candidates, mask, rejected


def candidateCenter(candidate):
    x, y, w, h = candidate["rect"]
    return (x + w * 0.5, y + h * 0.5)


def selectCandidate(candidates, predicted):
    """选最佳候选。已确认时优先选靠近预测中心的，避免框在多个红块间跳。"""
    if not candidates:
        return None

    if predicted is not None:
        px, py = predicted
        best = None
        for c in candidates:
            cx, cy = candidateCenter(c)
            limit = max(c["rect"][2], c["rect"][3]) * 0.9
            if abs(cx - px) <= limit and abs(cy - py) <= limit:
                if best is None or c["score"] > best["score"]:
                    best = c
        # 已确认时宁可当帧漏检，也不跳到画面里另一个红色物体
        return best

    best = candidates[0]
    for c in candidates[1:]:
        if c["score"] > best["score"]:
            best = c
    return best


def countVotes(history):
    n = 0
    for r in history:
        if r is not None:
            n += 1
    return n


def voteRect(history):
    x = y = w = h = 0.0
    n = 0
    for r in history:
        if r is None:
            continue
        x += r[0]
        y += r[1]
        w += r[2]
        h += r[3]
        n += 1
    if n == 0:
        return None
    return (x / n, y / n, w / n, h / n)


class StopTracker:
    """单目标跟踪：多帧投票确认 + α-β 滤波（带速度预测）。"""

    def __init__(self):
        self.history = []
        self.confirmed = False
        self.miss = 0
        self.state = None
        self.velocity = [0.0, 0.0, 0.0, 0.0]
        self.lastScore = 0.0

    def reset(self):
        self.history = []
        self.confirmed = False
        self.miss = 0
        self.state = None
        self.velocity = [0.0, 0.0, 0.0, 0.0]
        self.lastScore = 0.0

    def predictedCenter(self):
        if not self.confirmed or self.state is None:
            return None
        x, y, w, h = self.state
        return (
            x + w * 0.5 + self.velocity[0],
            y + h * 0.5 + self.velocity[1],
        )

    def roi(self, width, height):
        """根据滤波结果 + 速度预测生成下一帧的检测 ROI。"""
        if not self.confirmed or self.state is None:
            return None

        x, y, w, h = self.state
        cx = x + w * 0.5 + self.velocity[0] * ROI_PREDICT
        cy = y + h * 0.5 + self.velocity[1] * ROI_PREDICT
        halfW = w * ROI_SCALE * 0.5 + ROI_MARGIN
        halfH = h * ROI_SCALE * 0.5 + ROI_MARGIN

        x0 = int(max(0, cx - halfW))
        y0 = int(max(0, cy - halfH))
        x1 = int(min(width, cx + halfW))
        y1 = int(min(height, cy + halfH))

        if x1 - x0 < ROI_MIN_SIZE or y1 - y0 < ROI_MIN_SIZE:
            return None
        return (x0, y0, x1 - x0, y1 - y0)

    def update(self, rect, score):
        if len(self.history) >= WINDOW:
            self.history.pop(0)
        self.history.append(rect)

        if rect is None:
            self.miss += 1
        else:
            self.miss = 0

        if not self.confirmed:
            if rect is None or countVotes(self.history) < MIN_VOTES:
                return False
            seed = voteRect(self.history)
            if seed is None:
                return False
            self.state = [
                float(seed[0]),
                float(seed[1]),
                float(seed[2]),
                float(seed[3]),
            ]
            self.velocity = [0.0, 0.0, 0.0, 0.0]
            self.confirmed = True
            self.miss = 0
            self.lastScore = score
            return True

        if self.miss >= LOST_TOLERANCE:
            self.reset()
            return False

        if rect is None:
            # 短暂丢帧：按速度外推，速度衰减防止漂移
            self.state = [s + v for s, v in zip(self.state, self.velocity)]
            self.velocity = [v * 0.8 for v in self.velocity]
            self.lastScore = max(0.0, self.lastScore * 0.98)
            self._clamp()
            return True

        # α-β 滤波：先预测，再用本帧测量修正
        z = [float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3])]
        pred = [s + v for s, v in zip(self.state, self.velocity)]
        err = [zi - pi for zi, pi in zip(z, pred)]
        alpha = (SMOOTH_ALPHA, SMOOTH_ALPHA, SIZE_ALPHA, SIZE_ALPHA)
        beta = (SMOOTH_BETA, SMOOTH_BETA, SIZE_BETA, SIZE_BETA)
        self.state = [p + a * e for p, a, e in zip(pred, alpha, err)]
        self.velocity = [v + b * e for v, b, e in zip(self.velocity, beta, err)]
        self.lastScore = score
        self._clamp()
        return True

    def _clamp(self):
        if self.state[2] < 1.0:
            self.state[2] = 1.0
        if self.state[3] < 1.0:
            self.state[3] = 1.0

    def rect(self):
        if not self.confirmed or self.state is None:
            return None
        return (
            int(round(self.state[0])),
            int(round(self.state[1])),
            max(1, int(round(self.state[2]))),
            max(1, int(round(self.state[3]))),
        )


class StopDetector:
    """一帧检测 + 多帧确认/跟踪。"""

    def __init__(self):
        self.tracker = StopTracker()

    def reset(self):
        self.tracker.reset()

    def process(self, frame, collectDebug=False):
        """处理一帧 BGR/RGB 图像，返回 (result, debug)。"""
        if USE_GAUSSIAN:
            blur = cv2.GaussianBlur(frame, GAUSSIAN_KERNEL, 0)
        else:
            blur = frame

        if COLOR_ORDER == "RGB":
            hsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)
        else:
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        if USE_GAUSSIAN:
            # HSV 已生成，模糊图不再需要；K230D 上这是整整一帧 RGB
            del blur

        fullArea = frame.shape[0] * frame.shape[1]
        maxRatio = (
            MAX_AREA_RATIO_CONFIRMED if self.tracker.confirmed else MAX_AREA_RATIO
        )
        roi = self.tracker.roi(frame.shape[1], frame.shape[0])

        candidates = []
        mask = None
        rejected = []
        usedRoi = None

        if roi is not None:
            x0, y0, rw, rh = roi
            view = hsv[y0 : y0 + rh, x0 : x0 + rw]
            candidates, mask, rejected = findStopCandidates(
                view, fullArea, (x0, y0), collectDebug, maxRatio
            )
            usedRoi = roi
            if not candidates:
                # 快速运动时目标可能跑出 ROI，立即全图兜底
                candidates, mask, rejected = findStopCandidates(
                    hsv, fullArea, (0, 0), collectDebug, maxRatio
                )
                usedRoi = None
        else:
            candidates, mask, rejected = findStopCandidates(
                hsv, fullArea, (0, 0), collectDebug, maxRatio
            )

        best = selectCandidate(candidates, self.tracker.predictedCenter())
        bestRect = best["rect"] if best is not None else None
        bestScore = best["score"] if best is not None else 0.0

        confirmed = self.tracker.update(bestRect, bestScore)
        rect = self.tracker.rect() if confirmed else None

        result = {
            "stop": bool(confirmed),
            "rect": rect,
            "cx": -1,
            "cy": -1,
            "w": 0,
            "h": 0,
            "confidence": (
                min(100, int(round(self.tracker.lastScore * 100))) if confirmed else 0
            ),
            "fps": 0.0,
            "proc_ms": 0.0,
            "candidate_count": len(candidates),
            "candidate_score": (
                min(100, int(round(bestScore * 100))) if best is not None else 0
            ),
        }

        if rect is not None:
            result["cx"] = rect[0] + rect[2] // 2
            result["cy"] = rect[1] + rect[3] // 2
            result["w"] = rect[2]
            result["h"] = rect[3]

        debug = {
            "roi": usedRoi,
            "mask": mask if collectDebug else None,
            "rejected": rejected if collectDebug else [],
            "best": best,
            "best_rect": bestRect,
            "candidate_count": len(candidates),
        }

        return result, debug


def _label(frame, text, x, y):
    """带黑色底的文字，避免背景颜色干扰读数。"""
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    ty = y - 6
    if ty - th < 0:
        ty = y + th + 8
    cv2.rectangle(
        frame,
        (x, ty - th - 4),
        (x + tw + 4, ty + base),
        _color((0, 0, 0)),
        -1,
    )
    cv2.putText(
        frame,
        text,
        (x + 2, ty),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        _color((255, 255, 255)),
        2,
    )


def drawResult(frame, result, debug=None, debugMode=False):
    """把结果画到整帧上。实时路径用 debugMode=False，只画框/中心和置信度。"""
    if debug is None:
        debug = {}

    if debugMode and debug.get("roi") is not None:
        x, y, w, h = debug["roi"]
        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            _color((255, 0, 0)),
            1,
        )

    if debugMode:
        for r in debug.get("rejected", []):
            x, y, w, h = r["rect"]
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                _color((128, 128, 128)),
                1,
            )
            cv2.putText(
                frame,
                r["reason"],
                (x, max(12, y - 2)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                _color((200, 200, 200)),
                1,
            )

    if not result["stop"] and debug.get("best") is not None:
        best = debug["best"]
        x, y, w, h = best["rect"]
        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            _color((0, 255, 255)),
            2,
        )
        _label(frame, "CAND %d" % int(round(best["score"] * 100)), x, y)

    if result["stop"] and result["rect"] is not None:
        x, y, w, h = result["rect"]
        color = _color((0, 255, 0))
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

        cx = x + w // 2
        cy = y + h // 2
        cv2.circle(frame, (cx, cy), 5, color, -1)

        _label(frame, "STOP %d%%" % result["confidence"], x, y)

    if debugMode:
        info = "cand:%d" % debug.get("candidate_count", 0)
        cv2.putText(
            frame,
            info,
            (8, frame.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            _color((255, 255, 255)),
            1,
        )
