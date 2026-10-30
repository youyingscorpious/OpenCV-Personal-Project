# -*- coding: utf-8 -*-
"""无硬件自测：合成色卡 / STOP 牌，验证两个核心和 nav_vision 的字符串契约。

运行：
    uv run --with opencv-python --with numpy python selftest.py
"""

import math
import time

import cv2
import numpy as np

import color_core
import nav_vision
import stop_sign_core


W, H = 640, 480


def blank():
    return np.zeros((H, W, 3), dtype=np.uint8)


def card(x, y, w, h, bgr):
    frame = blank()
    cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, -1)
    return frame


def octagon(cx, cy, radius):
    pts = []
    for i in range(8):
        angle = math.pi / 8.0 + i * math.pi / 4.0
        pts.append(
            (
                int(round(cx + radius * math.cos(angle))),
                int(round(cy + radius * math.sin(angle))),
            )
        )
    return np.array(pts, dtype=np.int32)


def stop_frame(blue_large=False):
    frame = blank()
    if blue_large:
        cv2.rectangle(frame, (400, 20), (640, 260), (255, 0, 0), -1)
    pts = octagon(320, 240, 110)
    cv2.fillPoly(frame, [pts], (0, 0, 255))
    cv2.putText(
        frame, "STOP", (245, 265), cv2.FONT_HERSHEY_SIMPLEX, 1.8,
        (255, 255, 255), 6,
    )
    return frame


def check(name, cond, detail=""):
    if cond:
        print("PASS", name)
    else:
        print("FAIL", name, detail)
        raise SystemExit(1)


def test_color():
    cases = [
        ("red", card(60, 60, 120, 120, (0, 0, 255)), 1),
        ("yellow", card(60, 60, 120, 120, (0, 255, 255)), 2),
        ("blue", card(60, 60, 120, 120, (255, 0, 0)), 3),
    ]
    for name, frame, expected in cases:
        det = color_core.ColorDetector()
        result = None
        for _ in range(10):
            result, _debug = det.process(frame, collectDebug=True)
        check(
            "color_%s" % name,
            result is not None and result["color"] == expected,
            result,
        )

    # 多色：蓝块最大，应返回 3
    frame = blank()
    cv2.rectangle(frame, (40, 40), (160, 160), (0, 0, 255), -1)
    cv2.rectangle(frame, (220, 40), (300, 120), (0, 255, 255), -1)
    cv2.rectangle(frame, (380, 40), (540, 200), (255, 0, 0), -1)
    det = color_core.ColorDetector()
    result = None
    for _ in range(10):
        result, _debug = det.process(frame, collectDebug=True)
    check("color_largest", result["color"] == 3, result)
    check("color_trackers", len(result["trackers"]) >= 2, result["trackers"])


def test_stop():
    det = stop_sign_core.StopDetector()
    result = None
    debug = None
    for _ in range(12):
        result, debug = det.process(stop_frame(), collectDebug=True)
    check("stop_detected", result["stop"] is True, result)
    check("stop_rect", result["rect"] is not None, result)
    check("stop_confidence", result["confidence"] > 0, result)


class FakeStream(object):
    def __init__(self, frame_fn):
        self.frame_fn = frame_fn
        self.closed = False
        self.board_ip = "127.0.0.1"
        self.board_status = "480P c1=off c2=ok"

    def __call__(self, timeout=None):
        return self.frame_fn()

    def close(self):
        self.closed = True


class FailingStream(FakeStream):
    def __call__(self, timeout=None):
        raise TimeoutError("no frame")


def wait_value(fn, wanted, timeout=3.0):
    deadline = time.time() + timeout
    value = None
    while time.time() < deadline:
        value = fn()
        if value == wanted:
            return value
        time.sleep(0.05)
    return value


def test_nav_vision():
    nav_vision.close()
    fake = FakeStream(lambda: stop_frame(blue_large=True))
    nav_vision.pc_receiver.receiver = lambda *a, **k: fake

    value = wait_value(nav_vision.color, "3", 3.0)
    check("nav_color", value == "3", value)

    value = wait_value(nav_vision.stop, "1", 3.0)
    check("nav_stop", value == "1", value)

    st = nav_vision.status()
    check("nav_status", st["running"] is True and st["channel"] == 2, st)
    check("nav_status_color_active", st["color_active"] is True, st)
    check("nav_status_stop_active", st["stop_active"] is True, st)

    nav_vision.close()
    failing = FailingStream(lambda: None)
    nav_vision.pc_receiver.receiver = lambda *a, **k: failing
    value = wait_value(nav_vision.color, "fuck", 3.5)
    check("nav_fail_string", value == "fuck", value)
    nav_vision.close()


def main():
    test_color()
    test_stop()
    test_nav_vision()
    print("ALL SELFTESTS PASSED")


if __name__ == "__main__":
    main()
