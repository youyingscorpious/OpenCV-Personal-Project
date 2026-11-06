# -*- coding: utf-8 -*-
"""导航组对外视觉 API。

用法::

    import nav_vision as nv

    a = nv.color()    # "1" / "2" / "3" / "0" / "fuck"
    b = nv.stop()     # "1" / "0" / "fuck"
    print(nv.status())

广播不可用时，默认会对本机 /24 做单播扫描，一般不需要手动指定 IP。
想强制直连，在首次 color()/stop()/start() 之前::

    nv.set_board_ip("192.168.145.222")

也可以设置环境变量 ``NAV_VISION_BOARD_IP``，效果相同。
不想扫描就设 ``NAV_VISION_SCAN_SUBNET=off``；要指定网段就填
``192.168.145.0/24``。

返回值全部是字符串：

    color: "1"=红, "2"=黄, "3"=蓝, "0"=当前无目标, "fuck"=链路/线程故障
    stop : "1"=检测到 STOP, "0"=当前无 STOP, "fuck"=链路/线程故障

注意：非空字符串 "0" 和 "fuck" 在 Python 里都是真值。判断必须写::

    if a == "1":
        ...

不要写 ``if a:``。

模块首次调用 color()/stop() 时自动启动一个后台读取线程：
  * 只读通道 1（CSI1，前视，color 和 STOP 共用）
  * 每帧顺序跑 color、stop 两个 detector
  * 任一 detector 异常只影响自己，不影响另一个
  * 超过 STALE_RESULT_S 没有新结果返回 "0"；超过 DEAD_STREAM_S 没有新帧返回 "fuck"
"""

import atexit
import os
import sys
import threading
import time

# 让同目录的 color_core / stop_sign_core / pc_receiver 可直接 import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import color_core
    import pc_receiver
    import stop_sign_core
except ImportError as exc:
    raise ImportError(
        "nav_vision 需要 opencv-python 和 numpy："
        "pip install opencv-python numpy"
    ) from exc


# ==================== 对外常量 ====================
CHANNEL = 1

FAIL_TEXT = "fuck"
NO_TARGET = "0"
COLOR_NONE = "0"
COLOR_RED = "1"
COLOR_YELLOW = "2"
COLOR_BLUE = "3"
STOP_NO = "0"
STOP_YES = "1"

STALE_RESULT_S = 0.5      # 识别结果超过这么久没更新 -> "0"
DEAD_STREAM_S = 2.0       # 完全没有新帧超过这么久 -> "fuck"
FRAME_TIMEOUT_S = 1.0     # 单次等帧超时

# 广播不可用时，可在 import 前设置环境变量，或用 set_board_ip()
DEFAULT_BOARD_IP = os.environ.get("NAV_VISION_BOARD_IP", "").strip()
# 没有指定 board_ip 时，默认做单播扫描；设成 "off" 可关闭
# 默认同时扫本机 /24 和当前热点常见的 192.168.145.0/24
SCAN_SUBNET = os.environ.get(
    "NAV_VISION_SCAN_SUBNET", "auto,192.168.145.0/24"
).strip() or None
# ==================================================


class _Slot(object):
    """一个检测结果槽：值 + 时间戳 + 最近错误。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._value = NO_TARGET
        self._ts = 0.0
        self._err = None

    def set(self, value, ts, err=None):
        with self._lock:
            self._value = value
            self._ts = ts
            self._err = err

    def get(self, max_age):
        with self._lock:
            value = self._value
            ts = self._ts
            err = self._err
        if value is None or ts <= 0.0 or (time.perf_counter() - ts) > max_age:
            return NO_TARGET, err
        return value, err

    def peek(self):
        with self._lock:
            return self._value, self._ts, self._err

    def reset(self):
        with self._lock:
            self._value = NO_TARGET
            self._ts = 0.0
            self._err = None


class _VisionWorker(object):
    """单通道读取线程：一帧跑两个 detector，结果放各自的槽。"""

    def __init__(self):
        self._lock = threading.RLock()
        self._thread = None
        self._running = False

        self._want_color = False
        self._want_stop = False

        self._color_slot = _Slot()
        self._stop_slot = _Slot()

        self._color_det = None
        self._stop_det = None

        self._stream = None
        self._board_ip = DEFAULT_BOARD_IP or None
        self._last_frame_ts = 0.0
        self._started_ts = 0.0
        self._last_error = None
        self._frames = 0
        self._fps = 0.0
        self._fps_ts = 0.0
        self._fps_count = 0

    # ---- 生命周期 ----
    def ensure(self, want_color=False, want_stop=False):
        with self._lock:
            if want_color:
                self._want_color = True
            if want_stop:
                self._want_stop = True
            if self._running:
                return
            self._running = True
            self._started_ts = time.perf_counter()
            self._thread = threading.Thread(
                target=self._loop, name="nav-vision", daemon=True
            )
            self._thread.start()

    def set_board_ip(self, ip):
        """指定板子 IP；必须在首次 color()/stop()/start() 之前调用。"""
        with self._lock:
            if self._running:
                raise RuntimeError(
                    "set_board_ip() 必须在首次 color()/stop()/start() 之前调用；"
                    "运行中需要改 IP 请先 close()"
                )
            text = "" if ip is None else str(ip).strip()
            self._board_ip = text or None

            # 如果 pc_receiver 已经建过 transport，尽量把新 IP 写进去；
            # 最稳妥的用法仍然是启动前设置。
            transport = getattr(pc_receiver, "_transport", None)
            if transport is not None:
                transport._manual_ip = self._board_ip
                if not transport.running:
                    transport.board_ip = self._board_ip

    def close(self):
        with self._lock:
            self._running = False
            thread = self._thread
            self._thread = None
            stream = self._stream
            self._stream = None

        if stream is not None:
            try:
                stream.close()
            except Exception:
                pass

        if thread is not None and thread is not threading.current_thread():
            thread.join(1.0)

        with self._lock:
            self._want_color = False
            self._want_stop = False
            self._last_frame_ts = 0.0
            self._started_ts = 0.0
            self._last_error = None
            self._frames = 0
            self._fps = 0.0
            self._fps_ts = 0.0
            self._fps_count = 0
        self._color_slot.reset()
        self._stop_slot.reset()
        with self._lock:
            # 下次启动重新建 detector，避免把旧的跟踪状态带进来
            self._color_det = None
            self._stop_det = None

    # ---- 读取循环 ----
    def _is_running(self):
        with self._lock:
            return self._running

    def _get_stream(self):
        with self._lock:
            view = self._stream
            if view is not None and not view.closed:
                return view
            board_ip = self._board_ip

        kwargs = {}
        if board_ip:
            kwargs["board_ip"] = board_ip
        elif SCAN_SUBNET:
            kwargs["scan_subnet"] = SCAN_SUBNET
        view = pc_receiver.receiver(CHANNEL, **kwargs)

        with self._lock:
            self._stream = view
        return view

    def _loop(self):
        while self._is_running():
            try:
                stream = self._get_stream()
            except Exception as exc:
                self._set_error("open", exc)
                time.sleep(0.5)
                continue

            try:
                frame = stream(timeout=FRAME_TIMEOUT_S)
            except TimeoutError as exc:
                self._set_error("frame", exc)
                # 正常情况下 stream() 已经等了 1s；这里再让一点，避免异常流忙转
                time.sleep(0.02)
                continue
            except RuntimeError as exc:
                # stream.close() 或 transport 重启；下一轮重新取
                with self._lock:
                    self._stream = None
                self._set_error("frame", exc)
                time.sleep(0.2)
                continue
            except Exception as exc:
                self._set_error("frame", exc)
                time.sleep(0.2)
                continue

            now = time.perf_counter()
            with self._lock:
                self._last_frame_ts = now
                self._frames += 1
                self._last_error = None
                want_color = self._want_color
                want_stop = self._want_stop
            self._tick_fps(now)

            if want_color:
                self._run_color(frame, now)
            if want_stop:
                self._run_stop(frame, now)

    def _run_color(self, frame, now):
        if self._color_det is None:
            self._color_det = color_core.ColorDetector()
        try:
            result, _debug = self._color_det.process(frame, collectDebug=False)
            self._color_slot.set(str(result.get("color", color_core.COLOR_NONE)), now)
        except Exception as exc:
            self._set_error("color", exc)
            self._color_slot.set(FAIL_TEXT, now, self._error_text("color", exc))

    def _run_stop(self, frame, now):
        if self._stop_det is None:
            self._stop_det = stop_sign_core.StopDetector()
        try:
            result, _debug = self._stop_det.process(frame, collectDebug=False)
            self._stop_slot.set(STOP_YES if result.get("stop") else STOP_NO, now)
        except Exception as exc:
            self._set_error("stop", exc)
            self._stop_slot.set(FAIL_TEXT, now, self._error_text("stop", exc))

    def _tick_fps(self, now):
        with self._lock:
            if self._fps_ts <= 0.0:
                self._fps_ts = now
                self._fps_count = 0
            self._fps_count += 1
            dt = now - self._fps_ts
            if dt >= 1.0:
                self._fps = self._fps_count / dt
                self._fps_count = 0
                self._fps_ts = now

    # ---- 错误 ----
    @staticmethod
    def _error_text(where, exc):
        return "%s: %s: %s" % (where, type(exc).__name__, exc)

    def _set_error(self, where, exc):
        with self._lock:
            self._last_error = self._error_text(where, exc)

    # ---- 结果 ----
    def _value(self, slot):
        now = time.perf_counter()
        with self._lock:
            last_frame = self._last_frame_ts
            started = self._started_ts

        if last_frame <= 0.0:
            # 刚启动时给一个宽限期，避免第一帧还没到就报 fuck
            if started > 0.0 and (now - started) > DEAD_STREAM_S:
                return FAIL_TEXT
            return NO_TARGET
        if (now - last_frame) > DEAD_STREAM_S:
            return FAIL_TEXT

        value, _err = slot.get(STALE_RESULT_S)
        return value

    def color_value(self):
        return self._value(self._color_slot)

    def stop_value(self):
        return self._value(self._stop_slot)

    def snapshot(self):
        now = time.perf_counter()
        with self._lock:
            running = self._running
            want_color = self._want_color
            want_stop = self._want_stop
            last_frame = self._last_frame_ts
            err = self._last_error
            fps = self._fps
            frames = self._frames
            stream = self._stream
            configured_ip = self._board_ip

        board_ip = None
        board_status = None
        if stream is not None:
            try:
                board_ip = stream.board_ip
            except Exception:
                pass
            try:
                board_status = stream.board_status
            except Exception:
                pass

        color_value, color_ts, color_err = self._color_slot.peek()
        stop_value, stop_ts, stop_err = self._stop_slot.peek()

        return {
            "running": running,
            "channel": CHANNEL,
            "board_ip": board_ip if board_ip is not None else configured_ip,
            "configured_board_ip": configured_ip,
            "scan_subnet": SCAN_SUBNET,
            "board_status": board_status,
            "fps": round(fps, 1),
            "frames": frames,
            "color_active": want_color,
            "stop_active": want_stop,
            "last_frame_age_ms": None
            if last_frame <= 0.0
            else int((now - last_frame) * 1000),
            "last_color": color_value,
            "last_color_age_ms": None
            if color_ts <= 0.0
            else int((now - color_ts) * 1000),
            "last_color_error": color_err,
            "last_stop": stop_value,
            "last_stop_age_ms": None
            if stop_ts <= 0.0
            else int((now - stop_ts) * 1000),
            "last_stop_error": stop_err,
            "last_error": err,
        }


_worker = _VisionWorker()


# ==================== 模块级 API ====================
def color():
    """返回颜色编号字符串："1" 红 / "2" 黄 / "3" 蓝 / "0" 无 / "fuck" 故障。"""
    _worker.ensure(want_color=True)
    return _worker.color_value()


def stop():
    """返回 STOP 状态字符串："1" 有 / "0" 无 / "fuck" 故障。"""
    _worker.ensure(want_stop=True)
    return _worker.stop_value()


def set_board_ip(ip):
    """广播不可用时指定板子 IP；必须在首次 color()/stop()/start() 之前调用。"""
    _worker.set_board_ip(ip)
    return True


def start(color=True, stop=True, board_ip=None):
    """提前启动后台线程（可选）；不调用也会在首次 color()/stop() 时启动。"""
    if board_ip:
        _worker.set_board_ip(board_ip)
    _worker.ensure(want_color=bool(color), want_stop=bool(stop))
    return True


def close():
    """停止后台线程并释放接收端；一般不需要手动调用。"""
    _worker.close()


def status():
    """链路与结果诊断信息。"""
    return _worker.snapshot()


atexit.register(close)
