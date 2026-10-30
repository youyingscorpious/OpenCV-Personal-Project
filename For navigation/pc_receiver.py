#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pc_receiver.py — 庐山派 Lite-K230D 双摄接收 API + 演示 CLI

API:
    from pc_receiver import receiver

    video1 = receiver(1)        # PnP，CSI1
    video2 = receiver(2)        # 颜色识别，CSI2

    img1 = video1()             # 阻塞，返回 BGR ndarray
    img2 = video2(timeout=1.0)
    print(video1.last_meta)     # {channel, frame_id, ts_ms, latency_ms, fps}

    video1.try_get()            # 非阻塞，无新帧返回 None
    video1.close()

CLI:
    python pc_receiver.py --channel 1
    python pc_receiver.py --channel 2 --relay

依赖：pip install opencv-python numpy
"""

import argparse
import atexit
import os
import socket
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

MAGIC = b"KV"
VER = 1
# magic(2) ver(1) cam_id(1) frame_id(4) chunk_idx(2) chunk_total(2) ts_ms(4)
HDR_FMT = "<2sBBIHHI"
HDR_LEN = struct.calcsize(HDR_FMT)   # 16
MAX_PKT = 2048
CHANNELS = (1, 2)

FIRST_FRAME_TIMEOUT = 5.0
FRAME_TIMEOUT = 2.0
HELLO_GAP = 1.0
HELLO_AFTER = 2.0
CAL_GAP = 0.15


# --------------------------------------------------------------------------
# 重组
# --------------------------------------------------------------------------
class Reassembler:
    """按 (cam_id) 各自的 frame_id 重组 UDP 分片，只返回完整帧。"""

    def __init__(self, max_pending=4):
        self.max_pending = max_pending
        self.pending = {}
        self.lost = 0
        self.completed = 0

    def clear(self):
        self.pending.clear()

    def add(self, fid, idx, total, ts, payload):
        if total <= 0 or total > 4096 or idx >= total:
            return None

        ent = self.pending.get(fid)
        if ent is None:
            if len(self.pending) >= self.max_pending:
                oldest = next(iter(self.pending))
                old = self.pending.pop(oldest)
                if len(old["chunks"]) < old["total"]:
                    self.lost += 1
            ent = {"total": total, "chunks": {}, "ts": ts}
            self.pending[fid] = ent
        elif ent["total"] != total:
            del self.pending[fid]
            self.lost += 1
            return None

        ent["chunks"][idx] = payload
        if len(ent["chunks"]) == total:
            del self.pending[fid]
            try:
                jpg = b"".join(ent["chunks"][i] for i in range(total))
            except KeyError:
                self.lost += 1
                return None
            self.completed += 1
            return jpg, ent["ts"]
        return None


class _Channel:
    def __init__(self):
        self.reasm = Reassembler()
        self.seq = 0                # 交给调用方的递增序号
        self.frame = None
        self.jpeg = None
        self.fid = 0
        self.ts = 0
        self.latency_ms = None
        self.fps = 0.0
        self.last_ts = None
        self.times = deque(maxlen=120)

    def reset(self):
        self.reasm = Reassembler()
        self.seq = 0
        self.frame = None
        self.jpeg = None
        self.fid = 0
        self.ts = 0
        self.latency_ms = None
        self.fps = 0.0
        self.last_ts = None
        self.times.clear()


# --------------------------------------------------------------------------
# 共享 transport：发现、HELLO、CAL、单 socket 解复用
# --------------------------------------------------------------------------
class _Transport:
    def __init__(self, disc_port=5001, pc_port=5002, board_port=5000,
                 board_ip=None):
        self.disc_port = disc_port
        self.pc_port = pc_port
        self.board_port = board_port

        self.cond = threading.Condition()
        self.channels = {ch: _Channel() for ch in CHANNELS}

        self._manual_ip = board_ip
        self.board_ip = board_ip
        self.board_status = None
        self.offset = None
        self.cal_samples = []
        self.cal_sent = 0
        self.last_cal = 0.0
        self.last_frame = 0.0
        self.last_hello = 0.0

        self.running = False
        self.thread = None
        self.disc = None
        self.data = None

        self._refs = 0
        self._ref_lock = threading.Lock()
        self._state_lock = threading.Lock()

    # ---- 引用计数 ----
    def retain(self):
        with self._ref_lock:
            self._refs += 1

    def release(self):
        with self._ref_lock:
            self._refs -= 1
            need_stop = self._refs <= 0
        if need_stop:
            self.stop()

    # ---- 启停 ----
    def start(self):
        with self._state_lock:
            if self.running:
                return

            with self.cond:
                for ch in self.channels.values():
                    ch.reset()
                self.board_ip = self._manual_ip
                self.board_status = None
                self.offset = None
                self.cal_samples = []
                self.cal_sent = 0
                self.last_cal = 0.0
                self.last_frame = 0.0
                self.last_hello = 0.0

            disc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                disc.bind(("", self.disc_port))
                disc.settimeout(0.0)

                try:
                    data.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
                except OSError:
                    pass
                data.bind(("", self.pc_port))
                data.settimeout(0.1)
            except Exception:
                disc.close()
                data.close()
                raise

            self.disc = disc
            self.data = data
            self.running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

    def stop(self):
        with self._state_lock:
            if not self.running and self.thread is None:
                return
            self.running = False
            t = self.thread
            self.thread = None

        if t is not None and t is not threading.current_thread():
            t.join(1.0)

        for s in (self.disc, self.data):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
        self.disc = None
        self.data = None

    # ---- 后台循环 ----
    def _loop(self):
        while self.running:
            now = time.perf_counter()
            self._discover()
            self._hello(now)
            self._cal(now)
            try:
                pkt, _src = self.data.recvfrom(MAX_PKT)
            except OSError:
                continue
            self._packet(pkt, time.perf_counter())

    def _discover(self):
        while True:
            try:
                pkt, src = self.disc.recvfrom(256)
            except OSError:
                break
            if not pkt.startswith(b"HERE"):
                continue
            ip = src[0]
            status = pkt[5:].decode("ascii", "replace")
            with self.cond:
                changed = (ip != self.board_ip or status != self.board_status)
                self.board_ip = ip
                self.board_status = status
            if changed:
                print("[pc] board %s (%s)" % (ip, status))

    def _hello(self, now):
        if not self.board_ip:
            return
        if now - self.last_frame > HELLO_AFTER and now - self.last_hello > HELLO_GAP:
            try:
                self.data.sendto(b"HELLO", (self.board_ip, self.board_port))
            except OSError:
                return
            self.last_hello = now

    def _cal(self, now):
        if not self.board_ip or self.last_frame <= 0 or self.offset is not None:
            return
        if self.cal_sent >= 10 or len(self.cal_samples) >= 3:
            return
        if now - self.last_cal < CAL_GAP:
            return
        t1 = time.perf_counter()
        try:
            self.data.sendto(
                b"CAL" + struct.pack("<d", t1),
                (self.board_ip, self.board_port),
            )
        except OSError:
            return
        self.cal_sent += 1
        self.last_cal = now

    def _packet(self, pkt, rx):
        if pkt[:4] == b"CALR" and len(pkt) >= 16:
            t1, = struct.unpack_from("<d", pkt, 4)
            tb, = struct.unpack_from("<I", pkt, 12)
            rtt = rx - t1
            if 0.0 <= rtt < 2.0:
                cand = (t1 + rx) / 2.0 - tb / 1000.0
                with self.cond:
                    self.cal_samples.append((rtt, cand))
                    if len(self.cal_samples) >= 3 and self.offset is None:
                        self.cal_samples.sort()
                        self.offset = self.cal_samples[0][1]
                        print("[cal] offset=%.3fs min_rtt=%.1fms"
                              % (self.offset, self.cal_samples[0][0] * 1000.0))
            return

        if pkt[:2] != MAGIC or len(pkt) < HDR_LEN:
            return

        _magic, ver, cam_id, fid, idx, total, ts = struct.unpack_from(
            HDR_FMT, pkt, 0
        )
        if ver != VER or cam_id not in (0, 1, 2):
            return
        if cam_id == 0:
            cam_id = 1          # 兼容旧单摄板端（flags=0）

        ch = self.channels[cam_id]
        reboot = False
        with self.cond:
            if ch.last_ts is not None and ts < ch.last_ts - 5000:
                reboot = True
            ch.last_ts = ts

        if reboot:
            ch.reasm.clear()
            with self.cond:
                self.offset = None
                self.cal_samples = []
                self.cal_sent = 0
            print("[pc] channel %d timestamp reset, recalibrating" % cam_id)

        res = ch.reasm.add(fid, idx, total, ts, pkt[HDR_LEN:])
        if res is None:
            return

        jpg, ts = res
        try:
            frame = cv2.imdecode(
                np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR
            )
        except Exception:
            return
        if frame is None:
            return

        with self.cond:
            ch.seq += 1
            ch.frame = frame
            ch.jpeg = jpg
            ch.fid = fid
            ch.ts = ts
            ch.times.append(rx)
            while ch.times and rx - ch.times[0] > 2.0:
                ch.times.popleft()
            if len(ch.times) >= 2:
                span = ch.times[-1] - ch.times[0]
                ch.fps = (len(ch.times) - 1) / span if span > 0 else 0.0
            ch.latency_ms = (
                None if self.offset is None
                else (rx - (ts / 1000.0 + self.offset)) * 1000.0
            )
            self.last_frame = rx
            self.cond.notify_all()


# --------------------------------------------------------------------------
# 对外视图
# --------------------------------------------------------------------------
class CameraStream:
    """单个通道的视图；可调用，返回最新一帧 BGR ndarray。"""

    def __init__(self, transport, channel):
        self._t = transport
        self.channel = channel
        self._closed = False
        self._last_seq = (
            transport.channels[channel].seq if transport.running else 0
        )
        self._last_jpeg = None
        self._meta = None

    @property
    def closed(self):
        return self._closed

    @property
    def last_meta(self):
        with self._t.cond:
            return dict(self._meta) if self._meta is not None else None

    @property
    def last_jpeg(self):
        return self._last_jpeg

    @property
    def board_status(self):
        """板端最近一次 HERE 的状态，例如 '480P c1=fail c2=ok'。"""
        with self._t.cond:
            return self._t.board_status

    @property
    def board_ip(self):
        """已发现或手动指定的板子 IP。"""
        with self._t.cond:
            return self._t.board_ip

    def _deliver(self, ch):
        self._last_seq = ch.seq
        self._last_jpeg = ch.jpeg
        self._meta = {
            "channel": self.channel,
            "frame_id": ch.fid,
            "ts_ms": ch.ts,
            "latency_ms": ch.latency_ms,
            "fps": ch.fps,
        }
        return ch.frame

    def __call__(self, timeout=None):
        if self._closed:
            raise RuntimeError("stream closed")
        if timeout is None:
            timeout = FIRST_FRAME_TIMEOUT if self._meta is None else FRAME_TIMEOUT

        deadline = time.perf_counter() + timeout
        with self._t.cond:
            while True:
                if self._closed:
                    raise RuntimeError("stream closed")
                ch = self._t.channels[self.channel]
                if ch.seq > self._last_seq and ch.frame is not None:
                    return self._deliver(ch)
                remain = deadline - time.perf_counter()
                if remain <= 0:
                    if self._t.board_ip is None:
                        raise TimeoutError("未发现板子（收不到 HERE 广播）")
                    raise TimeoutError(
                        "通道 %d 在 %.1fs 内没有新帧" % (self.channel, timeout)
                    )
                self._t.cond.wait(remain)

    def try_get(self):
        if self._closed:
            raise RuntimeError("stream closed")
        with self._t.cond:
            ch = self._t.channels[self.channel]
            if ch.seq > self._last_seq and ch.frame is not None:
                return self._deliver(ch)
            return None

    def close(self):
        with self._t.cond:
            if self._closed:
                return
            self._closed = True
            self._t.cond.notify_all()
        self._t.release()


_transport = None
_streams = {}
_factory_lock = threading.Lock()


def receiver(channel=1, **transport_kwargs):
    """
    取得某个通道的接收视图（按 channel 缓存）。

    transport_kwargs 只在第一次调用、创建共享 transport 时生效：
        disc_port=5001, pc_port=5002, board_port=5000,
        board_ip=None  # 指定后跳过广播发现，直接向该 IP 发 HELLO
    """
    if channel not in CHANNELS:
        raise ValueError("channel 必须是 1 或 2")
    if cv2 is None or np is None:
        raise RuntimeError("缺少依赖：pip install opencv-python numpy")

    global _transport
    with _factory_lock:
        view = _streams.get(channel)
        if view is not None and not view.closed:
            return view

        if _transport is None:
            _transport = _Transport(**transport_kwargs)
            atexit.register(_shutdown)
        if not _transport.running:
            _transport.start()

        view = CameraStream(_transport, channel)
        _streams[channel] = view
        _transport.retain()
        return view


# 兼容你最初写的形态：
#   from pc_receiver import pc_receiver
#   img = pc_receiver()
pc_receiver = receiver


def _shutdown():
    global _transport
    tr = _transport
    _transport = None
    for view in _streams.values():
        view._closed = True
    _streams.clear()
    if tr is not None:
        tr.stop()


# --------------------------------------------------------------------------
# CLI 演示
# --------------------------------------------------------------------------
def on_frame(frame):
    """CLI 的算法钩子；API 本身不调用它。"""
    return None


class RelayState:
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0

    def publish(self, jpeg):
        if jpeg is None:
            return
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def wait_next(self, last_seq, timeout=1.0):
        with self.cond:
            if self.seq == last_seq:
                self.cond.wait(timeout)
            if self.seq == last_seq or self.jpeg is None:
                return None, last_seq
            return self.jpeg, self.seq


class RelayHandler(BaseHTTPRequestHandler):
    state = None

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            body = (
                b"<html><head><title>K230</title></head>"
                b"<body style='margin:0;background:#111'>"
                b"<img src='/stream' style='width:100%;height:auto'>"
                b"</body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/snapshot.jpg":
            jpeg, _ = self.state.wait_next(-1, timeout=0.1)
            if jpeg is None:
                self.send_error(503, "no frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return

        if self.path == "/stream":
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            try:
                while True:
                    jpeg, last = self.state.wait_next(last)
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                    )
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_error(404)


def start_relay(port, state):
    RelayHandler.state = state
    srv = ThreadingHTTPServer(("0.0.0.0", port), RelayHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def save_frame(frame, save_dir):
    name = datetime.now().strftime("frame_%Y%m%d_%H%M%S_%f")[:-3] + ".jpg"
    path = os.path.join(save_dir, name)
    cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print("[save]", path)


def parse_args():
    p = argparse.ArgumentParser(description="K230 dual-camera receiver")
    p.add_argument("--channel", choices=("auto", "1", "2"), default="auto",
                   help="auto=自动选有画面的那一路（默认），或强制 1/2")
    p.add_argument("--pc-port", type=int, default=5002)
    p.add_argument("--board-port", type=int, default=5000)
    p.add_argument("--disc-port", type=int, default=5001)
    p.add_argument("--board-ip", default=None,
                   help="跳过广播发现，直接连这个板子 IP（例如 192.168.209.193）")
    p.add_argument("--relay", action="store_true")
    p.add_argument("--relay-port", type=int, default=8080)
    p.add_argument("--no-gui", action="store_true",
                   help="不开 OpenCV 窗口（Ubuntu Server/无 X 环境）")
    p.add_argument("--save-dir", default="captures")
    return p.parse_args()


def main():
    args = parse_args()
    if cv2 is None or np is None:
        print("缺少依赖：pip install opencv-python numpy", file=sys.stderr)
        return 2

    os.makedirs(args.save_dir, exist_ok=True)

    relay_state = RelayState()
    if args.relay:
        try:
            start_relay(args.relay_port, relay_state)
            print("[relay] http://localhost:%d/stream" % args.relay_port)
        except OSError as e:
            print("[relay] 启动失败（端口被占用？）:", e)

    channels = (1, 2) if args.channel == "auto" else (int(args.channel),)
    streams = {}
    try:
        for ch in channels:
            streams[ch] = receiver(
                ch,
                disc_port=args.disc_port,
                pc_port=args.pc_port,
                board_port=args.board_port,
                board_ip=args.board_ip,
            )
    except OSError as e:
        print("[pc] 绑定端口失败:", e, file=sys.stderr)
        return 1

    mode_txt = "auto" if args.channel == "auto" else args.channel
    print("[pc] channel=%s, disc=%d video=%d (q 退出，s 存帧)"
          % (mode_txt, args.disc_port, args.pc_port))

    current = None
    active = None
    active_ch = None
    shown_ch = None
    last_stat = 0.0
    last_wait = time.perf_counter()

    try:
        while True:
            img = None
            for ch in channels:
                f = streams[ch].try_get()
                if f is not None:
                    img = f
                    active = streams[ch]
                    active_ch = ch
                    break

            if img is None:
                now = time.perf_counter()
                if now - last_wait > 5.0:
                    status = streams[channels[0]].board_status
                    ip = streams[channels[0]].board_ip
                    if status is None and ip is None:
                        msg = ("未发现板子：确认 PC 和板子在同一热点、热点没开客户端隔离、"
                               "Windows 防火墙放行 UDP 5001/5002")
                    elif status is None:
                        msg = ("已指定板子 %s，但还没收到帧；"
                               "检查 Windows 防火墙是否放行 UDP 5002" % ip)
                    else:
                        msg = ("板子已连接（%s），通道 %s 暂无新帧；"
                               "检查防火墙 UDP 5002、板端 CAM_ENABLED/排线"
                               % (status, "/".join(str(c) for c in channels)))
                        if args.channel == "auto":
                            msg += "；两路都无画面时先看板端日志"
                        else:
                            ch = int(args.channel)
                            if ("c%d=off" % ch) in status:
                                msg += "；该路在板端被 CAM_ENABLED 关闭了"
                            else:
                                other = 2 if ch == 1 else 1
                                if ("c%d=ok" % other) in status:
                                    msg += "；换 --channel %d 或 --channel auto" % other
                    print("[pc]", msg)
                    last_wait = now
                if current is None or args.no_gui:
                    time.sleep(0.01)
                elif cv2.waitKey(10) & 0xFF == ord("q"):
                    break
                continue

            if active_ch != shown_ch:
                print("[pc] display channel %d" % active_ch)
                shown_ch = active_ch

            shown = on_frame(img)
            if shown is None:
                shown = img
            current = shown

            relay_state.publish(active.last_jpeg)
            if not args.no_gui:
                cv2.imshow("K230 ch%d" % active_ch, shown)

            meta = active.last_meta or {}
            now = time.perf_counter()
            if now - last_stat >= 1.0:
                lat = meta.get("latency_ms")
                lat_txt = "--" if lat is None else "%.0fms" % lat
                print("[stat] ch=%d fps=%.1f latency=%s fid=%s"
                      % (active_ch, meta.get("fps", 0.0), lat_txt,
                         meta.get("frame_id")))
                last_stat = now

            if not args.no_gui:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s") and current is not None:
                    save_frame(current, args.save_dir)

    except KeyboardInterrupt:
        pass
    finally:
        for s in streams.values():
            s.close()
        if not args.no_gui:
            cv2.destroyAllWindows()
        print("[pc] bye")

    return 0


if __name__ == "__main__":
    sys.exit(main())
