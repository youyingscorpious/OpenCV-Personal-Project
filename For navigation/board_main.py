# -*- coding: utf-8 -*-
"""
board_main.py — 庐山派 Lite-K230D → PC UDP 图像流（CanMV v1.8）

通道 1：CSI1，PnP 用（可选，默认关闭）
通道 2：CSI2，color + STOP 共用，JPEG q75

流程：
  1. 连 2.4G 热点
  2. 按 CAM_ENABLED 初始化 CSI1/CSI2（构造 Sensor，只 run 一次）
  3. 周期广播 HERE，等 PC 发来 HELLO
  4. 已启用的各路交替 snapshot -> JPEG -> 1400B 分包 -> 同一 socket 发往 PC
  5. 响应 CAL（一次性时钟校准）

包头 16 字节，flags 字节承载 cam_id：
  magic(2) ver(1) cam_id(1) frame_id(4) chunk_idx(2) chunk_total(2) ts_ms(4)
"""

import gc
import os
import struct
import time
import socket

import network
from media.sensor import *
from media.media import *

# ======================== 配置 ========================
WIFI_SSID = "你的热点名称"
WIFI_PASSWORD = "你的热点密码"

CSI_PNP = 1        # 通道 1 接 CSI1
CSI_COLOR = 2      # 通道 2 接 CSI2
CSI_FOR_CH = {1: CSI_PNP, 2: CSI_COLOR}

# CSI1 开关：color 和 STOP 都走 CSI2 时设为 False，板子不初始化 CSI1，
# 采集/编码/发送时间全部让给 CSI2，ch2 有效帧率约翻倍。
# 需要 PnP 时改回 True。
ENABLE_CSI1 = False
ENABLE_CSI2 = True

# 每路开关：False 的路完全不初始化
CAM_ENABLED = {
    1: ENABLE_CSI1,   # CSI1 / PnP
    2: ENABLE_CSI2,   # CSI2 / color + STOP
}

WIDTH = 640
HEIGHT = 480
QUALITY = {1: 60, 2: 75}
TARGET_FPS = 15

BOARD_PORT = 5000
DISC_PORT = 5001
CHUNK_SIZE = 1400
MAX_CHUNKS = 900

WIFI_TIMEOUT_MS = 20000
ANNOUNCE_INTERVAL_MS = 2000
STAT_INTERVAL_MS = 5000

MAGIC = b"KV"
VER = 1
HDR_FMT = "<2sBBIHHI"
# ======================================================

pc_addr = None
last_announce = 0
bcast_addr = None


def log(*args):
    print("[board]", *args)


def wifi_connect():
    wlan = network.WLAN(0)
    try:
        if not wlan.active():
            wlan.active(True)
    except Exception:
        pass

    log("connecting to", WIFI_SSID)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    t0 = time.ticks_ms()
    while True:
        ip = wlan.ifconfig()[0]
        if ip and ip != "0.0.0.0":
            log("IP =", ip, "ifconfig =", wlan.ifconfig())
            return wlan
        if time.ticks_diff(time.ticks_ms(), t0) > WIFI_TIMEOUT_MS:
            raise RuntimeError(
                "WiFi 连接超时：检查 SSID/密码、热点是否为 2.4G、"
                "是否开了客户端隔离"
            )
        try:
            os.exitpoint()
        except Exception:
            pass
        time.sleep_ms(200)


def make_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(
            socket.SOL_SOCKET,
            getattr(socket, "SO_BROADCAST", 0x20),
            1,
        )
    except Exception:
        pass
    sock.bind(("0.0.0.0", BOARD_PORT))
    try:
        sock.settimeout(0)          # 非阻塞
    except Exception:
        sock.settimeout(0.001)
    return sock


def poll_ctrl(sock):
    try:
        data, addr = sock.recvfrom(64)
        return data, addr
    except OSError:
        return None, None


def handle_ctrl(sock, data, addr):
    global pc_addr

    if data[:5] == b"HELLO":
        if addr != pc_addr:
            log("PC registered:", addr)
        pc_addr = addr
        return

    if data[:3] == b"CAL" and len(data) >= 11:
        t1, = struct.unpack("<d", data[3:11])
        tb = time.ticks_ms() & 0xFFFFFFFF
        try:
            sock.sendto(b"CALR" + struct.pack("<dI", t1, tb), addr)
        except OSError:
            pass


def _ip_to_int(s):
    p = s.split(".")
    return (int(p[0]) << 24) | (int(p[1]) << 16) | (int(p[2]) << 8) | int(p[3])


def subnet_bcast(ip, mask):
    try:
        ipn = _ip_to_int(ip)
        mn = _ip_to_int(mask)
        b = (ipn & mn) | (~mn & 0xFFFFFFFF)
        return "%d.%d.%d.%d" % ((b >> 24) & 255, (b >> 16) & 255,
                                (b >> 8) & 255, b & 255)
    except Exception:
        return None


def _ch_state(ch, ready):
    if not CAM_ENABLED[ch]:
        return "off"
    return "ok" if ready[ch] else "fail"


def announce(sock, ready):
    global last_announce
    now = time.ticks_ms()
    if time.ticks_diff(now, last_announce) < ANNOUNCE_INTERVAL_MS:
        return
    last_announce = now

    msg = (b"HERE 480P c1=" + _ch_state(1, ready).encode()
           + b" c2=" + _ch_state(2, ready).encode())

    targets = ["255.255.255.255"]
    if bcast_addr and bcast_addr not in targets:
        targets.append(bcast_addr)
    for dst in targets:
        try:
            sock.sendto(msg, (dst, DISC_PORT))
        except OSError:
            pass


def compress_to_bytes(img, quality):
    """
    CanMV v1.8 的 img.compress() 返回 JPEG 格式的 Image 对象，
    不是 bytes；用 .bytearray() 取出 JPEG 字节再复制成 bytes。
    """
    out = img.compress(quality)
    try:
        return bytes(out.bytearray())
    except AttributeError:
        return bytes(out)


def build_sensors():
    """按 CAM_ENABLED 构造并配置各路；返回 {ch: sensor or None}。"""
    sensors = {}
    for ch in (1, 2):
        sid = CSI_FOR_CH[ch]
        if not CAM_ENABLED[ch]:
            log("c%d disabled (CSI%d)" % (ch, sid))
            sensors[ch] = None
            continue
        try:
            sensors[ch] = Sensor(id=sid, width=1920, height=1080, fps=30)
        except Exception as e:
            log("CSI%d construct failed: %r" % (sid, e))
            sensors[ch] = None

    for ch in (1, 2):
        s = sensors[ch]
        if s is None:
            continue
        try:
            s.reset()
            s.set_framesize(width=WIDTH, height=HEIGHT)
            s.set_pixformat(Sensor.RGB565)
            log("CSI%d configured (%dx%d)" % (CSI_FOR_CH[ch], WIDTH, HEIGHT))
        except Exception as e:
            log("CSI%d config failed: %r" % (CSI_FOR_CH[ch], e))
            sensors[ch] = None

    return sensors


def start_sensors(sensors):
    """只调一次 run()，它会启动所有已初始化的 Sensor。"""
    for ch in (1, 2):
        s = sensors[ch]
        if s is not None:
            try:
                s.run()
                log("sensors started via channel", ch)
                return True
            except Exception as e:
                log("run() on channel %d failed: %r" % (ch, e))
    return False


def self_test(sensors):
    """逐路抓一帧压缩，确认已启用的各路都能出图。"""
    ready = {1: False, 2: False}
    for ch in (1, 2):
        s = sensors[ch]
        if s is None:
            continue
        try:
            img = s.snapshot()
            jpg = compress_to_bytes(img, QUALITY[ch])
            if not jpg:
                raise RuntimeError("empty JPEG")
            log("c%d JPEG self-test ok: %d bytes" % (ch, len(jpg)))
            ready[ch] = True
            del img, jpg
        except Exception as e:
            log("c%d self-test failed: %r" % (ch, e))
    return ready


def send_frame(sock, cam_id, fid, ts, jpg):
    total = (len(jpg) + CHUNK_SIZE - 1) // CHUNK_SIZE
    if total == 0 or total > MAX_CHUNKS:
        return False
    off = 0
    for i in range(total):
        payload = jpg[off:off + CHUNK_SIZE]
        off += CHUNK_SIZE
        pkt = struct.pack(
            HDR_FMT, MAGIC, VER, cam_id, fid & 0xFFFFFFFF, i, total, ts
        ) + payload
        try:
            sock.sendto(pkt, pc_addr)
        except OSError:
            return False
    return True


def main():
    global pc_addr, bcast_addr

    log("cameras c1=%s c2=%s, %dx%d, q1=%d q2=%d" %
        ("on" if CAM_ENABLED[1] else "off",
         "on" if CAM_ENABLED[2] else "off",
         WIDTH, HEIGHT, QUALITY[1], QUALITY[2]))

    wlan = wifi_connect()
    try:
        ip, mask = wlan.ifconfig()[0], wlan.ifconfig()[1]
        bcast_addr = subnet_bcast(ip, mask)
        log("subnet broadcast =", bcast_addr)
    except Exception:
        bcast_addr = None

    sock = make_socket()

    sensors = build_sensors()

    try:
        MediaManager.init()      # v1.8 中已弃用/空实现，保留兼容
    except Exception:
        pass

    if not start_sensors(sensors):
        raise RuntimeError("没有可用摄像头：已启用的路都初始化失败")

    time.sleep_ms(200)
    ready = self_test(sensors)
    if not ready[1] and not ready[2]:
        raise RuntimeError("已启用的路自检都失败，检查排线/座子")

    fid = {1: 0, 2: 0}
    last_stat = time.ticks_ms()
    stat_frames = {1: 0, 2: 0}
    stat_bytes = {1: 0, 2: 0}
    interval_ms = 1000 // TARGET_FPS
    gc_tick = 0

    log("waiting for PC HELLO ...")

    try:
        while True:
            try:
                os.exitpoint()
            except Exception:
                pass

            announce(sock, ready)

            for _ in range(8):
                d, a = poll_ctrl(sock)
                if d is None:
                    break
                handle_ctrl(sock, d, a)

            if pc_addr is None:
                time.sleep_ms(10)
                continue

            for ch in (1, 2):
                if not ready[ch]:
                    continue
                s = sensors[ch]
                t_frame = time.ticks_ms()
                try:
                    img = s.snapshot()
                    ts = time.ticks_ms() & 0xFFFFFFFF
                    jpg = compress_to_bytes(img, QUALITY[ch])
                except Exception as e:
                    log("c%d capture/encode failed: %r" % (ch, e))
                    continue
                if not jpg:
                    continue

                ok = send_frame(sock, ch, fid[ch], ts, jpg)
                fid[ch] += 1
                if ok:
                    stat_frames[ch] += 1
                    stat_bytes[ch] += len(jpg)
                del img, jpg

                elapsed = time.ticks_diff(time.ticks_ms(), t_frame)
                if elapsed < interval_ms:
                    time.sleep_ms(interval_ms - elapsed)

            gc_tick += 1
            if gc_tick >= 200:
                gc.collect()
                gc_tick = 0

            now = time.ticks_ms()
            if time.ticks_diff(now, last_stat) >= STAT_INTERVAL_MS:
                dt = time.ticks_diff(now, last_stat) / 1000.0
                log("c1 %.1ffps %.1fKB/s (%s) | c2 %.1ffps %.1fKB/s (%s)"
                    % (stat_frames[1] / dt, stat_bytes[1] / 1024.0 / dt,
                       _ch_state(1, ready),
                       stat_frames[2] / dt, stat_bytes[2] / 1024.0 / dt,
                       _ch_state(2, ready)))
                last_stat = now
                stat_frames = {1: 0, 2: 0}
                stat_bytes = {1: 0, 2: 0}

    except KeyboardInterrupt:
        log("stopped by user")
    finally:
        for ch in (1, 2):
            s = sensors.get(ch)
            if s is not None:
                try:
                    s.stop()
                except Exception:
                    pass
        try:
            sock.close()
        except Exception:
            pass


main()
