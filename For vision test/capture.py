#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capture.py — 从庐山派拍照并保存

默认对准当前配置：通道 2（颜色摄像头），板子 192.168.209.193。
保存的是板端发来的 JPEG 原始字节（q75、640x480），不做二次压缩。

用法：
    python3 capture.py                         # 拍 1 张
    python3 capture.py -n 5                    # 连拍 5 张，间隔 1s
    python3 capture.py -n 10 -i 0.5            # 间隔 0.5s
    python3 capture.py -o photos               # 指定输出目录
    python3 capture.py --interactive           # 回车拍一张，q 退出
    python3 capture.py --png                   # 存 PNG（会重新编码）
    python3 capture.py --channel 1 --board-ip 192.168.209.193
"""

import argparse
import os
import sys
import time
from datetime import datetime

from pc_receiver import receiver


def make_path(out_dir, prefix, ext):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    return os.path.join(out_dir, "%s_%s%s" % (prefix, stamp, ext))


def shot(rx, args):
    frame = rx(timeout=args.timeout)          # 阻塞到下一帧
    meta = rx.last_meta or {}
    h, w = frame.shape[:2]

    ext = ".png" if args.png else ".jpg"
    path = make_path(args.out, args.prefix, ext)

    if args.png:
        import cv2
        cv2.imwrite(path, frame)
    else:
        jpg = rx.last_jpeg                     # 板端原始 JPEG bytes
        if jpg:
            with open(path, "wb") as f:
                f.write(jpg)
        else:
            import cv2
            cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

    lat = meta.get("latency_ms")
    lat_txt = "--" if lat is None else "%.0fms" % lat
    print("[capture] %s  %dx%d  fps=%.1f latency=%s"
          % (path, w, h, meta.get("fps", 0.0), lat_txt))
    return path


def main():
    ap = argparse.ArgumentParser(description="K230 photo capture")
    ap.add_argument("-n", "--count", type=int, default=1,
                    help="拍几张（默认 1）")
    ap.add_argument("-i", "--interval", type=float, default=1.0,
                    help="连拍间隔秒数（默认 1.0）")
    ap.add_argument("-o", "--out", default="photos",
                    help="输出目录（默认 photos）")
    ap.add_argument("--prefix", default="photo",
                    help="文件名前缀")
    ap.add_argument("--channel", type=int, choices=(1, 2), default=2,
                    help="通道（默认 2：颜色摄像头）")
    ap.add_argument("--board-ip", default="192.168.209.193",
                    help="板子 IP（默认 192.168.209.193）")
    ap.add_argument("--pc-port", type=int, default=5002)
    ap.add_argument("--board-port", type=int, default=5000)
    ap.add_argument("--disc-port", type=int, default=5001)
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="等一帧的超时秒数")
    ap.add_argument("--png", action="store_true",
                    help="保存为 PNG（重新编码，体积更大）")
    ap.add_argument("--interactive", action="store_true",
                    help="回车拍一张，q 退出")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    rx = receiver(
        args.channel,
        board_ip=args.board_ip,
        pc_port=args.pc_port,
        board_port=args.board_port,
        disc_port=args.disc_port,
    )

    print("[capture] channel=%d board=%s out=%s"
          % (args.channel, args.board_ip, args.out))

    try:
        if args.interactive:
            while True:
                try:
                    cmd = input("回车拍照，q 退出: ").strip().lower()
                except EOFError:
                    break
                if cmd == "q":
                    break
                try:
                    shot(rx, args)
                except TimeoutError as e:
                    print("[capture] 超时:", e)
        else:
            for i in range(max(1, args.count)):
                try:
                    shot(rx, args)
                except TimeoutError as e:
                    print("[capture] 超时:", e)
                    break
                if i + 1 < max(1, args.count):
                    time.sleep(max(0.0, args.interval))
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()
        print("[capture] done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
