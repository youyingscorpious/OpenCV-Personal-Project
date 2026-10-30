# -*- coding: utf-8 -*-
"""PC 调试入口：从板端流或本地视频读取，单窗口显示 color + STOP。

用法::

    python main.py                         # 读板端 ch2（默认）
    python main.py --board-ip 192.168.209.193
    python main.py --source 0              # 本地摄像头
    python main.py --source test.mp4       # 视频
    python main.py --no-gui                # 无窗口（Ubuntu Server）
    python main.py --debug                 # 画 ROI / 未确认候选

按 q 或 ESC 退出。
"""

import argparse
import time

import cv2

import color_core
import pc_receiver
import stop_sign_core


WINDOW_NAME = "nav vision (ch2)"
PROC_W = 640
PROC_H = 480


def parse_args():
    parser = argparse.ArgumentParser(description="color + STOP debug viewer")
    parser.add_argument(
        "--source",
        default="board",
        help="board（默认，读 pc_receiver）或摄像头编号/视频路径",
    )
    parser.add_argument("--channel", type=int, default=2)
    parser.add_argument("--board-ip", default=None)
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--no-loop", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--width", type=int, default=PROC_W)
    parser.add_argument("--height", type=int, default=PROC_H)
    return parser.parse_args()


def process_frame(frame, color_det, stop_det, debug):
    """缩放到处理分辨率，跑两个 detector，画到同一帧上。"""
    if frame.shape[1] != PROC_W or frame.shape[0] != PROC_H:
        frame = cv2.resize(
            frame, (PROC_W, PROC_H), interpolation=cv2.INTER_LINEAR
        )

    t0 = time.perf_counter()
    color_result, color_debug = color_det.process(frame, collectDebug=debug)
    stop_result, stop_debug = stop_det.process(frame, collectDebug=debug)
    proc_ms = (time.perf_counter() - t0) * 1000.0

    color_result["proc_ms"] = proc_ms
    stop_result["proc_ms"] = proc_ms

    color_core.drawResult(frame, color_result, color_debug, debug)
    stop_sign_core.drawResult(frame, stop_result, stop_debug, debug)
    return frame, color_result, stop_result, proc_ms


def main():
    args = parse_args()

    color_det = color_core.ColorDetector()
    stop_det = stop_sign_core.StopDetector()

    board = args.source.strip().lower() == "board"
    stream = None
    cap = None
    is_camera = False

    if board:
        stream = pc_receiver.receiver(args.channel, board_ip=args.board_ip)
        print("[main] board source, channel=%d" % args.channel)
    else:
        try:
            source = int(args.source)
            is_camera = True
        except ValueError:
            source = args.source

        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            print("无法打开视频源:", args.source)
            return 1
        if is_camera:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print("[main] source=%s" % args.source)

    if not args.no_gui:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    frames = 0
    fps = 0.0
    prev = time.perf_counter()
    last_print = 0.0
    last_state = None

    try:
        while True:
            if board:
                try:
                    frame = stream(timeout=1.0)
                except TimeoutError as exc:
                    print("[main] 等待画面:", exc)
                    continue
            else:
                ret, frame = cap.read()
                if not ret or frame is None:
                    if is_camera or args.no_loop:
                        break
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    color_det.reset()
                    stop_det.reset()
                    time.sleep(0.03)
                    continue

            frames += 1
            frame, color_result, stop_result, proc_ms = process_frame(
                frame, color_det, stop_det, args.debug
            )

            now = time.perf_counter()
            dt = now - prev
            prev = now
            if dt > 0:
                inst = 1.0 / dt
                fps = inst if fps == 0.0 else fps * 0.9 + inst * 0.1

            cv2.putText(
                frame,
                "%.1f FPS | %.1f ms" % (fps, proc_ms),
                (10, frame.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            state = (color_result["color"], stop_result["stop"])
            if state != last_state or (now - last_print) >= 0.5:
                last_state = state
                last_print = now
                print(
                    "[main] color=%d stop=%d trackers=%d cand=%d "
                    "proc=%.1fms fps=%.1f"
                    % (
                        color_result["color"],
                        1 if stop_result["stop"] else 0,
                        len(color_result["trackers"]),
                        stop_result["candidate_count"],
                        proc_ms,
                        fps,
                    )
                )

            if not args.no_gui:
                cv2.imshow(WINDOW_NAME, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
            else:
                time.sleep(0.001)

            if args.max_frames > 0 and frames >= args.max_frames:
                break

    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        if stream is not None:
            try:
                stream.close()
            except Exception:
                pass
        if not args.no_gui:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
