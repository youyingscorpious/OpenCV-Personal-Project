import time
import nav_vision as nv

nv.start()  # 可选：提前启动

try:
    while True:
        a = nv.color()
        b = nv.stop()
        st = nv.status()

        print(
            "color=%s stop=%s board=%s fps=%s last_frame=%sms err=%s"
            % (
                a,
                b,
                st["board_ip"],
                st["fps"],
                st["last_frame_age_ms"],
                st["last_error"],
            )
        )
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
finally:
    nv.close()
