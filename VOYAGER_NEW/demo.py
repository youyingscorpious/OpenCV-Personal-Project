# tune.py
import sys, time
sys.path.insert(0, '/home/scurry/Desktop/VOYAGER_WS/VISION/VISION_WS_doc/src')
import cv2
import pc_receiver
import color_core
import stop_sign_core as ssc

cd = color_core.ColorDetector()
sd = ssc.StopDetector()
view = pc_receiver.receiver(2)           # 板端 ch2

while True:
    frame = view(timeout=2.0)
    if frame is None:
        print('没收到帧')
        continue
    cres, cdbg = cd.process(frame, True)
    sres, sdbg = sd.process(frame, True)
    print('color=%d(%s) conf=%d%% | stop=%s conf=%d%% | 候选=%d'
          % (cres['color'], cres['label'], cres['confidence'],
             sres['stop'], sres['confidence'], sdbg['candidate_count']))
    for r in sdbg['rejected'][:4]:
        print('    拒绝 %-9s rect=%s' % (r['reason'], r['rect']))

    color_core.drawResult(frame, cres, cdbg, True)
    ssc.drawResult(frame, sres, sdbg, True)
    cv2.imshow('tune', frame)
    k = cv2.waitKey(1) & 0xFF
    if k in (27, ord('q')):
        break
    if k == ord('s'):
        cv2.imwrite('frame_%d.jpg' % int(time.time()), frame)
        print('已存图')

view.close()
cv2.destroyAllWindows()