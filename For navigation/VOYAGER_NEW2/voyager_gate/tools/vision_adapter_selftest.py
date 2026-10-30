# -*- coding: utf-8 -*-
"""vision_adapter 离线自检：不需要 ROS / rclpy / opencv / 硬件 / 相机。

用法::

    python3 VOYAGER_NEW2/voyager_gate/tools/vision_adapter_selftest.py

通过标准：``ALL ADAPTER CHECKS PASSED``。

它验证的是"胶水"而不是闸门（闸门看 ``tools/gate_selftest.py``）：

* B1 ``vision_roles`` 预置，``/navigation/start`` 的 role 门槛能被满足
* B2 ``vision_last_any`` / ``vision_last_seen`` 每 tick 被喂住
* P4 ``vision_roles`` 被 ``_reset_vision_session()`` 清空后自动补回
* P1 开赛前 ``"fuck"`` 不触发 FAULT（否则节点会被永久锁死）
* P2 开赛后 ``"fuck"`` 只 FAULT 一次，不会 33 Hz 刷 ESC
* F1 同一帧的 0.5 s 缓存只投一票，新帧才投下一票
* P5 ``nv.status()`` 异常时不投票（不退化成每 tick 一票）
* STOP 边沿触发一次，且 ``stop=="1"`` 时屏蔽当帧颜色

用一个假 ``nav_vision`` 模块替换真实模块，因此本文件可以放进 CI。
"""

import os
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)          # .../voyager_gate
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


class _FakeNV(object):
    """最小 nav_vision 替身：可控 color / stop / status。"""

    def __init__(self):
        self.color_val = '0'
        self.stop_val = '0'
        self.age = None
        self.status_raises = False
        self.start_raises = False
        self.start_calls = 0

    def start(self, color=True, stop=True):
        self.start_calls += 1
        if self.start_raises:
            raise RuntimeError('fake start failure')
        return True

    def color(self):
        return self.color_val

    def stop(self):
        return self.stop_val

    def status(self):
        if self.status_raises:
            raise RuntimeError('fake status failure')
        return {
            'fps': 25.0,
            'frames': 100,
            'last_frame_age_ms': 40,
            'last_color': self.color_val,
            'last_color_age_ms': self.age,
            'last_stop': self.stop_val,
            'last_error': None,
        }

    def close(self):
        pass


_NV = _FakeNV()
# 必须在 import adapter 之前注入：_vision_init 里是延迟 import。
sys.modules.setdefault('nav_vision', _NV)

from voyager_gate.vision_adapter import VisionAdapterMixin  # noqa: E402


class _Logger(object):
    def info(self, msg):
        pass

    def warning(self, msg):
        pass

    def error(self, msg):
        pass


class _FakeNode(VisionAdapterMixin):
    """只实现 adapter 需要的宿主接口。"""

    def __init__(self, overrides=None):
        self._params = {
            'vision_inline_enabled': True,
            'vision_module_path': '',
            'vision_poll_period_s': 0.03,
            'vision_stream_lost_fault': 5,
            'vision_suppress_color_on_stop': True,
            'vision_status_log_hz': 1.0,
        }
        self._params.update(overrides or {})
        # 父类 NavigationNode 中与视觉相关的字段
        self.vision_roles = set()
        self.vision_enabled = False
        self.vision_last_any = 0.0
        self.vision_last_seen = {}
        self.started = False
        self.state = 'IDLE'
        # 记录
        self.routes = []
        self.stop_requests = 0
        self.faults = []
        self.phases = []
        self._logger = _Logger()

    # ---- rclpy 替身 ----
    def declare_parameter(self, name, value):
        self._params.setdefault(name, value)
        return types.SimpleNamespace(value=self._params[name])

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def create_timer(self, period, callback):
        return None

    def get_logger(self):
        return self._logger

    # ---- 宿主钩子 ----
    def _select_route(self, color):
        self.routes.append(color)

    def _request_motor_stop(self):
        self.stop_requests += 1

    def _publish_comp(self, phase, detail):
        self.phases.append(phase)

    def _fault(self, reason):
        # 真实实现会把 state 切到 FAULT 并把 started 置 True
        self.faults.append(reason)
        self.state = 'FAULT'
        self.started = True

    # ---- 便捷 ----
    def boot(self):
        self._vision_declare()
        self._vision_read()
        self._vision_init()
        self._vision_nv = _NV
        return self


def _reset_nv():
    global _NV
    _NV.color_val = '0'
    _NV.stop_val = '0'
    _NV.age = None
    _NV.status_raises = False
    _NV.start_raises = False
    sys.modules['nav_vision'] = _NV


def _selftest():
    results = []

    def check(label, cond):
        results.append((label, bool(cond)))
        print('   %s %s' % ('PASS' if cond else 'FAIL', label))

    # ---------- 0. 默认关闭时不工作 ----------
    print('\n=== 0. vision_inline_enabled=false 时完全不工作 ===')
    _reset_nv()
    off = _FakeNode({'vision_inline_enabled': False}).boot()
    off._vision_tick()
    check('未开启时 vision_roles 保持空', off.vision_roles == set())
    check('未开启时 last_any 不被改写', off.vision_last_any == 0.0)

    # ---------- 1. B1 / B2 ----------
    print('\n=== 1. B1 role 预置 + B2 看门狗喂帧 ===')
    _reset_nv()
    n = _FakeNode().boot()
    check('B1 vision_roles == {1,2}', n.vision_roles == {1, 2})
    check('B1 vision_enabled == True', n.vision_enabled is True)
    n._vision_tick()
    check('B2 vision_last_any 已刷新', n.vision_last_any > 0.0)
    check('B2 last_seen[(1,TARGET_LIST)] 已写入',
          (1, 0x01) in n.vision_last_seen)

    # ---------- 2. P4 role 被清空后自动补回 ----------
    print('\n=== 2. P4 _reset_vision_session() 清空后自动补回 ===')
    n.vision_roles.clear()
    n.vision_last_seen.clear()
    n.vision_last_any = 0.0
    n._vision_tick()
    check('vision_roles 被补回 {1,2}', n.vision_roles == {1, 2})
    check('vision_last_seen 被补回', (1, 0x01) in n.vision_last_seen)

    # ---------- 3. F1 单帧去重 ----------
    print('\n=== 3. F1 同一帧缓存只投一票 ===')
    n.routes.clear()
    _NV.color_val = '1'
    for age in (5, 35, 65, 95, 125, 155, 185, 215, 245, 275):
        _NV.age = age
        n._vision_tick()
    check('同一帧轮询 10 次 -> 1 票', len(n.routes) == 1)
    for age in (5, 35, 65):
        _NV.age = age
        n._vision_tick()
    check('新帧到来 -> 再投 1 票', len(n.routes) == 2)

    # ---------- 4. P5 status 异常不投票 ----------
    print('\n=== 4. P5 nv.status() 异常时不投票 ===')
    m = _FakeNode().boot()
    m.routes.clear()
    _NV.color_val = '2'
    _NV.status_raises = True
    for _ in range(5):
        m._vision_tick()
    check('status 异常 -> 0 票（不退化成每 tick 一票）', m.routes == [])
    _NV.status_raises = False

    # ---------- 5. STOP 边沿 + 颜色屏蔽 ----------
    print('\n=== 5. STOP 边沿触发 + 屏蔽当帧颜色 ===')
    s = _FakeNode().boot()
    _reset_nv()
    s._vision_nv = _NV
    _NV.color_val = '1'
    _NV.age = 5
    _NV.stop_val = '1'
    for _ in range(5):
        s._vision_tick()
    check('stop 持续为 1 -> 只请求 1 次停车', s.stop_requests == 1)
    check('stop=="1" 时颜色被屏蔽', s.routes == [])
    _NV.stop_val = '0'
    s._vision_tick()
    _NV.stop_val = '1'
    s._vision_tick()
    check('0->1 再来一次 -> 第 2 次停车请求', s.stop_requests == 2)

    # ---------- 6. P1 开赛前不 fault ----------
    print('\n=== 6. P1 开赛前 "fuck" 不锁死节点 ===')
    _reset_nv()
    p = _FakeNode().boot()
    p.started = False
    _NV.color_val = 'fuck'
    for _ in range(50):
        p._vision_tick()
    check('开赛前 50 次 fuck -> 0 次 FAULT', p.faults == [])
    check('开赛前 started 仍为 False', p.started is False)
    check('开赛前仍发布 VISION_STATUS 诊断（排查链路要用）',
          'VISION_STATUS' in p.phases)

    # ---------- 7. P2 开赛后只 fault 一次 ----------
    print('\n=== 7. P2 开赛后 "fuck" 只 FAULT 一次 ===')
    p.started = True
    p.state = 'LINE_FOLLOW'
    for _ in range(30):
        p._vision_tick()
    check('30 次 fuck -> 恰好 1 次 FAULT', len(p.faults) == 1)
    check('FAULT 后 state 已切走', p.state == 'FAULT')

    # ---------- 8. lost_fault=0 不 fault ----------
    print('\n=== 8. vision_stream_lost_fault=0 只告警 ===')
    _reset_nv()
    q = _FakeNode({'vision_stream_lost_fault': 0}).boot()
    q.started = True
    q.state = 'LINE_FOLLOW'
    _NV.color_val = 'fuck'
    for _ in range(50):
        q._vision_tick()
    check('lost_fault=0 -> 0 次 FAULT', q.faults == [])
    check('仍然发布 COLOR_STREAM_LOST 告警',
          'COLOR_STREAM_LOST' in q.phases)

    # ---------- 9. 链路恢复 ----------
    print('\n=== 9. 链路恢复后计数归零 ===')
    _reset_nv()
    r = _FakeNode().boot()
    r.started = True
    r.state = 'LINE_FOLLOW'
    _NV.color_val = 'fuck'
    for _ in range(4):
        r._vision_tick()
    _NV.color_val = '3'
    _NV.age = 5
    r._vision_tick()
    check('恢复后计数归零', r._vision_lost_count == 0)
    check('恢复后发布 COLOR_STREAM_RECOVERED',
          'COLOR_STREAM_RECOVERED' in r.phases)

    failed = [label for label, ok in results if not ok]
    print('\n%d/%d checks passed' % (len(results) - len(failed), len(results)))
    for label in failed:
        print('  FAILED: %s' % label)
    if failed:
        return False
    print('ALL ADAPTER CHECKS PASSED')
    return True


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)
