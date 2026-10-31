# -*- coding: utf-8 -*-
"""闸门离线自检：不需要 ROS / 硬件 / rclpy，用虚拟时钟验证 6 个场景。

用法::

    python3 VOYAGER_NEW/voyager_gate/tools/gate_selftest.py

通过标准：``ALL SCENARIOS PASSED``（当前 13 项断言）。

场景
----
1. 正常路径：行进中被拒 → 停稳确认 → 三帧同色锁定
2. 颜色抖动 1→2→1→2 不锁定
3. 停稳确认超时 + proceed → 强制开窗，不卡死
4. 停稳确认超时 + fault → 进入 FAULT
5. 闸门关闭：不再拒绝行进中的颜色，投票仍生效
6. 闸门关闭 + 投票 1 帧 = 完全退回改造前行为
"""

import os
import sys

# 允许直接运行：把包根目录（含内层 voyager_gate/ 的那一层）加入 sys.path。
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from voyager_gate.junction_gate import (        # noqa: E402
    CMD_STATUS,
    STATE_IDLE,
    JunctionGateMixin,
)


class _FakeParam(object):
    def __init__(self, value):
        self.value = value


class _FakeLogger(object):
    def info(self, msg):
        print('   [info] %s' % msg)

    def warning(self, msg):
        print('   [warn] %s' % msg)

    def error(self, msg):
        print('   [err ] %s' % msg)


class _FakePublisher(object):
    def __init__(self, sink):
        self._sink = sink

    def publish(self, msg):
        self._sink.append(bool(msg.data))


class _SelfTestNode(JunctionGateMixin):
    """只实现闸门所需的最小宿主接口（不 import rclpy）。"""

    def __init__(self, overrides=None):
        self._params = {}
        self._logger = _FakeLogger()
        self.phases = []
        self.events = []
        self.requests = []
        self._now = 0.0
        self.selected_route = None
        self.state = 'LINE_FOLLOW'
        self.motor_status = None
        self.stopped_speed_threshold = 10
        self.status_is_measured = True
        self.turn_on_color_without_junction = False
        self.junction_color_wait = 8.0
        self._junction_enter_ts = None
        self._maneuver_done_once = False
        self._turn_phase = 'idle'
        self._route_done = False

        self._gate_declare()
        for key, value in (overrides or {}).items():
            self._params[key] = value
        self._gate_read()
        self._gate_init()

    # ---- 宿主接口 ----
    def declare_parameter(self, name, value):
        self._params.setdefault(name, value)
        return _FakeParam(self._params[name])

    def get_parameter(self, name):
        return _FakeParam(self._params[name])

    def create_publisher(self, *args, **kwargs):
        return _FakePublisher(self.requests)

    def get_logger(self):
        return self._logger

    def _publish_comp(self, phase, detail):
        self.phases.append(phase)
        self.events.append((self._now, phase))
        print('   <%5.2f> %-24s %s' % (self._now, phase, detail))

    def _fault(self, reason):
        self.state = 'FAULT'
        self.phases.append('FAULT')
        print('   <FAULT> %s' % reason)


# ---------------------------------------------------------------------------
# 场景驱动
# ---------------------------------------------------------------------------


def _run_scenario(name, steps, overrides=None):
    print('\n=== %s ===' % name)
    node = _SelfTestNode(overrides)
    for when, action in steps:
        node._now = when
        action(node, when)
    return node


def _step(func, *args):
    """把 (node, t, *args) 形式的函数适配成 (node, t) 回调。"""
    return lambda node, t: func(node, t, *args)


def _feed_status(node, t, state, left, right):
    """注入一帧 0x10 实测状态，并驱动一次闸门 tick。"""
    node._gate_note_motor_frame(CMD_STATUS, bytes(5), now=t)
    node.motor_status = (left, right, state)
    node._gate_tick(t)


def _feed_color(node, t, value):
    """注入一帧色卡；被闸门拒绝时不进入投票。"""
    if node._gate_allows_color(value, t) and node._gate_vote(value, t):
        node.selected_route = value


def _enter_junction(node, t):
    node.state = 'WAIT_ROUTE_MANEUVER'
    node._junction_enter_ts = t
    node._gate_on_junction_enter(t)


def _event_time(node, phase):
    for when, name in node.events:
        if name == phase:
            return when
    return None


def _selftest():
    results = []

    def check(label, condition):
        results.append((label, bool(condition)))
        print('   %s %s' % ('PASS' if condition else 'FAIL', label))

    # ---------- 场景 1：正常路径 ----------
    node = _run_scenario('场景 1：行进中被拒 → 停稳确认 → 三帧锁定', [
        (0.05, _step(_feed_color, 3)),                      # 行进中，必须被拒
        (0.10, _enter_junction),                            # 路口事件
        (0.15, _step(_feed_status, 1, 130, 130)),           # RUNNING，仍在滑行
        (0.30, _step(_feed_status, STATE_IDLE, 0, 0)),
        (0.40, _step(_feed_status, STATE_IDLE, 0, 0)),
        (0.50, _step(_feed_status, STATE_IDLE, 0, 0)),
        (0.60, _step(_feed_status, STATE_IDLE, 0, 0)),      # 第 3 帧 + 0.30 s 驻留 → 确认
        (0.70, _step(_feed_color, 1)),
        (0.80, _step(_feed_color, 1)),
        (0.95, _step(_feed_color, 1)),                      # 第 3 帧 → 锁定
    ])
    check('行进中的颜色被拒（I2）', 'COLOR_REJECTED' in node.phases)
    check('停稳证据在 0.60 s 才成立',
          abs((_event_time(node, 'JUNCTION_STOP_CONFIRMED') or -1.0) - 0.60) < 1e-9)
    check('识别窗口已打开', 'COLOR_WINDOW_OPEN' in node.phases)
    check('三帧同色后锁定为红', node.selected_route == 1)
    check('color_request 末态为 True', node.requests and node.requests[-1] is True)

    # ---------- 场景 2：颜色抖动不锁定 ----------
    node2 = _run_scenario('场景 2：颜色抖动 1→2→1→2 不锁定', [
        (0.00, _enter_junction),
        (0.10, _step(_feed_status, STATE_IDLE, 0, 0)),
        (0.20, _step(_feed_status, STATE_IDLE, 0, 0)),
        (0.30, _step(_feed_status, STATE_IDLE, 0, 0)),
        (0.40, _step(_feed_status, STATE_IDLE, 0, 0)),
        (0.50, _step(_feed_color, 1)),
        (0.60, _step(_feed_color, 2)),
        (0.70, _step(_feed_color, 1)),
        (0.80, _step(_feed_color, 2)),
    ])
    check('抖动不锁定（I3）', node2.selected_route is None)
    check('抖动期间窗口保持打开', 'JUNCTION_STOP_CONFIRMED' in node2.phases)

    # ---------- 场景 3 / 4：停稳确认超时 ----------
    def timeout_tick(node, t):
        node._gate_tick(2.1)                                # 上界 2.0 s 已被越过

    node3 = _run_scenario('场景 3：停稳超时 + proceed（默认）', [
        (0.00, _enter_junction),
        (0.10, timeout_tick),
    ])
    check('超时后强制开窗而非卡死（I4）', 'JUNCTION_STOP_CONFIRMED' in node3.phases)
    check('proceed 不改变导航状态', node3.state == 'WAIT_ROUTE_MANEUVER')

    node4 = _run_scenario('场景 4：停稳超时 + fault', [
        (0.00, _enter_junction),
        (0.10, timeout_tick),
    ], overrides={'junction_stop_confirm_timeout_action': 'fault'})
    check('fault 模式进入 FAULT', node4.state == 'FAULT')

    # ---------- 场景 5：闸门关闭，投票仍生效 ----------
    node5 = _run_scenario('场景 5：闸门关闭，行进中直接投票', [
        (0.00, _step(_feed_color, 3)),
        (0.10, _step(_feed_color, 3)),
        (0.20, _step(_feed_color, 3)),
    ], overrides={'junction_gate_enabled': False})
    check('闸门关闭后不再拒绝行进中的颜色', 'COLOR_REJECTED' not in node5.phases)
    check('投票仍生效（3 帧锁定）', node5.selected_route == 3)

    # ---------- 场景 6：闸门关闭 + 投票 1 帧 = 完全原行为 ----------
    node6 = _run_scenario('场景 6：gate=false + vote_frames=1 = 改造前行为', [
        (0.00, _step(_feed_color, 2)),
    ], overrides={'junction_gate_enabled': False, 'route_color_vote_frames': 1})
    check('单帧即锁定（等同改造前）', node6.selected_route == 2)

    failed = [label for label, good in results if not good]
    print('\n%d/%d checks passed' % (len(results) - len(failed), len(results)))
    for label in failed:
        print('  FAILED: %s' % label)
    if failed:
        return False
    print('ALL SCENARIOS PASSED')
    return True


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)
