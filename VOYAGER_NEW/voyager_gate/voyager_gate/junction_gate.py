# -*- coding: utf-8 -*-
"""路口「停车 → 色卡识别」闸门（JunctionGateMixin）。

所属
----
``voyager_gate`` 功能包的生产实现，对应设计文档 ``VOYAGER_NEW/01_算法设计.md``
中的 I1–I5 不变量与 G0–G4 相位。

设计约束
--------
* 不修改 ``robopioneers_navigation`` 任何文件、不修改任何通信参数、不新增电控命令字。
* 闸门不新增 state，只在 ``WAIT_ROUTE_MANEUVER`` 内部维护子相位。
* 每个等待都有上界，上界到达后有确定动作（proceed / fault）。

调用点（全部在 ``voyager_node.py``，共 8 处）
--------------------------------------------
| 时机 | 调用 |
|---|---|
| ``__init__`` | ``_gate_declare`` / ``_gate_read`` / ``_gate_init`` |
| ``_handle_motor_frame`` 开头 | ``_gate_note_motor_frame(cmd, data)`` |
| 进入 ``WAIT_ROUTE_MANEUVER`` | ``_gate_on_junction_enter()`` |
| ``_junction_tick(now)`` 开头 | ``_gate_tick(now)`` |
| ``_select_route`` 锁色前 | ``_gate_allows_color`` / ``_gate_vote`` |
| ``_junction_tick`` 判 ``selected_route`` 前 | ``_gate_blocks_turn()`` |
| ``_finish_turn`` 结束 | ``_gate_on_turn_finish()`` |

离线自检
--------
::

    python3 VOYAGER_NEW/voyager_gate/tools/gate_selftest.py

不依赖 ROS / 硬件 / rclpy，用虚拟时钟跑 6 个场景。
"""

import time

# ---------------------------------------------------------------------------
# ROS / 原包依赖：为了支持离线自检，导入失败时退化为最小替身。
# 在真机上这两个 import 一定成功，替身分支不会生效。
# ---------------------------------------------------------------------------
try:                                                    # pragma: no cover
    from std_msgs.msg import Bool
except ImportError:                                     # 离线自检

    class Bool(object):
        """`std_msgs/msg/Bool` 的最小替身，只保留 data 字段。"""

        def __init__(self):
            self.data = False


try:                                                    # pragma: no cover
    from robopioneers_navigation.protocols import (
        CMD_STATUS,
        STATE_IDLE,
        STATE_JUNCTION,
    )
except ImportError:                                     # 离线自检
    # 取值与 robopioneers_navigation/protocols.py 完全一致
    CMD_STATUS = 0x10
    STATE_IDLE = 0
    STATE_JUNCTION = 6


__all__ = ['JunctionGateMixin']

# 停稳判据里 0x10 状态帧的新鲜度上限（秒）。
# 固件约 100 ms 一帧，0.25 s 允许丢 2 帧。
STATUS_FRESH_S = 0.25


class JunctionGateMixin(object):
    """给 CompetitionNode 补上「停稳确认 → 开识别窗口 → 投票锁定」三道闸门。

    依赖宿主类（CompetitionNode / NavigationNode）提供：
        state, motor_status, status_is_measured, stopped_speed_threshold,
        selected_route, _junction_enter_ts, _maneuver_done_once, _turn_phase,
        junction_color_wait, turn_on_color_without_junction,
        get_logger(), _publish_comp(phase, detail), _fault(reason), _hold_zero(now)
    """

    # ---- 类级默认值 ----
    # super().__init__() 期间若提前触发 _fault，这些属性必须已经存在。
    _motor_status_ts = 0.0
    _stop_confirm_samples = 0
    _stop_confirm_since = None
    _stop_confirmed = False
    _color_window_open = False
    _vote_color = 0
    _vote_count = 0
    _vote_ts = 0.0
    _reject_log_ts = 0.0
    _gate_warn_ts = 0.0

    # ==================== 初始化 ====================
    def _gate_declare(self):
        """① 声明闸门参数。全部带默认值，YAML 不写也能跑。"""
        d = self.declare_parameter
        # 总开关：False = 完全回到本方案落地前的行为（行进中锁存、到路口就转）。
        d('junction_gate_enabled', True)
        # 停稳确认：连续多少帧 0x10 同时满足「状态=停」与「左右轮速≈0」。
        d('junction_stop_confirm_samples', 3)
        # 停稳确认：从首帧满足起，至少还要持续多久。
        d('junction_stop_confirm_dwell_s', 0.25)
        # 停稳确认：多久还没确认就放弃等待。
        d('junction_stop_confirm_timeout_s', 2.0)
        # 放弃等待后的动作：proceed（告警后强制开窗）/ fault（进 FAULT）。
        d('junction_stop_confirm_timeout_action', 'proceed')
        # 只采信「停稳之后」的颜色帧；False = 退回行进中锁存。
        d('route_color_require_after_stop', True)
        # 锁定所需连续同色帧数；1 = 来一帧就锁（等同原行为）。
        d('route_color_vote_frames', 3)
        # 投票有效期：超过这么久没有同色帧就重新计数。
        d('route_color_vote_timeout_s', 0.6)
        # 「丢弃颜色帧」告警的限频，避免 10 Hz 帧流刷满日志。
        d('route_color_reject_log_hz', 2.0)

    def _gate_read(self):
        """② 读取参数到实例字段。"""
        p = self.get_parameter
        self.junction_gate_enabled = bool(p('junction_gate_enabled').value)
        self.stop_confirm_samples = max(1, int(p('junction_stop_confirm_samples').value))
        self.stop_confirm_dwell = max(0.0, float(p('junction_stop_confirm_dwell_s').value))
        self.stop_confirm_timeout = max(0.2, float(p('junction_stop_confirm_timeout_s').value))
        action = str(p('junction_stop_confirm_timeout_action').value).strip().lower()
        self.stop_confirm_timeout_action = action if action in ('proceed', 'fault') else 'proceed'
        self.color_require_after_stop = bool(p('route_color_require_after_stop').value)
        self.color_vote_frames = max(1, int(p('route_color_vote_frames').value))
        self.color_vote_timeout = max(0.05, float(p('route_color_vote_timeout_s').value))
        self.reject_log_period = 1.0 / max(0.1, float(p('route_color_reject_log_hz').value))

    def _gate_init(self):
        """③ 运行期字段与诊断话题。"""
        self._motor_status_ts = 0.0
        self._stop_confirm_samples = 0
        self._stop_confirm_since = None
        self._stop_confirmed = False
        self._color_window_open = False
        self._vote_color = 0
        self._vote_count = 0
        self._vote_ts = 0.0
        self._reject_log_ts = 0.0
        self._gate_warn_ts = 0.0
        self.color_request_pub = self.create_publisher(
            Bool, '/navigation/junction/color_request', 10
        )

    # ==================== 钩子 ====================
    def _gate_note_motor_frame(self, cmd, data, now=None):
        """⑤ 记录 0x10 实测轮速帧的时刻，停稳确认靠它判断新鲜度。"""
        if cmd == CMD_STATUS and len(data) == 5:
            self._motor_status_ts = time.monotonic() if now is None else now

    def _gate_on_junction_enter(self, now=None):
        """④ 进入 WAIT_ROUTE_MANEUVER：闸门复位到 G1 ARMING。

        注意：清空 `selected_route` 由调用方负责（只在首次进入时清，
        重复进入 / 转向后被打回路口时不能清，否则会触发二次转向）。
        """
        self._stop_confirm_samples = 0
        self._stop_confirm_since = None
        self._stop_confirmed = False
        self._color_window_open = False
        self._vote_color = 0
        self._vote_count = 0
        self._vote_ts = 0.0 if now is None else now
        self._gate_publish_request(False)

    def _gate_tick(self, now):
        """⑥ 闸门主循环（每 20 ms 一次）：G1 → G2。"""
        if not self.junction_gate_enabled or self._stop_confirmed:
            return
        if self.state != 'WAIT_ROUTE_MANEUVER' or self._maneuver_done_once:
            return

        if self._gate_is_stopped(now):
            if self._stop_confirm_since is None:
                self._stop_confirm_since = now
            self._stop_confirm_samples += 1
            if (self._stop_confirm_samples >= self.stop_confirm_samples
                    and (now - self._stop_confirm_since) >= self.stop_confirm_dwell):
                self._gate_confirm(now)
                return
        else:
            self._stop_confirm_samples = 0
            self._stop_confirm_since = None

        # I4：等待必须有上界，上界到达后动作确定。
        if (self._junction_enter_ts is not None
                and (now - self._junction_enter_ts) > self.stop_confirm_timeout):
            detail = 'junction stop not confirmed in %.1f s (%s)' % (
                self.stop_confirm_timeout, self._gate_status_text(now))
            if self.stop_confirm_timeout_action == 'fault':
                self._fault(detail)
            else:
                self.get_logger().warning(detail + '; opening color window anyway')
                self._gate_confirm(now, forced=True)

    def _gate_is_stopped(self, now):
        """单帧停稳判据：新鲜 + 状态 ∈ {IDLE, JUNCTION} + 左右轮速 ≤ 阈值。"""
        status = self.motor_status
        if status is None or (now - self._motor_status_ts) > STATUS_FRESH_S:
            return False
        if status[2] not in (STATE_IDLE, STATE_JUNCTION):
            return False
        if not self.status_is_measured:
            # 轮速字段不是编码器实测值：降级为 state-only 判据（D3），限频告警。
            self._gate_warn(
                'motor_status_is_measured=false; junction stop confirm degrades to state-only'
            )
            return True
        return (abs(status[0]) <= self.stopped_speed_threshold
                and abs(status[1]) <= self.stopped_speed_threshold)

    def _gate_confirm(self, now, forced=False):
        """G2：停稳证据达标，打开识别窗口。"""
        self._stop_confirmed = True
        self._color_window_open = True
        self._stop_confirm_samples = 0
        self._stop_confirm_since = None
        self._vote_color = 0
        self._vote_count = 0
        self._vote_ts = now
        self._publish_comp(
            'JUNCTION_STOP_CONFIRMED',
            'forced=%s %s' % (forced, self._gate_status_text(now)),
        )
        self._publish_comp(
            'COLOR_WINDOW_OPEN',
            'need %d same-color frames within %.1f s'
            % (self.color_vote_frames, getattr(self, 'junction_color_wait', 8.0)),
        )
        self._gate_publish_request(True)

    # ==================== 采信闸门 ====================
    def _gate_allows_color(self, color, now=None):
        """⑦ 这一帧颜色是否允许进入投票（I2）。"""
        if not self.junction_gate_enabled or not self.color_require_after_stop:
            return True
        now = time.monotonic() if now is None else now
        if self.state == 'WAIT_ROUTE_MANEUVER':
            if self._stop_confirmed:
                return True
            self._gate_reject('color=%d rejected: junction stop not confirmed yet' % color, now)
            return False
        if self.state == 'LINE_FOLLOW':
            # D7：显式开启 fallback 时放行（仍在停车前开窗，属应急路径）。
            if self.turn_on_color_without_junction:
                return True
            self._gate_reject('color=%d rejected: still line-following (I2)' % color, now)
            return False
        return True

    def _gate_vote(self, color, now):
        """⑦ 连续同色投票；返回 True 表示应锁定。"""
        if color != self._vote_color or (now - self._vote_ts) > self.color_vote_timeout:
            self._vote_color = color
            self._vote_count = 1
        else:
            self._vote_count += 1
        self._vote_ts = now
        return self._vote_count >= self.color_vote_frames

    def _gate_blocks_turn(self):
        """⑧ 未确认停稳时禁止转向（I1）。"""
        if not self.junction_gate_enabled:
            return False
        if self._maneuver_done_once:
            # 转向完成后被固件锁存的路口事件拉回来：允许直接恢复，不再等确认。
            return False
        return not self._stop_confirmed

    def _gate_on_turn_finish(self):
        """⑨ 转向完成，关闭识别窗口（I3 的收尾）。"""
        self._color_window_open = False
        self._gate_publish_request(False)

    # ==================== 工具 ====================
    def _gate_publish_request(self, on):
        msg = Bool()
        msg.data = bool(on)
        self.color_request_pub.publish(msg)

    def _gate_reject(self, text, now):
        """限频记录被丢弃的颜色帧，便于诊断却不刷屏。"""
        if self._reject_log_ts > 0.0 and (now - self._reject_log_ts) < self.reject_log_period:
            return
        self._reject_log_ts = now
        self._publish_comp('COLOR_REJECTED', text)

    def _gate_warn(self, text):
        """限频告警（1 Hz）。"""
        now = time.monotonic()
        if (now - self._gate_warn_ts) < 1.0:
            return
        self._gate_warn_ts = now
        self.get_logger().warning(text)

    def _gate_status_text(self, now):
        status = self.motor_status
        if status is None:
            return 'no 0x10 status'
        return 'state=%d v=%d/%d age=%.2fs' % (
            status[2], status[0], status[1], now - self._motor_status_ts
        )
