# -*- coding: utf-8 -*-
"""【模板副本】赛程增强导航节点（路口定角转向 + STOP 定距停车）。

来源
----
本文件由 ``robopioneers_competition/robopioneers_competition/route_controller.py``
**原样复制**而来，只做两处改动：

1. 本段说明文字（标明这是模板副本）；
2. 删掉文件末尾的 ``main()`` 与 ``if __name__ == '__main__'`` 段
   （本包用自己的入口 ``voyager_node.py:main``）。

算法逻辑、参数默认值、方法名与原件逐行一致；原文件未被修改一个字节。

为什么复制而不是直接 import
--------------------------
``voyager_gate`` 只依赖**真正的原件** ``robopioneers_navigation``，不依赖
``robopioneers_competition``。这样两个增强包可以各自独立编译、互不干扰，
本包也能单独放进任何已装原导航包的工作空间。

继承链
------
::

    NavigationNode                (robopioneers_navigation，原件只读)
        └── CompetitionNode       (本文件，模板副本：定角转向 + STOP 定距)
                └── VoyagerNode   (voyager_node.py，新增路口「停车→识别」闸门)

本节点原本只做三件事（全部通过继承与方法重写完成）：


1. **路口定角转向**（方案 A，不动电机固件）
   进入 ``WAIT_ROUTE_MANEUVER`` 后，用电机板 0x11 遥测里的 ``heading_deg``
   做开环定角：左路 −X°、中路 0°、右路 +X°，转完再发 ``CMD_LINE`` 恢复循迹。
   转向期间以 20 Hz 持续下发 ``CMD_CHASSIS`` 差速帧，同时喂饱电机板 300 ms
   指令看门狗。

2. **STOP 定距停车**
   视觉报 STOP 时不立刻零速，而是按 0x11 的 ``distance_mm`` 再走固定里程
   再停车（``stop_delay_mm``，默认 0 = 完全保持原行为）。

3. **路口等待期间的零速保持**
   原节点在 ``WAIT_ROUTE_MANEUVER`` 只发一次零速，之后就不再刷帧，容易触发
   固件 300 ms 看门狗。本节点在等待期按 ``junction_hold_hz`` 保持零速。

设计约束
--------
* 不修改 ``robopioneers_navigation`` 的任何源文件。
* 不改动任何通信参数：端口 5000、波特率 115200、AA55+CRC16（电控）、
  AA55+SUM8（视觉）帧格式、串口设备名全部沿用原值。
* 唯一「运行期改动」是向原模块的 ``TRANSITIONS`` 字典补一条转移边
  ``(WAIT_ROUTE_MANEUVER, ROUTE_SELECTED) -> ROUTE_FOLLOW``，字典是模块级
  运行期数据，原文件保持只读。
"""

import json
import time

from std_msgs.msg import String, UInt8

# 导入原包：只读引用，不做任何源码修改。
from robopioneers_navigation import navigation_node as nav_mod
from robopioneers_navigation.navigation_node import NavigationNode

from robopioneers_navigation.protocols import (
    CMD_STATUS,
    EVENT_JUNCTION,
    STATE_JUNCTION,
    STATE_RUNNING,
)

# 运行期补一条状态转移边：路口转向完成后从 WAIT_ROUTE_MANEUVER 进入 ROUTE_FOLLOW。
# 原表里没有这条边（原实现根本不需要），这里补齐即可，原文件不动。
nav_mod.TRANSITIONS[('WAIT_ROUTE_MANEUVER', 'ROUTE_SELECTED')] = 'ROUTE_FOLLOW'

# 颜色到角度参数的映射（规则 3.2.4 / 4.2：红=左、黄=中、蓝=右）。
ROUTE_ANGLE_PARAM = {
    1: 'route_angle_left_deg',
    2: 'route_angle_mid_deg',
    3: 'route_angle_right_deg',
}
# 颜色到中文名，仅用于日志。
ROUTE_NAME = {
    1: 'RED/LEFT',
    2: 'YELLOW/MID',
    3: 'BLUE/RIGHT',
}


def _wrap180(deg):
    """把角度规整到 (-180, 180]，避免航向跨 0/360 时跳变。"""
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return deg


class CompetitionNode(NavigationNode):
    """在原导航节点之上补齐路口选路与 STOP 定距停车。"""

    # 类级默认值：保证 super().__init__() 内部若提前触发 _fault 时属性已存在。
    _enh_ready = False
    _turn_phase = 'idle'
    _junction_enter_ts = None
    _route_done = False
    _stop_delay_active = False
    _last_zero_ts = 0.0
    _route_pub_cache = None
    _junction_done_ts = 0.0
    _resume_retries = 0
    _maneuver_done_once = False

    def __init__(self):
        # 先完成原节点初始化：参数、串口、视觉链路、服务、20 ms 轮询全部就绪。
        super().__init__()

        # ==================== 本包新增参数 ====================
        # 总开关：False 时完全退回原节点行为（便于对照排查）。
        self.declare_parameter('route_turn_enabled', True)
        # 三条支路的转向角（度，相对路口停车瞬间的航向，正负由 heading 符号决定）。
        self.declare_parameter('route_angle_left_deg', -40.0)
        self.declare_parameter('route_angle_mid_deg', 0.0)
        self.declare_parameter('route_angle_right_deg', 40.0)
        # 航向符号：实测发现转向方向与目标相反时置 -1.0。
        self.declare_parameter('route_heading_sign', 1.0)
        # 原地转向时的单轮速度绝对值（mm/s），差速帧左右取反号。
        self.declare_parameter('route_turn_speed_mm_s', 120)
        # 到角判据（度），越小越准但越容易抖。
        self.declare_parameter('route_turn_tolerance_deg', 3.0)
        # 转向超时（秒），超时进入 FAULT，避免原地无限旋转。
        self.declare_parameter('route_turn_timeout_s', 4.0)
        # 到角后保持零速的稳定时间（秒）。
        self.declare_parameter('route_turn_settle_s', 0.25)
        # 路口等不到颜色时的兜底超时（秒），超时进入 FAULT。
        self.declare_parameter('route_junction_color_wait_s', 8.0)
        # 是否允许「没有路口事件、只收到颜色」时也停车转向（默认关闭，符合规则要求先停车）。
        self.declare_parameter('route_turn_on_color_without_junction', False)
        # 转向完成后的「路口事件屏蔽时间」（秒）：电机固件可能持续锁存路口事件，
        # 若不清除会把车反复拉回 WAIT_ROUTE_MANEUVER。
        self.declare_parameter('route_junction_ignore_s', 3.0)
        # 路口等待/转向期间保持零速刷帧的频率（Hz），用于喂 300 ms 看门狗。
        self.declare_parameter('route_junction_hold_hz', 20.0)
        # 路口等待时是否持续刷零速帧（关闭后完全等同原节点）。
        self.declare_parameter('route_junction_hold_enable', True)

        # STOP 定距停车：识别到 STOP 后按里程再走多少 mm 才真正零速。
        self.declare_parameter('stop_delay_mm', 0.0)
        # 里程变化不可用时的兜底超时（秒）。
        self.declare_parameter('stop_delay_timeout_s', 3.0)
        # WAIT_MOTOR_STOP 阶段是否持续刷零速（原节点只发一次）。
        self.declare_parameter('refresh_zero_in_wait_stop', False)

        # ==================== 读取参数 ====================
        self.route_turn_enabled = bool(self.get_parameter('route_turn_enabled').value)
        self.route_angles = {
            1: float(self.get_parameter(ROUTE_ANGLE_PARAM[1]).value),
            2: float(self.get_parameter(ROUTE_ANGLE_PARAM[2]).value),
            3: float(self.get_parameter(ROUTE_ANGLE_PARAM[3]).value),
        }
        self.route_heading_sign = float(self.get_parameter('route_heading_sign').value)
        self.route_turn_speed = abs(int(self.get_parameter('route_turn_speed_mm_s').value))
        self.route_turn_tolerance = abs(float(self.get_parameter('route_turn_tolerance_deg').value))
        self.route_turn_timeout = max(0.5, float(self.get_parameter('route_turn_timeout_s').value))
        self.route_turn_settle = max(0.0, float(self.get_parameter('route_turn_settle_s').value))
        self.junction_color_wait = max(0.5, float(self.get_parameter('route_junction_color_wait_s').value))
        self.turn_on_color_without_junction = bool(
            self.get_parameter('route_turn_on_color_without_junction').value
        )
        self.junction_ignore_s = max(0.0, float(self.get_parameter('route_junction_ignore_s').value))
        hold_hz = max(1.0, float(self.get_parameter('route_junction_hold_hz').value))
        self.junction_hold_period = 1.0 / hold_hz
        self.junction_hold_enable = bool(self.get_parameter('route_junction_hold_enable').value)
        self.stop_delay_mm = max(0.0, float(self.get_parameter('stop_delay_mm').value))
        self.stop_delay_timeout = max(0.5, float(self.get_parameter('stop_delay_timeout_s').value))
        self.refresh_zero_in_wait_stop = bool(
            self.get_parameter('refresh_zero_in_wait_stop').value
        )

        # ==================== 运行期字段 ====================
        self._turn_phase = 'idle'          # idle / turning / settling
        self._turn_start_heading = 0.0
        self._turn_target = 0.0
        self._turn_started = 0.0
        self._settle_ts = 0.0
        self._junction_enter_ts = None
        self._route_done = False
        self._last_zero_ts = 0.0
        self._stop_delay_active = False
        self._stop_delay_base_mm = 0
        self._stop_delay_started = 0.0
        self._route_pub_cache = None
        self._junction_done_ts = 0.0
        self._resume_retries = 0
        self._maneuver_done_once = False

        # 赛程诊断话题（只增不改，不影响原话题）。
        self.competition_pub = self.create_publisher(String, '/navigation/competition/status', 10)

        # 20 ms 增强定时器，与原节点轮询同周期。
        self.enh_timer = self.create_timer(0.02, self._competition_tick)

        self._enh_ready = True
        self.get_logger().info(
            'competition enhancements ready: turn=%s angles L/M/R=%.1f/%.1f/%.1f deg, '
            'speed=%d mm/s, sign=%+.1f, stop_delay=%.0f mm'
            % (
                self.route_turn_enabled,
                self.route_angles[1], self.route_angles[2], self.route_angles[3],
                self.route_turn_speed, self.route_heading_sign, self.stop_delay_mm,
            )
        )

    # ==================== 状态机钩子 ====================
    def _on_enter(self, state, previous_state):
        """在原节点进入动作之后补充本包的计时与日志。"""
        super()._on_enter(state, previous_state)
        if not self._enh_ready:
            return
        # 原节点进入 WAIT_ROUTE_MANEUVER 时可能直接 _fault，此时不再写本包状态。
        if self.state != state:
            return
        if state == 'WAIT_ROUTE_MANEUVER':
            self._junction_enter_ts = time.monotonic()
            self._turn_phase = 'idle'
            # 已经完成过一次转向时不要重复转，只负责把状态推回 ROUTE_FOLLOW。
            if not self._maneuver_done_once:
                self._route_done = False
            self._publish_comp('JUNCTION_PARKED', 'stopped at junction; waiting for route color')
        elif state in ('LINE_FOLLOW', 'ROUTE_FOLLOW'):
            self._junction_enter_ts = None
            if state == 'ROUTE_FOLLOW':
                self._turn_phase = 'idle'
                self._publish_comp(
                    'ROUTE_FOLLOW',
                    'route %s selected; resuming motor LINE mode'
                    % (ROUTE_NAME.get(self.selected_route, str(self.selected_route))),
                )

    # ==================== 路口事件去重 ====================
    def _junction_ignored(self, now):
        """转向完成后的屏蔽窗口内，忽略电机板重复锁存的路口事件。"""
        if not self._enh_ready or not self._route_done:
            return False
        return (now - self._junction_done_ts) < self.junction_ignore_s

    def _handle_chassis_status(self, data):
        """重写：屏蔽转向刚完成时固件仍锁存的路口事件位，避免被拉回路口。"""
        if self._enh_ready and len(data) == 12:
            event = data[11]
            if (event & EVENT_JUNCTION) and self._junction_ignored(time.monotonic()):
                data = bytes(data[:11]) + bytes((event & (~EVENT_JUNCTION & 0xFF),))
        super()._handle_chassis_status(data)

    def _handle_motor_frame(self, frame):
        """重写：屏蔽路口状态下电机状态字节 6（STATE_JUNCTION）的重复触发。"""
        cmd, data = frame
        if (
            self._enh_ready
            and cmd == CMD_STATUS
            and len(data) == 5
            and data[4] == STATE_JUNCTION
            and self.state in ('LINE_FOLLOW', 'ROUTE_FOLLOW')
            and self._junction_ignored(time.monotonic())
        ):
            # 仅把状态字节当作 RUNNING 透传，速度字段保持原值。
            data = bytes(data[:4]) + bytes((STATE_RUNNING,))
        super()._handle_motor_frame((cmd, data))

    # ==================== 颜色事件 ====================
    def _select_route(self, color):
        """重写原实现：只记录颜色，等路口停稳后再转向（符合规则 4.2）。"""
        if not self._enh_ready:
            return super()._select_route(color)
        if not self.route_turn_enabled:
            # 关闭增强时完全保持原行为。
            return super()._select_route(color)

        if color not in (1, 2, 3):
            return
        # 已经在分支循迹或本次路口动作已完成时忽略后续帧。
        if self.state == 'ROUTE_FOLLOW' or self._route_done or self._turn_phase != 'idle':
            return
        # 同色重复帧不重复发布，避免 20 Hz 刷屏。
        if self.selected_route == color and self._route_pub_cache == color:
            return

        self.selected_route = color
        self._route_pub_cache = color
        route_msg = UInt8()
        route_msg.data = color
        self.route_pub.publish(route_msg)

        if self.state == 'WAIT_ROUTE_MANEUVER':
            self.get_logger().info(
                'route color=%d (%s) received while parked at junction; starting maneuver'
                % (color, ROUTE_NAME.get(color, '?'))
            )
            self._publish_comp('ROUTE_COLOR_READY', 'color=%d received at junction' % color)
        elif self.state == 'LINE_FOLLOW' and self.turn_on_color_without_junction:
            self.get_logger().info(
                'route color=%d seen without junction event; stopping to maneuver' % color
            )
            self._dispatch('JUNCTION_DETECTED', 'route color seen; stopping to maneuver')
        else:
            self.get_logger().info(
                'route color=%d (%s) recorded; maneuver waits for junction stop event'
                % (color, ROUTE_NAME.get(color, '?'))
            )

    # ==================== 周期调度 ====================
    def _competition_tick(self):
        if not self._enh_ready:
            return
        now = time.monotonic()
        # STOP 定距停车计时优先，任何状态下都要推进。
        self._stop_delay_tick(now)
        if not self.started:
            return
        if self.state == 'WAIT_ROUTE_MANEUVER':
            self._junction_tick(now)
        elif self.state == 'WAIT_MOTOR_STOP' and self.refresh_zero_in_wait_stop:
            self._hold_zero(now, force_period=0.1)

    def _junction_tick(self, now):
        """路口阶段：等颜色 → 定角转向 → 稳定 → 恢复循迹。"""
        if self._turn_phase == 'idle':
            # 已经转过一次却又被打回路口：只重发转移，绝不重复转向。
            if self._maneuver_done_once:
                self._hold_zero(now)
                if self._junction_enter_ts is None:
                    self._junction_enter_ts = now
                if now - self._junction_enter_ts < 0.3:
                    return
                self._resume_retries += 1
                if self._resume_retries > 5:
                    self._fault('repeated junction re-entry after maneuver; giving up')
                    return
                self.get_logger().warning(
                    'junction re-entry after maneuver (attempt %d); forcing ROUTE_SELECTED'
                    % self._resume_retries
                )
                self._dispatch('ROUTE_SELECTED', 're-arm after junction re-entry')
                return
            if self._route_done:
                return
            if self.selected_route in (1, 2, 3):
                self._begin_turn(now)
                return
            # 没有颜色：保持零速（喂看门狗）并等待。
            self._hold_zero(now)
            if (
                self._junction_enter_ts is not None
                and now - self._junction_enter_ts > self.junction_color_wait
            ):
                self._fault(
                    'no valid route color within %.1f s while parked at junction'
                    % self.junction_color_wait
                )
            return

        if self._turn_phase == 'turning':
            self._turn_tick(now)
            return

        if self._turn_phase == 'settling':
            self._hold_zero(now)
            if now - self._settle_ts >= self.route_turn_settle:
                self._finish_turn()

    # ==================== 定角转向 ====================
    def _begin_turn(self, now):
        """开始转向；中路（0°）直接进入稳定阶段。"""
        angle = self.route_angles.get(self.selected_route, 0.0)
        self._publish_comp(
            'ROUTE_TURN_BEGIN',
            'color=%d target=%.1f deg' % (self.selected_route, angle),
        )

        if abs(angle) <= self.route_turn_tolerance:
            self.get_logger().info(
                'route %s needs no pivot turn; resuming line following'
                % (ROUTE_NAME.get(self.selected_route, '?'),)
            )
            self._turn_phase = 'settling'
            self._settle_ts = now
            self._hold_zero(now, force_period=0.0)
            return

        chassis = self.motor_chassis_status
        if chassis is None or (now - self.line_status_time) > 0.35:
            self._fault('heading telemetry unavailable before route turn')
            return

        self._turn_start_heading = float(chassis['heading_deg'])
        self._turn_target = float(angle)
        self._turn_started = now
        self._turn_phase = 'turning'
        self.get_logger().info(
            'route turn start: color=%s heading0=%.1f target=%+.1f deg sign=%+.1f'
            % (
                ROUTE_NAME.get(self.selected_route, '?'),
                self._turn_start_heading,
                self._turn_target,
                self.route_heading_sign,
            )
        )

    def _turn_tick(self, now):
        """开环定角：按 heading 增量与目标角的误差驱动差速旋转。"""
        chassis = self.motor_chassis_status
        if chassis is None or (now - self.line_status_time) > 0.35:
            self._hold_zero(now, force_period=0.0)
            self._fault('heading telemetry lost during route turn')
            return

        delta = _wrap180(
            (float(chassis['heading_deg']) - self._turn_start_heading) * self.route_heading_sign
        )
        error = self._turn_target - delta

        if abs(error) <= self.route_turn_tolerance:
            self._hold_zero(now, force_period=0.0)
            self._turn_phase = 'settling'
            self._settle_ts = now
            self.get_logger().info(
                'route turn reached: delta=%.1f deg (target %+.1f)'
                % (delta, self._turn_target)
            )
            self._publish_comp('ROUTE_TURN_REACHED', 'delta=%.1f deg' % delta)
            return

        if now - self._turn_started > self.route_turn_timeout:
            self._hold_zero(now, force_period=0.0)
            self._fault(
                'route turn timeout: delta=%.1f deg of target %+.1f deg'
                % (delta, self._turn_target)
            )
            return

        drive = 1 if error > 0 else -1
        speed = self.route_turn_speed
        # 差速原地旋转：左轮正转 + 右轮反转 = 顺时针（车身右转）。
        # 若实测转向与 heading 增量方向相反，把 route_heading_sign 置 -1.0。
        if not self._send_wheel_speeds(drive * speed, -drive * speed):
            self._fault('failed to command wheel speeds during route turn')
            return

    def _finish_turn(self):
        """转向完成：零速并回到 ROUTE_FOLLOW，交给电机板本地 PID 继续循迹。"""
        self._send_wheel_speeds(0, 0)
        self._route_done = True
        self._maneuver_done_once = True
        self._junction_done_ts = time.monotonic()
        self._turn_phase = 'idle'
        if not self._dispatch(
            'ROUTE_SELECTED',
            'route maneuver complete: %s' % (ROUTE_NAME.get(self.selected_route, '?'),),
        ):
            self._fault('route maneuver finished but ROUTE_SELECTED transition was rejected')

    # ==================== STOP 定距停车 ====================
    def _request_motor_stop(self):
        """重写：识别到 STOP 后可按里程再走一段再停车（默认 0 保持原行为）。"""
        if not self._enh_ready or self.stop_delay_mm <= 0.0:
            return super()._request_motor_stop()
        # 已在等待停稳或已在校准里程时忽略重复请求（每帧都会调用一次）。
        if self.state == 'WAIT_MOTOR_STOP' or self._stop_delay_active:
            return

        chassis = self.motor_chassis_status
        if chassis is None or (time.monotonic() - self.line_status_time) > 0.35:
            self.get_logger().warning(
                'stop_delay skipped: motor chassis telemetry stale; stopping immediately'
            )
            return super()._request_motor_stop()

        self._stop_delay_active = True
        self._stop_delay_base_mm = int(chassis['distance_mm'])
        self._stop_delay_started = time.monotonic()
        self.get_logger().info(
            'STOP marker seen; delaying stop until travelled %.0f mm (base=%d mm)'
            % (self.stop_delay_mm, self._stop_delay_base_mm)
        )
        self._publish_comp(
            'STOP_DELAY_BEGIN',
            'travelling %.0f mm before zero speed' % self.stop_delay_mm,
        )

    def _stop_delay_tick(self, now):
        """里程闭环：走够 stop_delay_mm 或超时后真正下发零速。"""
        if not self._stop_delay_active:
            return
        travelled = None
        chassis = self.motor_chassis_status
        if chassis is not None and (now - self.line_status_time) <= 0.35:
            travelled = abs(int(chassis['distance_mm']) - self._stop_delay_base_mm)

        if travelled is not None and travelled >= self.stop_delay_mm:
            self.get_logger().info('stop delay reached: travelled %d mm' % travelled)
        elif now - self._stop_delay_started > self.stop_delay_timeout:
            self.get_logger().warning(
                'stop delay timeout after %.1f s; stopping now' % (now - self._stop_delay_started)
            )
        else:
            return

        self._stop_delay_active = False
        super()._request_motor_stop()

    # ==================== 工具 ====================
    def _hold_zero(self, now, force_period=None):
        """按频率下发零速；既保持静止又喂饱电机板 300 ms 看门狗。"""
        if not self.junction_hold_enable and force_period is None:
            return
        period = self.junction_hold_period if force_period is None else force_period
        if now - self._last_zero_ts < period:
            return
        self._last_zero_ts = now
        if not self._send_wheel_speeds(0, 0):
            self._fault('failed to hold zero speed during junction maneuver')

    def _publish_comp(self, phase, detail):
        message = String()
        message.data = json.dumps(
            {
                'phase': phase,
                'detail': str(detail),
                'route': None if self.selected_route is None else int(self.selected_route),
                'state': self.state,
                'turn_phase': self._turn_phase,
            },
            ensure_ascii=False,
        )
        self.competition_pub.publish(message)
        self.get_logger().info('competition phase=%s: %s' % (phase, detail))
