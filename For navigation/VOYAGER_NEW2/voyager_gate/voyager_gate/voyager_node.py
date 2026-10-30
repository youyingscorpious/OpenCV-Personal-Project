# -*- coding: utf-8 -*-
"""VoyagerNode：在 CompetitionNode 之上挂载路口「停车 → 色卡识别」闸门。

继承链（VOYAGER_NEW2 增量）
--------------------------
::

    NavigationNode         (robopioneers_navigation，原件只读)
      └── CompetitionNode  (competition_base.py，模板副本：定角转向 + STOP 定距)
            └── VoyagerNode (本文件，只做接线)
                  ├── JunctionGateMixin   (junction_gate.py：G1→G4 闸门判据)
                  └── VisionAdapterMixin   (vision_adapter.py：同进程 import nav_vision)

本文件只做「接线」，不重复实现算法：

* 转向与停车逻辑 → ``competition_base.py``
* 闸门判据        → ``junction_gate.py``
* 同进程视觉桥接  → ``vision_adapter.py``

集成点共 5 处，全部通过方法重写 + ``super()`` 完成，不修改任何父类。
``vision_inline_enabled:=false``（默认）时，VisionAdapter 完全不工作，
行为与 VOYAGER_NEW 原版逐字段一致。
"""

import time

import rclpy
from std_msgs.msg import UInt8

from .competition_base import CompetitionNode, ROUTE_NAME
from .junction_gate import JunctionGateMixin
from .vision_adapter import VisionAdapterMixin


class VoyagerNode(VisionAdapterMixin, JunctionGateMixin, CompetitionNode):
    """路口「停车确认 → 识别窗口 → 颜色投票 → 定角转向」导航节点。"""

    def __init__(self):
        # 先让 CompetitionNode 完成全部原逻辑：
        # 参数、串口、视觉链路、服务、20 ms 轮询定时器。
        super().__init__()

        # 闸门参数与运行期字段。必须放在 super() 之后：
        # declare_parameter 需要节点已完成初始化。
        self._gate_declare()
        self._gate_read()
        self._gate_init()

        # 同进程视觉适配器（VOYAGER_NEW2 增量；默认关）
        self._vision_declare()
        self._vision_read()
        self._vision_init()

        self.get_logger().info(
            'voyager gate ready: enabled=%s samples=%d dwell=%.2fs timeout=%.2fs(%s) '
            'require_after_stop=%s vote=%d frames/%s | vision_inline=%s'
            % (
                self.junction_gate_enabled,
                self.stop_confirm_samples,
                self.stop_confirm_dwell,
                self.stop_confirm_timeout,
                self.stop_confirm_timeout_action,
                self.color_require_after_stop,
                self.color_vote_frames,
                ('%.2fs' % self.color_vote_timeout) if self.color_vote_frames > 1 else 'n/a',
                self._vision_inline,
            )
        )

    # ==================== 集成点 1：记下 0x10 实测轮速帧的时刻 ====================
    def _handle_motor_frame(self, frame):
        cmd, data = frame
        self._gate_note_motor_frame(cmd, data)
        # 父类可能改写 data（屏蔽锁存的路口事件），所以传入原始 frame。
        super()._handle_motor_frame(frame)

    # ==================== 集成点 2：进入路口，闸门复位 ====================
    def _on_enter(self, state, previous_state):
        if (
            self._enh_ready
            and state == 'WAIT_ROUTE_MANEUVER'
            and self.state == state
            and not self._maneuver_done_once
        ):
            # I2：丢掉行进途中被误锁的颜色，本轮只认停稳之后的识别结果。
            self.selected_route = None
            self._route_pub_cache = None
            # 闸门复位到 G1 ARMING，并关闭 color_request。
            self._gate_on_junction_enter()
        super()._on_enter(state, previous_state)

    # ==================== 集成点 3：闸门推进 + 未停稳禁止转向 ====================
    def _junction_tick(self, now):
        if not self.junction_gate_enabled or self.state != 'WAIT_ROUTE_MANEUVER':
            return super()._junction_tick(now)

        # 同一 tick 内先确认停稳，再判断能否转向，避免多等一个 20 ms 周期。
        self._gate_tick(now)
        # _gate_tick 在 fault 模式下会把状态切走，此时不再推进转向逻辑。
        if self.state != 'WAIT_ROUTE_MANEUVER':
            return

        # I1：没有停稳证据时禁止转向，但仍以零速刷帧喂 300 ms 看门狗。
        # 仅拦 idle 相位：turning / settling 必须放行，_maneuver_done_once
        # 的「被打回路口」分支也由 _gate_blocks_turn() 内部放行。
        if self._turn_phase == 'idle' and self._gate_blocks_turn():
            self._hold_zero(now)
            return

        super()._junction_tick(now)

    # ==================== 集成点 4：颜色采信（闸门 + 投票） ====================
    def _select_route(self, color):
        """重写：颜色只在「停稳后的识别窗口」里投票锁定（规则 4.2）。

        调用方有两个：
          * 原 UDP 5000 视觉帧回调（板端直发编号时）；
          * VOYAGER_NEW2 增量：VisionAdapterMixin._vision_tick 里
            ``nv.color()`` 返回 "1"/"2"/"3" 时转 int 后调本函数。
        两路共用同一道闸门投票，不会绕过 G1/G2。
        """
        if not self._enh_ready or not self.route_turn_enabled:
            # 关闭增强时完全保持原节点行为。
            return super()._select_route(color)
        if color not in (1, 2, 3):
            return
        if self.state == 'ROUTE_FOLLOW' or self._route_done or self._turn_phase != 'idle':
            return
        # I3：已锁定颜色，后续帧一律忽略。
        if self.selected_route in (1, 2, 3):
            return

        now = time.monotonic()
        # I2：停稳之前的颜色帧不记录、不投票（行进中 / 路口未确认停稳）。
        if not self._gate_allows_color(color, now):
            return
        # 去抖：连续 N 帧同色才锁定。
        if not self._gate_vote(color, now):
            return

        self.selected_route = color
        self._route_pub_cache = color
        route_msg = UInt8()
        route_msg.data = color
        self.route_pub.publish(route_msg)

        if self.state == 'WAIT_ROUTE_MANEUVER':
            self._publish_comp(
                'ROUTE_COLOR_READY',
                'color=%d confirmed by %d frames at junction'
                % (color, self._vote_count),
            )
        elif self.state == 'LINE_FOLLOW' and self.turn_on_color_without_junction:
            # D7 应急路径：没有路口事件也停车转向，颜色投票仍然生效。
            self._publish_comp(
                'ROUTE_COLOR_READY',
                'color=%d seen without junction event (fallback)' % color,
            )
            self._dispatch('JUNCTION_DETECTED',
                           'route color seen; stopping to maneuver')
        else:
            self.get_logger().info(
                'route color=%d (%s) recorded; maneuver waits for junction stop'
                % (color, ROUTE_NAME.get(color, '?'))
            )

    # ==================== 集成点 5：转向完成，关闭识别窗口 ====================
    def _finish_turn(self):
        super()._finish_turn()
        self._gate_on_turn_finish()


def main(args=None):
    rclpy.init(args=args)
    node = VoyagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('voyager node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
