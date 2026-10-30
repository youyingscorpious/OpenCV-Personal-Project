# -*- coding: utf-8 -*-
"""同进程视觉适配器：把 ``nav_vision`` 的结果喂给导航节点。

所属
----
``VOYAGER_NEW2`` 在原 ``voyager_gate`` 基础上的增量模块。
对应视觉模块 ``vision_nav/nav_vision.py`` 提供的同进程 API：
``nv.color()`` 返回 "1"/"2"/"3"/"0"/"fuck"，``nv.stop()`` 返回 "1"/"0"/"fuck"。

为什么需要这层胶水
------------------
原 ``NavigationNode`` 是按"板端发 AA55 协议帧"设计的，有三个硬门槛：

* **B1 开赛门槛**：``/navigation/start`` 要求 ``vision_roles == {1, 2}``，
  而 ``vision_roles`` 只在收到 AA55 ``VISION_VERSION`` 帧时填充。
  inline 模式下板端只发 JPEG，一帧 AA55 都没有，开不了赛。
* **B2 视觉看门狗**：``_check_vision_timeout`` 在 LINE_FOLLOW/ROUTE_FOLLOW 下，
  要求 ``vision_last_any`` 和 ``vision_last_seen[(1,VISION_TARGET_LIST)]``
  持续刷新（1.5 s / 0.6 s），否则进 FAULT。这两个时间戳也只由 AA55 帧刷新。
* **F1 投票架空**：``nav_vision`` 结果槽 0.5 s 保鲜（STALE_RESULT_S），
  30 ms 轮询会把同一帧结果读 ~16 次，``route_color_vote_frames=3`` 退化成
  "1 帧锁定"，I3 不变量失效。

本适配器在**不改原包一个字节**的前提下，用运行时字段写入满足这三个门槛：
B1/B2 不是伪造链路——inline 模式下视觉链路确实活着（nav_vision 后台线程
在收 UDP 5002 图像流），只是它不走 AA55 协议；这里只是诚实地把"链路活着"
这个事实告诉父类的看门狗。F1 按 ``nv.status()['last_color_age_ms']`` 边沿
触发：age 变小才算新帧，才给闸门投一票。

第二轮 review 又补了 5 处（都在本文件内，仍不动父类）：

* **P1 启动顺序**：``self.started`` 为 False 时不统计 ``"fuck"``。否则板子比
  节点晚 2 s 出图就会 ``_fault()``，把 ``started`` 置 True，
  ``/navigation/start`` 此后永远返回 ``already started: FAULT``。
* **P2 FAULT 只跳一次**：计数器只增不减，越界条件会一直成立，加
  ``state != 'FAULT'`` 判断，避免 33 Hz 反复发 ``CMD_ESTOP``。
* **P3 提示说实话**：import 失败时不再谎称"回退到 UDP 5000"——本部署板端
  只发 JPEG，没有那条退路，实际后果是开不了赛。
* **P4 role 重新断言**：``_reset_vision_session()`` 会清空 ``vision_roles``，
  每 tick 重新写一次，链路上偶发 OSError 不会打掉开赛门槛。
* **P5 age 取不到就不投票**：``nv.status()`` 异常时不再退化成"每 tick 一票"。

依赖宿主类（VoyagerNode / CompetitionNode / NavigationNode）提供：
    state, started, vision_roles, vision_enabled, vision_last_any,
    vision_last_seen, _select_route(color:int), _request_motor_stop(),
    _publish_comp(phase, detail), _fault(reason),
    get_logger(), declare_parameter, get_parameter, create_timer
"""

import os
import sys
import time

# VISION_TARGET_LIST = 0x01（与 robopioneers_navigation.protocols 一致）。
# 离线自检时原包不可用，用常量值兜底。
try:
    from robopioneers_navigation.protocols import VISION_TARGET_LIST
except ImportError:  # pragma: no cover - 离线兜底
    VISION_TARGET_LIST = 0x01


class VisionAdapterMixin(object):
    """把 nav_vision 的 color()/stop() 桥接到导航节点已有钩子。"""

    # ---- 类级默认值：保证 super().__init__() 期间属性已存在 ----
    _vision_inline = False
    _vision_nv = None
    _vision_timer = None
    _vision_lost_count = 0
    _vision_last_status_log = 0.0
    _vision_stop_now = False
    _vision_poll_period = 0.03
    _vision_lost_fault = 5
    _vision_suppress_color_on_stop = True
    _vision_status_period = 1.0
    # F1：按帧去重——上次读到的 last_color_age_ms
    _last_color_age_ms = None
    # STOP 边沿触发：上一帧 stop 是否为 1
    _stop_was_seen = False

    # ==================== 初始化 ====================
    def _vision_declare(self):
        """① 声明参数。全部带默认值，YAML 不写也能跑（默认关）。"""
        d = self.declare_parameter
        d('vision_inline_enabled', False)
        d('vision_module_path', '')
        d('vision_poll_period_s', 0.03)
        d('vision_stream_lost_fault', 5)
        d('vision_suppress_color_on_stop', True)
        d('vision_status_log_hz', 1.0)

    def _vision_read(self):
        """② 读取参数到实例字段。"""
        p = self.get_parameter
        self._vision_inline = bool(p('vision_inline_enabled').value)
        self._vision_module_path = str(p('vision_module_path').value).strip()
        self._vision_poll_period = max(0.01, float(p('vision_poll_period_s').value))
        self._vision_lost_fault = max(0, int(p('vision_stream_lost_fault').value))
        self._vision_suppress_color_on_stop = bool(
            p('vision_suppress_color_on_stop').value)
        self._vision_status_period = 1.0 / max(
            0.1, float(p('vision_status_log_hz').value))

    def _vision_init(self):
        """③ sys.path + import nav_vision，启动轮询定时器。"""
        if not self._vision_inline:
            return

        if self._vision_module_path:
            if not os.path.isdir(self._vision_module_path):
                self.get_logger().warning(
                    'vision_inline: module path not found: %s; inline vision DISABLED'
                    % self._vision_module_path)
                self._vision_inline = False
                return
            if self._vision_module_path not in sys.path:
                sys.path.insert(0, self._vision_module_path)

        try:
            import nav_vision as nv  # noqa: E402  延迟 import
        except ImportError as exc:
            self.get_logger().error(
                'vision_inline: import nav_vision FAILED (%s)。\n'
                '  请确认 vision_module_path 指向 nav_vision.py 所在目录，'
                '且已在本机 pip install opencv-python numpy。\n'
                '  inline vision DISABLED。注意：本部署板端只发 JPEG，没有 AA55 '
                '后备链路，此时 /navigation/start 会因 vision_roles 为空被拒；'
                '要么修好本路径，要么改用 start_vision_bridge:=true。' % exc)
            self._vision_inline = False
            return

        self._vision_nv = nv

        try:
            nv.start(color=True, stop=True)
        except Exception as exc:
            # nv.start 只是预激活；首次 color()/stop() 仍会自动起线程，
            # 所以这里只告警，不把 _vision_inline 置 False。
            self.get_logger().warning('vision_inline: nv.start() raised: %s' % exc)

        # ---- B1 胶水：满足父类 /navigation/start 的 role 握手门槛 ----
        # inline 模式下板端不发 AA55 VERSION 帧，但视觉链路确实活着
        # （nav_vision 后台线程在收 UDP 5002）。这里诚实地把"两路视觉已
        # 握手"这个事实告诉父类，否则 /navigation/start 永远返回
        # "waiting for vision roles 1 and 2; got []"。
        try:
            self.vision_roles = {1, 2}
            self.vision_enabled = True
        except Exception as exc:
            self.get_logger().warning(
                'vision_inline: could not pre-seed vision_roles: %s' % exc)

        self._vision_timer = self.create_timer(
            self._vision_poll_period, self._vision_tick)

        self.get_logger().info(
            'vision_inline READY: poll=%.0fms module=%s suppress_color_on_stop=%s'
            % (self._vision_poll_period * 1000.0,
               self._vision_module_path or '(PYTHONPATH)',
               self._vision_suppress_color_on_stop))

    # ==================== 主轮询 ====================
    def _vision_tick(self):
        """每 vision_poll_period_s 调一次：读 color/stop，喂给导航钩子。"""
        if not self._vision_inline or self._vision_nv is None:
            return
        nv = self._vision_nv
        now = time.monotonic()

        # ---- B2 胶水：喂住父类视觉看门狗 ----
        # _check_vision_timeout 要求 vision_last_any 1.5 s 内、
        # vision_last_seen[(1,VISION_TARGET_LIST)] 0.6 s 内被刷新。
        # inline 模式下没有 AA55 帧，这里每 30 ms 诚实地标记一次"链路活着"。
        try:
            self.vision_last_any = now
            self.vision_last_seen[(1, VISION_TARGET_LIST)] = now
            # P4：_close_vision_link() 会经 _reset_vision_session() 清空
            # vision_roles，而 B1 的 seed 只在 _vision_init 里跑过一次。
            # 这里每 tick 重新断言，链路上偶发一次 OSError 也不会把开赛门槛打掉。
            if self.vision_roles != {1, 2}:
                self.vision_roles = {1, 2}
        except Exception:
            pass

        # 1) STOP 先读（决定是否屏蔽当帧颜色）
        stop_raw = '0'
        try:
            stop_raw = nv.stop()
        except Exception as exc:
            self.get_logger().warning('nv.stop() raised: %s' % exc)
        stop_now = (stop_raw == '1')

        # STOP 边沿触发：只在 "0"→"1" 的那一帧调 _request_motor_stop，
        # 避免结果槽 0.5 s 保鲜导致 33 Hz 重复调用。
        if stop_now and not self._stop_was_seen:
            try:
                self._request_motor_stop()
            except Exception as exc:
                self.get_logger().warning(
                    '_request_motor_stop() raised: %s' % exc)
        self._stop_was_seen = stop_now
        self._vision_stop_now = stop_now

        # 2) 颜色（value 与 age 分两次取，分别报错，便于定位是哪个调用坏了）
        color_raw = '0'
        color_age_ms = None
        try:
            color_raw = nv.color()
        except Exception as exc:
            self.get_logger().warning('nv.color() raised: %s' % exc)
        try:
            color_age_ms = nv.status().get('last_color_age_ms')
        except Exception as exc:
            self.get_logger().warning('nv.status() raised: %s' % exc)

        # 链路死：累计计数，到上界进 FAULT
        if color_raw == 'fuck':
            if not self.started:
                # P1：未开赛前（板子还在开机 / 还没连上）不算故障。
                # 否则 _fault() 会把 self.started 置 True，此后
                # /navigation/start 永远返回 'already started: FAULT'，
                # 必须重启节点才能恢复——这是"先 launch 后上电"的必踩坑。
                # 开赛前车不会动，跳过计数没有安全损失；但**不 return**，
                # 继续往下发 VISION_STATUS 诊断（正是排查链路时最需要的）。
                color_raw = '0'
            else:
                self._vision_lost_count += 1
                if self._vision_lost_count == 1:
                    self._publish_comp('COLOR_STREAM_LOST',
                                       'nav_vision color returned "fuck"')
                # P2：只在第一次越界时 fault。计数器只增不减，条件会一直成立，
                # 不加 state 判断就会 33 Hz 反复 _fault()，每次都给电机板 /
                # 外设板各发一帧 CMD_ESTOP，并反复 _dispatch('FAULT')。
                if (self._vision_lost_fault > 0
                        and self._vision_lost_count >= self._vision_lost_fault
                        and self.state != 'FAULT'):
                    self._fault(
                        'nav_vision color stream dead for %d polls'
                        % self._vision_lost_count)
                return

        if self._vision_lost_count > 0:
            self._publish_comp(
                'COLOR_STREAM_RECOVERED',
                'after %d lost polls' % self._vision_lost_count)
        self._vision_lost_count = 0

        # 红色 STOP 牌屏蔽：stop=="1" 时这一帧不喂 color
        if self._vision_suppress_color_on_stop and self._vision_stop_now:
            color_raw = '0'

        # ---- F1 胶水：按帧去重，不把同一帧缓存读 16 次 ----
        # last_color_age_ms 是"距离上次 color 结果更新的毫秒数"。
        # 新帧到来时 age 重置为小值，然后随时间增长。
        # 只有 age 比上次读到的更小（新帧到来）才投票；age 不变/变大说明
        # 还在同一次识别结果的 0.5 s 保鲜窗口里，跳过。
        is_new_color_frame = False
        if color_raw in ('1', '2', '3'):
            if color_age_ms is None:
                # P5：拿不到结果时间戳（nv.status() 异常 / 槽里还没有时间戳）时
                # 不投票。宁可这次不锁（最多 8 s 后由闸门兜底 FAULT），
                # 也不能退化成"每 tick 一票"把同一帧读成 3 帧。
                is_new_color_frame = False
            elif self._last_color_age_ms is None:
                # 首次拿到有效 age：按新帧处理（这是真实的一次识别）
                is_new_color_frame = True
            elif color_age_ms < self._last_color_age_ms:
                # age 变小 = 后台线程刚更新了结果槽
                is_new_color_frame = True
            # 记录本次 age（不管是否投票，下次比较要用）
            self._last_color_age_ms = color_age_ms
        else:
            # color 是 "0" 时也重置 age 基准，下次新帧能正确边沿触发
            self._last_color_age_ms = color_age_ms

        if is_new_color_frame and color_raw in ('1', '2', '3'):
            try:
                self._select_route(int(color_raw))
            except Exception as exc:
                self.get_logger().warning(
                    '_select_route(%s) raised: %s' % (color_raw, exc))

        # 3) 周期性诊断：fps / 帧龄 / 最近错误
        if (now - self._vision_last_status_log) >= self._vision_status_period:
            self._vision_last_status_log = now
            try:
                st = nv.status()
                self._publish_comp(
                    'VISION_STATUS',
                    'fps=%.1f frame_age=%sms color=%s stop=%s err=%s' % (
                        st.get('fps', 0.0),
                        st.get('last_frame_age_ms'),
                        st.get('last_color'),
                        st.get('last_stop'),
                        st.get('last_error'),
                    ))
            except Exception:
                pass
