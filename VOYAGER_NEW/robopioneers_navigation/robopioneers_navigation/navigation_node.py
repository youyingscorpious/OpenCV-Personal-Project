# 导入 JSON：以可读结构发布视觉帧和导航诊断信息。
import json  # 使用标准库编码 ROS String 消息。
import struct  # 按 MCU 协议的小端格式编码左右轮速度。
# 导入 socket：通过视觉模块配置的 UDP、TCP 网络链路接收帧。
import socket  # 使用非阻塞网络 I/O，避免阻塞 ROS 定时器。
# 导入 time：记录超时、停止稳定时间和重连时间。
import time  # 所有超时都使用单调时钟。
# 导入 ROS 2 Python 客户端库。
import rclpy  # 初始化并运行 ROS 2 节点。
# 导入 ROS 2 节点基类。
from rclpy.node import Node  # 定义单一主控节点。
# 导入标准 ROS 字符串消息。
from std_msgs.msg import String, UInt8, Bool  # 发布诊断、颜色与状态。
# 导入标准 Trigger 服务类型。
from std_srvs.srv import Trigger  # 提供赛前启动流程服务。
# 导入 PySerial：仅在串口配置成功时使用。
import serial  # 以非阻塞方式读写两块单片机 UART。
# 导入本包的协议常量、帧构造、流解析和视觉字段解析。
from robopioneers_navigation.protocols import CMD_ARM, CMD_CHASSIS, CMD_STATUS, CMD_ESTOP, CMD_PING, CMD_LINE, CMD_CHASSIS_STATUS, VISION_TARGET_LIST, VISION_TARGET_COORD, VISION_HEARTBEAT, VISION_ERROR, VISION_VERSION, STATE_IDLE, STATE_RUNNING, STATE_ARM_BUSY, STATE_ARM_DONE, STATE_FAULT, STATE_TIMEOUT, STATE_JUNCTION, STATE_LINE_LOST, EVENT_JUNCTION, EVENT_LINE_LOST, StreamParser, build_electrical_frame, parse_status, parse_chassis_status, parse_peripheral_status, parse_target_list, parse_target_coord, parse_vision_heartbeat, parse_vision_error  # noqa: E501

# 声明导航流程允许的显式状态转移；星号表示任意当前状态。
TRANSITIONS = {
    ('WAIT_START', 'START'): 'LINE_FOLLOW',  # 启动后进入循迹阶段。
    ('LINE_FOLLOW', 'ROUTE_SELECTED'): 'ROUTE_FOLLOW',  # 收到路线颜色后进入分支阶段。
    ('LINE_FOLLOW', 'PAUSE'): 'PAUSED',  # 循迹时允许受控暂停。
    ('ROUTE_FOLLOW', 'PAUSE'): 'PAUSED',  # 分支循迹时允许受控暂停。
    ('PAUSED', 'RESUME'): None,  # None 表示恢复到暂停前保存的阶段。
    ('LINE_FOLLOW', 'JUNCTION_DETECTED'): 'WAIT_ROUTE_MANEUVER',  # 路口处安全停车等待选线能力。
    ('ROUTE_FOLLOW', 'JUNCTION_DETECTED'): 'WAIT_ROUTE_MANEUVER',  # 路口处安全停车等待选线能力。
    ('LINE_FOLLOW', 'STOP_MARKER'): 'WAIT_MOTOR_STOP',  # STOP 标志后等待实测停稳。
    ('ROUTE_FOLLOW', 'STOP_MARKER'): 'WAIT_MOTOR_STOP',  # STOP 标志后等待实测停稳。
    ('WAIT_ROUTE_MANEUVER', 'STOP_MARKER'): 'WAIT_MOTOR_STOP',  # 路口等待期间收到 STOP 仍须停车并确认停稳。
    ('WAIT_MOTOR_STOP', 'MOTOR_ZERO'): 'ARM_ACTION',  # 停稳确认后才允许启动舵机动作。
    ('ARM_ACTION', 'ARM_SEQUENCE_DONE'): 'COMPLETE',  # 抓取、放置和复位全部完成后结束任务。
    ('*', 'FAULT'): 'FAULT',  # 任意状态都允许进入故障安全状态。
}

# 定义 ROS 2 导航主控节点：管理视觉、两路 UART 和全流程状态。
class NavigationNode(Node):
    # 初始化 ROS 节点、参数、通信接口、发布器、服务和定时器。
    def __init__(self):
        # 向 ROS 图注册节点名称。
        super().__init__('robopioneers_navigation')
        # 声明电机单片机 UART 设备路径。
        self.declare_parameter('motor_port', '/dev/ttyUSB0')
        # 声明舵机单片机 UART 设备路径。
        self.declare_parameter('servo_port', '/dev/ttyUSB1')
        # 声明电机 UART 波特率。
        self.declare_parameter('motor_baud', 115200)
        # 声明舵机 UART 波特率。
        self.declare_parameter('servo_baud', 115200)
        # 使用第三版电机固件的 AA55 CRC16 速度协议。
        self.declare_parameter('motor_profile', 'core_crc16')
        # 声明视觉传输类型：匹配 Communication_comm.py 默认 UDP，也保留 TCP/串口。
        self.declare_parameter('vision_transport', 'udp')
        # 声明可选 TCP 模式的视觉服务端地址，UDP 模式不使用此参数。
        self.declare_parameter('vision_host', '192.168.4.1')
        # 声明视觉 TCP 端口，按协议表默认值配置。
        self.declare_parameter('vision_port', 5000)
        # 声明 UDP 本地绑定地址，0.0.0.0 接收任意网卡收到的单播和广播。
        self.declare_parameter('vision_udp_bind_address', '0.0.0.0')
        # 声明视觉模块默认发送的 UDP 目的端口。
        self.declare_parameter('vision_udp_bind_port', 5000)
        # 声明视觉 UART 设备，只有 vision_transport=serial 时使用。
        self.declare_parameter('vision_port_device', '/dev/ttyUSB2')
        # 声明视觉 UART 波特率，只有串口模式时使用。
        self.declare_parameter('vision_baud', 115200)
        # 第三版电机状态包直接回报编码器换算的实测轮速。
        self.declare_parameter('motor_status_is_measured', True)
        # 声明电机板本地循迹基础速度，单位 mm/s，固件范围为 100 到 800。
        self.declare_parameter('motor_line_base_speed_mm_s', 150)
        # 声明电机板本地循迹差速限幅，单位 mm/s，固件范围为 50 到 600。
        self.declare_parameter('motor_line_turn_limit_mm_s', 300)
        # 声明判定车辆停稳所需的连续状态帧数量。
        self.declare_parameter('stopped_samples_required', 3)
        # 声明判定停稳允许的左右轮速度绝对值，单位与 STM32 状态帧一致。
        self.declare_parameter('stopped_speed_threshold', 10)
        # 声明停止状态持续时间下限，避免单帧零速误判。
        self.declare_parameter('stopped_dwell_seconds', 0.5)
        # 声明等待电机确认停稳的超时秒数。
        self.declare_parameter('motor_stop_timeout_seconds', 5.0)
        # 声明等待每个机械臂动作完成的超时秒数。
        self.declare_parameter('arm_action_timeout_seconds', 15.0)
        # 声明相机 1 目标列表超时阈值，兼容 20 Hz 限频及短时 UDP 丢包。
        self.declare_parameter('vision_source_timeout_seconds', 0.6)
        # 声明整条视觉链路超时阈值，留出 1 Hz 自动心跳的调度余量。
        self.declare_parameter('vision_link_timeout_seconds', 1.5)
        # 读取用户配置的电机 UART 参数。
        self.motor_port = self.get_parameter('motor_port').value
        # 读取用户配置的舵机 UART 参数。
        self.servo_port = self.get_parameter('servo_port').value
        # 读取电机串口速率。
        self.motor_baud = int(self.get_parameter('motor_baud').value)
        # 读取舵机串口速率。
        self.servo_baud = int(self.get_parameter('servo_baud').value)
        # 读取电机协议模式。
        self.motor_profile = str(self.get_parameter('motor_profile').value)
        # 读取视觉传输模式。
        self.vision_transport = str(self.get_parameter('vision_transport').value)
        # 读取是否使用视觉 TCP。
        self.vision_host = str(self.get_parameter('vision_host').value)
        # 读取视觉 TCP 端口。
        self.vision_tcp_port = int(self.get_parameter('vision_port').value)
        # 读取 UDP 绑定地址。
        self.vision_udp_bind_address = str(self.get_parameter('vision_udp_bind_address').value)
        # 读取 UDP 绑定端口。
        self.vision_udp_bind_port = int(self.get_parameter('vision_udp_bind_port').value)
        # 读取视觉串口设备。
        self.vision_port_device = str(self.get_parameter('vision_port_device').value)
        # 读取视觉串口波特率。
        self.vision_baud = int(self.get_parameter('vision_baud').value)
        # 读取停稳速度字段可信性配置。
        self.status_is_measured = bool(self.get_parameter('motor_status_is_measured').value)
        # 读取并约束电机板本地循迹基础速度。
        self.motor_line_base_speed = max(100, min(800, int(self.get_parameter('motor_line_base_speed_mm_s').value)))
        # 读取并约束电机板本地循迹差速限幅。
        self.motor_line_turn_limit = max(50, min(600, int(self.get_parameter('motor_line_turn_limit_mm_s').value)))
        # 读取连续零速帧数要求。
        self.stopped_samples_required = int(self.get_parameter('stopped_samples_required').value)
        # 读取停稳速度阈值。
        self.stopped_speed_threshold = int(self.get_parameter('stopped_speed_threshold').value)
        # 读取停车保持时间。
        self.stopped_dwell_seconds = float(self.get_parameter('stopped_dwell_seconds').value)
        # 读取等待电机停稳超时。
        self.motor_stop_timeout = float(self.get_parameter('motor_stop_timeout_seconds').value)
        # 读取等待机械臂动作超时。
        self.arm_timeout = float(self.get_parameter('arm_action_timeout_seconds').value)
        # 读取视觉单路数据超时。
        self.vision_source_timeout = float(self.get_parameter('vision_source_timeout_seconds').value)
        # 读取视觉整链路超时。
        self.vision_link_timeout = float(self.get_parameter('vision_link_timeout_seconds').value)
        # 创建电机 UART 状态帧解析器。
        self.motor_parser = StreamParser('electrical')
        # 创建舵机 UART 状态帧解析器。
        self.servo_parser = StreamParser('electrical')
        # 创建视觉 SUM8 帧解析器。
        self.vision_parser = StreamParser('vision')
        # 初始化电机串口句柄为空。
        self.motor_serial = None
        # 初始化舵机串口句柄为空。
        self.servo_serial = None
        # 初始化视觉 UART 句柄为空。
        self.vision_serial = None
        # 初始化视觉 TCP 套接字为空。
        self.vision_socket = None
        # 初始化视觉 UDP 套接字为空。
        self.vision_udp_socket = None
        # 初始化最近收到视觉 UDP 报文的对端地址。
        self.vision_udp_peer = None
        # 初始化所有链路下次重连时间为立即尝试。
        self.next_reconnect = 0.0
        # 初始化视觉网络链路就绪标志。
        self.vision_connected = False
        # 初始化双相机角色上线记录。
        self.vision_roles = set()
        # 记录视觉双角色握手是否完成，视觉识别由发送端 API 循环驱动。
        self.vision_enabled = False
        # 初始化各视觉来源和命令的最后接收时刻。
        self.vision_last_seen = {}
        # 初始化最近一帧视觉有效数据时间。
        self.vision_last_any = 0.0
        # 初始化 ROS 状态机阶段为尚未启动。
        self.state = 'WAIT_START'
        # 初始化赛程已启动标志。
        self.started = False
        # 记录暂停前所处的赛程阶段，供 /navigation/resume 恢复。
        self.paused_from_state = None
        # 初始化机械臂序列，按抓取、放置、复位顺序执行。
        self.arm_actions = [1, 2, 3]
        # 初始化当前机械臂序列位置。
        self.arm_action_index = 0
        # 初始化机械臂动作等待中的忙状态确认标志。
        self.arm_saw_busy = False
        # 初始化机械臂动作发出时刻。
        self.arm_action_started = 0.0
        # 初始化电机停稳连续确认帧计数。
        self.stopped_samples = 0
        # 初始化首次满足停稳条件的时刻。
        self.stopped_since = None
        # 初始化等待电机停稳的开始时刻。
        self.stop_wait_started = 0.0
        # 初始化最新电机速度和状态。
        self.motor_status = None
        # 初始化电机板底盘遥测和循迹位图。
        self.motor_chassis_status = None
        self.line_bits = None
        # 初始化舵机外设板在线状态。
        self.servo_status_seen = False
        # 初始化循迹遥测接收时刻。
        self.line_status_time = 0.0
        # 标记电机板本地循迹模式是否已启用。
        self.motor_line_mode_active = False
        # 记录上次电机心跳时刻，低于固件 300 ms 看门狗上限。
        self.last_motor_ping = 0.0
        # 缓存最近一次底盘事件，供状态诊断。
        self.last_motor_event = 0
        # 初始化视觉选定路线。
        self.selected_route = None
        # 初始化最近一次控制输出，供日志和故障停车使用。
        self.last_wheel_command = (0, 0)
        # 创建视觉目标列表 ROS 发布器。
        self.target_list_pub = self.create_publisher(String, '/navigation/vision/target_list', 10)
        # 创建视觉坐标 ROS 发布器。
        self.target_coord_pub = self.create_publisher(String, '/navigation/vision/target_coord', 10)
        # 创建视觉心跳 ROS 发布器，公开视觉状态和处理性能。
        self.vision_heartbeat_pub = self.create_publisher(String, '/navigation/vision/heartbeat', 10)
        # 创建视觉颜色 ROS 发布器。
        self.color_pub = self.create_publisher(UInt8, '/navigation/vision/color', 10)
        # 创建已选择支路 ROS 发布器。
        self.route_pub = self.create_publisher(UInt8, '/navigation/route', 10)
        # 创建视觉 STOP ROS 发布器。
        self.stop_vision_pub = self.create_publisher(Bool, '/navigation/vision/stop_detected', 10)
        # 创建电控状态 ROS 发布器。
        self.motor_state_pub = self.create_publisher(UInt8, '/navigation/motor/state', 10)
        # 创建电机板循迹、里程和航向遥测发布器。
        self.motor_chassis_pub = self.create_publisher(String, '/navigation/motor/chassis_status', 10)
        # 创建电机板循迹位图发布器，遥测来自 PB12 到 PB15。
        self.line_bits_pub = self.create_publisher(UInt8, '/navigation/motor/line_bits', 10)
        # 创建导航阶段 ROS 发布器。
        self.navigation_state_pub = self.create_publisher(String, '/navigation/state', 10)
        # 创建赛前启动 Trigger 服务。
        self.start_service = self.create_service(Trigger, '/navigation/start', self.start_callback)
        # 创建受控暂停 Trigger 服务：下发零速并保持零速，不触发抓放。
        self.stop_service = self.create_service(Trigger, '/navigation/stop', self.stop_callback)
        # 创建恢复 Trigger 服务：从暂停阶段继续循迹。
        self.resume_service = self.create_service(Trigger, '/navigation/resume', self.resume_callback)
        # 创建 20 毫秒非阻塞通信轮询定时器。
        self.timer = self.create_timer(0.02, self.poll_links)
        # 记录电机协议模式错误，防止错误配置静默运行。
        if self.motor_profile != 'core_crc16':
            # 记录严重错误并锁定节点状态。
            self._fault('motor_profile must be core_crc16 for the supplied third-version firmware')
        # 记录视觉链路模式错误，防止无法建立视觉连接。
        if self.vision_transport not in ('udp', 'tcp', 'serial'):
            # 记录严重错误并锁定节点状态。
            self._fault('vision_transport must be udp, tcp, or serial')
        # 明确提示关闭编码器实测速度安全条件会禁止抓放动作。
        if not self.status_is_measured:
            # 保持安全锁：未确认实测速度时禁止启动抓放流程。
            self.get_logger().warning('motor_status_is_measured=false; servo motion remains locked after STOP')
        # 记录节点启动以及当前传输配置。
        self.get_logger().info(f'navigation node ready; motor_profile={self.motor_profile}, vision_transport={self.vision_transport}')

    # 处理赛前启动服务：确认链路和两路视觉角色在线后启动状态机。
    def start_callback(self, request, response):
        # 忽略空请求对象，因为 Trigger 不含字段。
        del request
        # 拒绝重复启动，避免比赛流程被重置。
        if self.started:
            # 返回当前阶段供调用者判断。
            response.success = False
            # 写入拒绝原因。
            response.message = f'already started: {self.state}'
            # 结束本次服务回调。
            return response
        # 检查电机串口是否打开。
        if self.motor_serial is None:
            # 告知调用方电机 MCU 链路尚未就绪。
            response.success = False
            # 写入可操作的提示。
            response.message = 'motor UART is not open'
            # 结束本次服务回调。
            return response
        # 检查舵机串口是否打开。
        if self.servo_serial is None:
            # 告知调用方舵机 MCU 链路尚未就绪。
            response.success = False
            # 写入可操作的提示。
            response.message = 'servo UART is not open'
            # 结束本次服务回调。
            return response
        # 必须先收到电机编码器、底盘遥测及舵机板状态，避免盲启动。
        if self.motor_status is None or self.motor_chassis_status is None or not self.servo_status_seen:
            response.success = False
            response.message = 'waiting for initial motor, motor-chassis and peripheral status frames'
            return response
        # 要求视觉表中定义的相机 1 和相机 2 角色均已握手。
        if self.vision_roles != {1, 2}:
            # 拒绝在视觉角色未确认时运行比赛流程。
            response.success = False
            # 提示当前缺失的视觉角色。
            response.message = f'waiting for vision roles 1 and 2; got {sorted(self.vision_roles)}'
            # 结束本次服务回调。
            return response
        # 要求电机状态速度确认为编码器实测值，避免目标速度零造成误判。
        if not self.status_is_measured:
            # 拒绝启动可能在停车确认处误动作的流程。
            response.success = False
            # 说明电控固件需先回报实测轮速。
            response.message = 'set motor_status_is_measured=true only after MCU status uses encoder-measured wheel speed'
            # 结束本次服务回调。
            return response
        # 锁存流程已启动标志。
        self.started = True
        # 进入由电机板 PID 执行的 S 弯循迹阶段。
        self.motor_line_mode_active = False
        self._dispatch('START', 'competition flow started; motor MCU owns four-sensor line PID')
        # 向服务调用者确认启动成功。
        response.success = True
        # 返回当前流程状态。
        response.message = 'started; waiting for route color and then STOP marker'
        # 返回服务结果。
        return response

    # 受控暂停：车辆立即零速，保持零速帧，不触发抓放序列，可恢复。
    def stop_callback(self, request, response):
        # Trigger 请求无字段。
        del request
        # 未启动时无需暂停，仅确认当前阶段。
        if not self.started:
            # 返回可解释的拒绝原因。
            response.success = False
            # 告知调用方流程尚未启动。
            response.message = f'not started: {self.state}'
            # 结束本次回调。
            return response
        # 已处于暂停状态时幂等返回，避免刷新暂停计时。
        if self.state == 'PAUSED':
            # 重复调用视为成功，便于上位机无脑重试。
            response.success = True
            # 返回暂停前的阶段。
            response.message = f'already paused from {self.paused_from_state}'
            # 结束本次回调。
            return response
        # 仅允许在自动行驶阶段暂停，避免打断停稳确认或机械臂动作。
        if self.state not in ('LINE_FOLLOW', 'ROUTE_FOLLOW'):
            # 拒绝在不安全阶段插入暂停。
            response.success = False
            # 说明当前阶段不允许暂停。
            response.message = f'cannot pause in state {self.state}; only LINE_FOLLOW/ROUTE_FOLLOW'
            # 结束本次回调。
            return response
        # 电机串口必须在线，否则无法下发零速。
        if self.motor_serial is None:
            # 拒绝在无法下发停车指令时进入暂停。
            response.success = False
            # 说明链路缺失。
            response.message = 'motor UART is not open'
            # 结束本次回调。
            return response
        # 记录暂停来源阶段，供恢复时回到同一控制器。
        self.paused_from_state = self.state
        # 立即下发左右轮零速，缩短车辆惯性滑行距离。
        if not self._send_wheel_speeds(0, 0):
            # 零速写入失败时不能声称已暂停。
            self._fault('failed to write zero-speed command when pausing')
            # 返回失败并说明已进入故障。
            response.success = False
            # 告知调用方需要排查串口。
            response.message = 'failed to send zero speed; entered FAULT'
            # 结束本次回调。
            return response
        # 进入暂停阶段；该阶段每周期仍刷新零速帧以免触发固件看门狗。
        self._dispatch('PAUSE', f'paused from {self.paused_from_state}; holding zero wheel speed')
        # 返回暂停成功与来源阶段。
        response.success = True
        # 提示可用 resume 继续。
        response.message = f'paused from {self.paused_from_state}; call /navigation/resume to continue'
        # 返回服务结果。
        return response

    # 从暂停恢复：回到暂停前的循迹阶段，无需重新等待视觉或 MCU 握手。
    def resume_callback(self, request, response):
        # Trigger 请求无字段。
        del request
        # 仅在暂停状态可恢复。
        if self.state != 'PAUSED':
            # 拒绝非暂停状态下的恢复请求。
            response.success = False
            # 说明当前阶段。
            response.message = f'not paused: {self.state}'
            # 结束本次回调。
            return response
        # 暂停来源未知时不允许猜测恢复目标，避免进入错误控制器。
        if self.paused_from_state not in ('LINE_FOLLOW', 'ROUTE_FOLLOW'):
            # 拒绝无法确定的恢复目标。
            response.success = False
            # 说明记录异常。
            response.message = f'unknown pause origin: {self.paused_from_state}'
            # 结束本次回调。
            return response
        # 恢复前要求视觉目标列表新鲜，防止在视觉已失效时继续高速行驶。
        now = time.monotonic()
        # 计算相机 1 目标列表最后接收时刻距今的间隔。
        last_target = self.vision_last_seen.get((1, VISION_TARGET_LIST), 0.0)
        # 目标列表缺失或过期时拒绝恢复。
        if now - last_target > self.vision_source_timeout:
            # 保持暂停状态，不放开行驶。
            response.success = False
            # 提示视觉链路尚未恢复。
            response.message = f'vision target list stale ({now - last_target:.2f}s); stay paused'
            # 结束本次回调。
            return response
        # 恢复前要求电机板循迹遥测仍然新鲜，避免 MCU 链路已失效时继续运行。
        if self.motor_chassis_status is None or now - self.line_status_time > 0.35:
            response.success = False
            response.message = 'motor chassis telemetry stale; stay paused'
            return response
        # 恢复前要求电机串口仍在线。
        if self.motor_serial is None:
            # 拒绝在链路断开时恢复。
            response.success = False
            # 提示串口缺失。
            response.message = 'motor UART is not open'
            # 结束本次回调。
            return response
        # 读取待恢复的目标阶段。
        target_state = self.paused_from_state
        # 恢复时重新下发 LINE 模式，避免暂停的 CMD_CHASSIS 留在手动模式。
        self.motor_line_mode_active = False
        # 回到暂停前阶段，由电机板本地 PID 继续循迹。
        self._dispatch('RESUME', 'resumed from PAUSED; restarting motor MCU local line PID')
        # 转移成功后清除暂停来源，避免旧值被后续流程误用。
        self.paused_from_state = None
        # 返回恢复成功。
        response.success = True
        # 告知恢复到的阶段。
        response.message = f'resumed to {target_state}'
        # 返回服务结果。
        return response

    # 定期尝试打开串口并轮询三条链路，不在回调中进行阻塞读取。
    def poll_links(self):
        # 记录当前单调时间，供超时判断使用。
        now = time.monotonic()
        # 当达到重连时间时尝试打开未连接的链路。
        if now >= self.next_reconnect:
            # 尝试打开电机和舵机 UART。
            self._open_serial_links()
            # 尝试打开视觉 UDP、连接 TCP 或打开视觉 UART。
            self._open_vision_link()
            # 下一次连接重试间隔设为 1 秒。
            self.next_reconnect = now + 1.0
        # 非阻塞读取电机 UART 并解析状态帧。
        self._poll_serial(self.motor_serial, self.motor_parser, self._handle_motor_frame)
        # 非阻塞读取舵机 UART 并解析状态帧。
        self._poll_serial(self.servo_serial, self.servo_parser, self._handle_servo_frame)
        # 轮询视觉 UDP 报文、TCP 字节流或视觉 UART 字节流。
        self._poll_vision_link()
        # 使用最新循迹位刷新底盘速度，满足电机板 300 ms 指令看门狗。
        self._update_line_follow(now)
        # 每次轮询都检查是否已等待电机停稳过久。
        self._check_stop_timeout(now)
        # 每次轮询都检查是否等待机械臂完成过久。
        self._check_arm_timeout(now)
        # 检查视觉数据链路超时并采用安全停车策略。
        self._check_vision_timeout(now)

    # 打开电机和舵机两路独立 UART 设备。
    def _open_serial_links(self):
        # 若电机串口未打开则尝试建立非阻塞串口连接。
        if self.motor_serial is None:
            # 保护串口打开过程的系统异常。
            try:
                # 打开电机 MCU 串口并将超时设为零以保持 ROS 轮询非阻塞。
                self.motor_serial = serial.Serial(self.motor_port, self.motor_baud, timeout=0, write_timeout=0.05)
                # 清空残留字节，避免读取上次运行留下的半帧。
                self.motor_serial.reset_input_buffer()
                # 记录电机串口已经连接。
                self.get_logger().info(f'motor UART connected: {self.motor_port}')
            # 捕获设备不存在、权限不足等串口异常。
            except (serial.SerialException, OSError) as exc:
                # 记录连接失败原因，稍后由重连逻辑重试。
                self.get_logger().warning(f'motor UART unavailable: {exc}')
        # 若舵机串口未打开则尝试建立非阻塞串口连接。
        if self.servo_serial is None:
            # 保护舵机串口打开过程的系统异常。
            try:
                # 打开舵机 MCU 串口并将超时设为零以保持 ROS 轮询非阻塞。
                self.servo_serial = serial.Serial(self.servo_port, self.servo_baud, timeout=0, write_timeout=0.05)
                # 清空舵机串口残留字节，避免旧帧误触发动作。
                self.servo_serial.reset_input_buffer()
                # 记录舵机串口已经连接。
                self.get_logger().info(f'servo UART connected: {self.servo_port}')
            # 捕获舵机设备不存在、权限不足等异常。
            except (serial.SerialException, OSError) as exc:
                # 记录连接失败原因，稍后由重连逻辑重试。
                self.get_logger().warning(f'servo UART unavailable: {exc}')

    # 按当前配置打开视觉 UDP 接收端、TCP 客户端或 USB-UART。
    def _open_vision_link(self):
        # UDP 模式绑定本机端口以接收视觉模块的一帧一包广播。
        if self.vision_transport == 'udp' and self.vision_udp_socket is None:
            # 创建 IPv4 UDP 接收套接字。
            candidate = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 保护 UDP 绑定过程并按通信模块配置监听端口 5000。
            try:
                # 允许节点重启后快速重新绑定相同 UDP 端口。
                candidate.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                # 显式允许在启用广播场景的系统上操作 UDP 广播套接字。
                candidate.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                # 绑定所有本机网卡地址，接收 Wi-Fi AP 广播报文。
                candidate.bind((self.vision_udp_bind_address, self.vision_udp_bind_port))
                # 切换为非阻塞模式，避免 UDP 接收等待影响 ROS 定时器。
                candidate.setblocking(False)
                # 保存已绑定的 UDP 套接字。
                self.vision_udp_socket = candidate
                # 丢弃上次连接中残留的半帧状态。
                self.vision_parser.reset()
                # 标记 UDP 接收端已就绪，实际帧超时仍由 watchdog 检查。
                self.vision_connected = True
                # 记录本机 UDP 监听地址和端口。
                self.get_logger().info(f'vision UDP listening: {self.vision_udp_bind_address}:{self.vision_udp_bind_port}')
            # 捕获端口被占用或绑定地址不可用等错误。
            except OSError as exc:
                # 关闭未能使用的 UDP 套接字。
                candidate.close()
                # 记录错误，定时器稍后会再次尝试绑定。
                self.get_logger().warning(f'vision UDP unavailable: {exc}')
        # TCP 模式下只在套接字未连接时进行连接尝试。
        if self.vision_transport == 'tcp' and self.vision_socket is None:
            # 创建 TCP 客户端套接字。
            candidate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 为连接操作设置有限超时，避免阻塞比赛状态机。
            candidate.settimeout(0.25)
            # 尝试连接庐山派 AP 地址和端口。
            try:
                # 按 xlsx 约定连接视觉主机。
                candidate.connect((self.vision_host, self.vision_tcp_port))
                # 成功连接后切换为非阻塞读写。
                candidate.setblocking(False)
                # 保存视觉 TCP 套接字句柄。
                self.vision_socket = candidate
                # 清除旧连接残留的半帧数据。
                self.vision_parser.reset()
                # 设置视觉链路为在线状态。
                self.vision_connected = True
                # 记录连接成功；视觉模块会由发送 API 自动周期上报版本和心跳。
                self.get_logger().info(f'vision TCP connected: {self.vision_host}:{self.vision_tcp_port}')
            # 捕获网络不可达、端口未监听等异常。
            except OSError as exc:
                # 关闭未使用的候选套接字。
                candidate.close()
                # 记录连接失败，定时器将在一秒后重试。
                self.get_logger().warning(f'vision TCP unavailable: {exc}')
        # 串口模式下只在视觉串口未打开时尝试连接。
        if self.vision_transport == 'serial' and self.vision_serial is None:
            # 保护视觉串口初始化过程。
            try:
                # 打开视觉端 USB-UART 桥，协议帧本身不作任何改动。
                self.vision_serial = serial.Serial(self.vision_port_device, self.vision_baud, timeout=0, write_timeout=0.05)
                # 清空串口残留字节，防止上次会话半帧污染。
                self.vision_serial.reset_input_buffer()
                # 标记视觉链路为在线。
                self.vision_connected = True
                # 记录串口链路打开成功；当前视觉模块按 API 调用自动上报版本。
                self.get_logger().info(f'vision UART connected: {self.vision_port_device}')
            # 捕获视觉串口设备异常。
            except (serial.SerialException, OSError) as exc:
                # 记录失败原因，后续定时重试。
                self.get_logger().warning(f'vision UART unavailable: {exc}')

    # 对串口做非阻塞读取，并把有效帧交给指定回调。
    def _poll_serial(self, link, parser, callback):
        # 设备未连接时无需读取。
        if link is None:
            # 结束本次轮询。
            return
        # 保护串口读操作以应对拔线或 USB 设备重置。
        try:
            # 最多读取 512 字节，完整处理粘包和半包。
            chunk = link.read(512)
            # 解析字节流中所有完整帧。
            for frame in parser.feed(chunk):
                # 将每一帧交给对应链路的状态处理函数。
                callback(frame)
        # 捕获设备拔出或串口 I/O 错误。
        except (serial.SerialException, OSError) as exc:
            # 记录链路故障。
            self.get_logger().error(f'UART read failed: {exc}')
            # 关闭失效串口并允许下一次定时重连。
            self._close_serial(link)

    # 轮询视觉链路并按 UDP 数据报或 TCP/UART 字节流解析视觉帧。
    def _poll_vision_link(self):
        # UDP 模式下一数据报对应通信模块发送的一帧。
        if self.vision_transport == 'udp' and self.vision_udp_socket is not None:
            # 每次定时器最多读取 64 个数据报，避免网络突发占满回调。
            for _ in range(64):
                # 接收视觉帧和发送方地址。
                try:
                    # 一次读取完整 UDP 数据报及其来源地址。
                    packet, peer = self.vision_udp_socket.recvfrom(2048)
                    # 缓存最新视觉发送方，便于诊断单播/广播来源。
                    self.vision_udp_peer = peer
                    # 将数据报交给协议解析器检查帧头、长度和 SUM8。
                    frames = self.vision_parser.feed(packet)
                    # 当前通信模块保证一包一帧，拒绝不完整或未通过校验的数据报。
                    if not frames or self.vision_parser.buffer:
                        # 清除错误数据报留下的半帧，避免与下一包错误拼接。
                        self.vision_parser.reset()
                        # 记录无效 UDP 数据报及解析器累计错误数。
                        self.get_logger().warning(f'ignored invalid vision UDP datagram from {peer}; parser_errors={self.vision_parser.errors}')
                        # 继续检查队列中的下一份 UDP 数据报。
                        continue
                    # 逐帧更新来源角色、ROS 话题和导航流程状态。
                    for frame in frames:
                        # 将有效协议帧交给统一视觉处理器。
                        self._handle_vision_frame(frame)
                # 当前没有更多 UDP 报文时结束本轮非阻塞轮询。
                except BlockingIOError:
                    # 返回主 ROS 定时器继续处理其他链路。
                    break
                # UDP 套接字异常时关闭链路并等待自动重建。
                except OSError as exc:
                    # 记录接收故障和错误原因。
                    self.get_logger().warning(f'vision UDP read failed: {exc}')
                    # 关闭 UDP 套接字并清理视觉会话。
                    self._close_vision_link()
                    # 结束本次 UDP 轮询。
                    break
        # TCP 模式仅在套接字有效时读取。
        if self.vision_transport == 'tcp' and self.vision_socket is not None:
            # 非阻塞读取连接中的视觉数据。
            try:
                # 一次接收最多 2048 字节，允许一个 TCP 包包含多帧。
                chunk = self.vision_socket.recv(2048)
                # 空数据表示 TCP 对端关闭连接。
                if not chunk:
                    # 关闭套接字并等待自动重连。
                    self._close_vision_link()
                    # 结束本轮视觉读取。
                    return
                # 将字节块解析为视觉完整帧。
                for frame in self.vision_parser.feed(chunk):
                    # 逐帧更新视觉角色、ROS 话题和比赛状态机。
                    self._handle_vision_frame(frame)
            # 非阻塞 TCP 暂无数据不是错误。
            except BlockingIOError:
                # 保持连接并返回主轮询。
                pass
            # 连接关闭或网络 I/O 异常时触发重连。
            except OSError as exc:
                # 记录异常原因。
                self.get_logger().warning(f'vision TCP read failed: {exc}')
                # 关闭失效链路并重置解析器。
                self._close_vision_link()
        # 串口模式下把视觉 USB-UART 也作为非阻塞字节流解析。
        if self.vision_transport == 'serial' and self.vision_serial is not None:
            # 保护视觉串口读取过程。
            try:
                # 读取已到达的字节。
                chunk = self.vision_serial.read(2048)
                # 解析粘包、半包和连续视觉帧。
                for frame in self.vision_parser.feed(chunk):
                    # 逐帧更新视觉状态。
                    self._handle_vision_frame(frame)
            # 捕获 USB-UART 设备拔出和串口异常。
            except (serial.SerialException, OSError) as exc:
                # 记录视觉串口故障。
                self.get_logger().warning(f'vision UART read failed: {exc}')
                # 关闭失效串口并等待自动重连。
                self._close_serial(self.vision_serial)
                # 标记视觉链路断开。
                self.vision_serial = None
                # 清除半帧并清空角色，断线后必须重新握手。
                self._reset_vision_session()

    # 处理电机 MCU 发回的 CRC16 状态帧。
    def _handle_motor_frame(self, frame):
        # 解包命令字和数据区。
        cmd, data = frame
        # 0x11 是新版电机板循迹遥测和事件帧。
        if cmd == CMD_CHASSIS_STATUS:
            self._handle_chassis_status(data)
            return
        # 忽略电机 MCU 的非状态帧。
        if cmd != CMD_STATUS:
            # 结束非状态帧处理。
            return
        # 解析左右轮速度和电控状态字节。
        status = parse_status(data)
        # 丢弃状态数据长度错误的帧。
        if status is None:
            # 记录协议错误供排查。
            self.get_logger().warning('ignored motor status with invalid payload length')
            # 结束无效帧处理。
            return
        # 更新最新左右轮速度与状态值。
        self.motor_status = status
        # 将电控状态发布到 ROS。
        state_msg = UInt8()
        # 填入状态字节。
        state_msg.data = status[2]
        # 发布电控状态话题。
        self.motor_state_pub.publish(state_msg)
        # 只在等待停稳阶段统计连续编码器零速状态。
        if self.state == 'WAIT_MOTOR_STOP':
            # 检查状态为 IDLE 且左右编码器实测速度低于阈值。
            stopped = status[2] == STATE_IDLE and abs(status[0]) <= self.stopped_speed_threshold and abs(status[1]) <= self.stopped_speed_threshold
            # 满足停稳条件且速度字段确认来自编码器时累加样本。
            if stopped and self.status_is_measured:
                # 首个零速帧记录稳定计时起点。
                if self.stopped_since is None:
                    # 保存连续停稳计时起点。
                    self.stopped_since = time.monotonic()
                # 增加连续有效停稳帧计数。
                self.stopped_samples += 1
                # 检查采样次数与持续时间是否都满足。
                if self.stopped_samples >= self.stopped_samples_required and time.monotonic() - self.stopped_since >= self.stopped_dwell_seconds:
                    # 确认车辆已停稳后才启动机械臂序列。
                    self._begin_arm_sequence()
            # 任意一帧仍在运动或状态不为空闲时重置连续计数。
            else:
                # 清零停稳样本，要求之后重新连续满足条件。
                self.stopped_samples = 0
                # 重置稳定计时起点。
                self.stopped_since = None
        # 故障或指令超时状态时立即停止状态机并锁定舵机流程。
        if status[2] in (STATE_FAULT, STATE_TIMEOUT) and self.started:
            # 进入故障并向电机端发送紧急停止。
            self._fault(f'motor controller reported state={status[2]}')
        # 即使 0x11 事件短暂丢失，状态 6 仍说明电机已在路口停车。
        if status[2] == STATE_JUNCTION and self.started and self.state in ('LINE_FOLLOW', 'ROUTE_FOLLOW'):
            self.motor_line_mode_active = False
            self._dispatch('JUNCTION_DETECTED', f'motor stopped at junction; selected route={self.selected_route}; awaiting route-select support')
        # 电机板丢线后已经停车，导航不自动重新启动底盘。
        if status[2] == STATE_LINE_LOST and self.started:
            self.motor_line_mode_active = False
            self._fault('motor controller reported LINE_LOST')

    # 处理新版电机板 0x11 循迹遥测、里程和一次性事件。
    def _handle_chassis_status(self, data):
        # 解码传感器位、PID 误差/修正、里程、航向和事件。
        chassis = parse_chassis_status(data)
        # 长度不符合固件格式时丢弃状态帧。
        if chassis is None:
            self.get_logger().warning('ignored motor chassis status with invalid payload length')
            return
        # 保存电机板的本地循迹数据及最后更新时间。
        self.motor_chassis_status = chassis
        self.line_bits = chassis['line_bits']
        self.line_status_time = time.monotonic()
        self.last_motor_event = chassis['event']
        # 发布电机板循迹位图供调试观察。
        line_msg = UInt8()
        line_msg.data = self.line_bits
        self.line_bits_pub.publish(line_msg)
        # 发布完整遥测，方便检查 PID 误差、里程和航向。
        chassis_msg = String()
        chassis_msg.data = json.dumps(chassis, ensure_ascii=False)
        self.motor_chassis_pub.publish(chassis_msg)
        # 固件报告路口后车辆已刹停；此版本没有红黄蓝分支选择命令。
        if chassis['event'] & EVENT_JUNCTION and self.started and self.state in ('LINE_FOLLOW', 'ROUTE_FOLLOW'):
            self.motor_line_mode_active = False
            self._dispatch('JUNCTION_DETECTED', f'junction detected; selected route={self.selected_route}; awaiting route-select support')
            self.get_logger().warning('motor LINE mode has no red/yellow/blue branch selector; keeping the vehicle stopped at junction')
        # 丢线事件由电机固件先停车，导航进入故障状态并锁定动作。
        if chassis['event'] & EVENT_LINE_LOST and self.started:
            self.motor_line_mode_active = False
            self._fault('motor firmware reported line-lost event')

    # 处理外设 MCU 发回的舵机状态和循迹状态 CRC16 帧。
    def _handle_servo_frame(self, frame):
        # 解包舵机链路上的命令和数据。
        cmd, data = frame
        # 忽略不是状态反馈的舵机 MCU 帧。
        if cmd != CMD_STATUS:
            # 结束非状态帧处理。
            return
        # 解析三个舵机角度、机械臂状态和外设遥测位。
        status = parse_peripheral_status(data)
        # 丢弃长度非法的反馈帧。
        if status is None:
            # 记录协议错误。
            self.get_logger().warning('ignored peripheral status with invalid payload length')
            # 结束无效帧处理。
            return
        # 标记舵机外设板在线；循迹位图只由电机板 0x11 更新。
        self.servo_status_seen = True
        # 机械臂动作阶段只接受 BUSY 后的 DONE，防止旧 DONE 帧被误当作新动作完成。
        if self.state == 'ARM_ACTION' and status[3] == STATE_ARM_BUSY:
            # 记录当前命令已经被机械臂 MCU 接收并开始执行。
            self.arm_saw_busy = True
        # 收到有效 BUSY->DONE 转换后执行下一步动作。
        if self.state == 'ARM_ACTION' and self.arm_saw_busy and status[3] == STATE_ARM_DONE:
            # 推进抓取、放置、复位序列。
            self._advance_arm_sequence()
        # 舵机 MCU 故障时立即进入安全故障状态。
        if status[3] == STATE_FAULT and self.started:
            # 锁定流程并紧急停止电机。
            self._fault(f'peripheral controller reported state={status[3]}')

    # 处理视觉帧并按协议源编号区分相机 1 与相机 2。
    def _handle_vision_frame(self, frame):
        # 解包视觉帧来源、命令、序号和数据。
        src, cmd, seq, data = frame
        # 忽略无效来源编号，协议只允许相机 1 或相机 2。
        if src not in (1, 2):
            # 记录错误来源。
            self.get_logger().warning(f'ignored vision frame with invalid SRC={src}')
            # 结束无效来源处理。
            return
        # 记录本路本命令最近收到数据的时刻。
        self.vision_last_seen[(src, cmd)] = time.monotonic()
        # 记录最近任何视觉有效帧时间。
        self.vision_last_any = time.monotonic()
        # 接收版本与角色帧并校验角色和来源一致。
        if cmd == VISION_VERSION and len(data) == 9:
            # 读取数据区末尾角色编号。
            role = data[8]
            # 只有角色与 SRC 一致时才确认视觉来源。
            if role == src:
                # 将该相机角色加入握手集合。
                self.vision_roles.add(role)
                # 按通信模块布局记录固件版本和协议版本。
                self.get_logger().info(f'vision role {role} online; firmware={data[2]}.{data[3]}, protocol={data[0]}.{data[1]}')
                # 两路视觉角色上线后不发送此通信模块不会处理的启动命令。
                if self.vision_roles == {1, 2} and not self.vision_enabled:
                    # 视觉组由识别循环逐帧调用 API，导航端只接收主动推送的数据。
                    self.vision_enabled = True
                    # 记录双来源版本握手完成。
                    self.get_logger().info('both vision roles online; waiting for API-pushed target frames')
            # 角色字段与来源字段不一致时丢弃且不握手。
            else:
                # 记录视觉端角色配置错误。
                self.get_logger().error(f'vision SRC={src} advertises role={role}')
            # 版本帧处理结束。
            return
        # 仅相机 1 按协议职责提供颜色与 STOP 识别。
        if src == 1 and cmd == VISION_TARGET_LIST:
            # 解析颜色列表和 STOP 位域。
            target_list = parse_target_list(data)
            # 不让格式错误的目标列表影响机器人动作。
            if target_list is None:
                # 记录视觉数据格式异常。
                self.get_logger().warning('ignored invalid target-list payload')
                # 结束当前帧处理。
                return
            # 发布完整目标列表供 ROS 其他节点查看。
            list_msg = String()
            # 使用 JSON 表示颜色、距离、置信度和有效性。
            list_msg.data = json.dumps(target_list, ensure_ascii=False)
            # 发布视觉目标列表。
            self.target_list_pub.publish(list_msg)
            # 发布 STOP 标志检测状态。
            stop_msg = Bool()
            # 从协议 bit2 取 STOP 标记。
            stop_msg.data = target_list['stop_detected']
            # 发布 STOP 识别话题。
            self.stop_vision_pub.publish(stop_msg)
            # 发布检测到的第一个有效红黄蓝色卡代码。
            color = self._select_route_color(target_list)
            # 只有本轮尚未选择路线时才处理色卡。
            if color is not None:
                # 发布统一颜色编码，1 红、2 黄、3 蓝。
                color_msg = UInt8()
                # 填入色卡颜色代码。
                color_msg.data = color
                # 发布颜色结果话题。
                self.color_pub.publish(color_msg)
                # 仅在主流程已启动且仍在循迹或路线阶段时选择路线。
                if self.started and self.state in ('LINE_FOLLOW', 'ROUTE_FOLLOW', 'WAIT_ROUTE_MANEUVER'):
                    # 将颜色代码映射为电控约定的路线代码。
                    self._select_route(color)
            # STOP 标志有效时立即发停车指令并等待电机实测速度归零。
            if target_list['stop_detected'] and self.started and self.state in ('LINE_FOLLOW', 'ROUTE_FOLLOW', 'WAIT_ROUTE_MANEUVER'):
                # 命令底盘停车并开始零速反馈确认。
                self._request_motor_stop()
            # 目标列表帧处理结束。
            return
        # 相机 2/或相机 1 的坐标数据统一发布，但不做无依据的坐标变换。
        if cmd == VISION_TARGET_COORD:
            # 解析相机已经转换到机器人坐标系的目标数据。
            coordinate = parse_target_coord(data)
            # 丢弃无效长度坐标帧。
            if coordinate is None:
                # 记录视觉坐标数据长度错误。
                self.get_logger().warning('ignored invalid target-coordinate payload')
                # 结束无效坐标处理。
                return
            # 创建 ROS 字符串消息发布数据。
            coord_msg = String()
            # 增加 SRC 和 SEQ 以区分相机来源及消息先后。
            coordinate['src'] = src
            # 将当前视觉序号放入诊断消息。
            coordinate['seq'] = seq
            # 使用 JSON 发布坐标、偏航、置信度和状态位。
            coord_msg.data = json.dumps(coordinate, ensure_ascii=False)
            # 发布视觉坐标话题。
            self.target_coord_pub.publish(coord_msg)
            # 坐标帧处理结束。
            return
        # 视觉心跳只更新链路时间，不将其误认为目标识别数据。
        if cmd == VISION_HEARTBEAT:
            # 校验并解析 8 字节自动心跳负载。
            heartbeat = parse_vision_heartbeat(data)
            # 丢弃长度不符合协议的心跳帧。
            if heartbeat is None:
                # 记录视觉心跳格式错误。
                self.get_logger().warning(f'ignored invalid heartbeat payload from SRC={src}')
                # 结束当前帧处理。
                return
            # 增加来源编号以区分视觉处理角色。
            heartbeat['src'] = src
            # 创建视觉心跳 ROS 消息。
            heartbeat_msg = String()
            # 将状态、帧率、错误数和处理耗时编码为 JSON。
            heartbeat_msg.data = json.dumps(heartbeat, ensure_ascii=False)
            # 发布心跳诊断信息。
            self.vision_heartbeat_pub.publish(heartbeat_msg)
            # 心跳处理结束。
            return
        # 解析视觉通信模块发送的标准错误帧。
        if cmd == VISION_ERROR:
            # 按 <BBH 布局解码错误码、细节码和计数。
            error = parse_vision_error(data)
            # 长度非法时打印原始负载帮助排查协议不匹配。
            if error is None:
                # 输出无法解析的错误数据。
                self.get_logger().error(f'vision SRC={src} invalid error payload={data.hex()}')
            # 有效错误帧使用具名字段记录。
            else:
                # 输出错误来源、代码、细节及累计计数。
                self.get_logger().error(f'vision SRC={src} error={error["err_code"]} detail={error["detail"]} counter={error["counter"]}')

    # 根据相机 1 有效目标列表选出红、黄、蓝路线色卡。
    def _select_route_color(self, target_list):
        # 颜色无效时不猜测路线。
        if not target_list['color_valid']:
            # 返回无路线结果。
            return None
        # 逐个检查协议中的有效目标。
        for target in target_list['targets']:
            # 只接受规则规定的红、黄、蓝三种色卡。
            if target['color'] in (1, 2, 3):
                # 返回路线代码，映射保持 1 红、2 黄、3 蓝。
                return target['color']
        # 没有可用颜色时不选择路线。
        return None

    # 保存视觉识别出的红/黄/蓝目标路线。
    def _select_route(self, color):
        # 如果此前已进入路线跟随，不允许重复切换到新支路。
        if self.state == 'ROUTE_FOLLOW':
            # 忽略抖动导致的重复色卡结果。
            return
        # 路线代码与视觉颜色编码相同：红左 1、黄中 2、蓝右 3。
        route_code = color
        # 保存路线色码，供路口分支控制使用。
        self.selected_route = route_code
        # 发布路线选择结果用于 RViz 外的 ROS 联调监视。
        route_msg = UInt8()
        # 设置发布的路线代码。
        route_msg.data = route_code
        # 发布路线结果。
        self.route_pub.publish(route_msg)
        # 视觉结果只记录路线；当前固件的 LINE 命令没有红黄蓝分支选择参数。
        if self.state == 'WAIT_ROUTE_MANEUVER':
            self.get_logger().warning(f'route color={color} recorded, but motor firmware has no route-select command; keeping the vehicle stopped')
            return
        # 进入选线阶段；路径本身仍由电机板的本地循迹 PID 跟踪。
        self._dispatch('ROUTE_SELECTED', f'route color={color} selected; motor MCU continues local line PID')

    # 启用电机板本地循迹并周期发送心跳，导航端不再计算循迹轮速。
    def _update_line_follow(self, now):
        # 暂停期间必须持续下发零速，否则电机固件 300 ms 看门狗会停车并报 CMD_TIMEOUT。
        if self.state == 'PAUSED':
            # 刷新零速帧以喂狗并保持车辆静止。
            if not self._send_wheel_speeds(0, 0):
                # 零速帧写入失败时进入故障，避免车辆在未知状态下失控。
                self._fault('failed to refresh zero-speed command while paused')
            # 零速 CMD_CHASSIS 会令电机固件退出本地 LINE 模式。
            self.motor_line_mode_active = False
            # 暂停阶段不执行循迹计算。
            return
        # 仅在赛程循迹状态下驱动电机板循迹模式。
        if not self.started or self.state not in ('LINE_FOLLOW', 'ROUTE_FOLLOW'):
            return
        # 遥测超过 350 ms 未更新时停止导航流程，防止电机板/串口离线后继续比赛。
        if self.motor_chassis_status is None or now - self.line_status_time > 0.35:
            self._fault('motor chassis telemetry missing or stale')
            return
        # 进入循迹阶段时只发一次 0x07，参数是基础速度和最大转向修正量。
        if not self.motor_line_mode_active:
            payload = struct.pack('<hh', self.motor_line_base_speed, self.motor_line_turn_limit)
            if not self._write_serial(self.motor_serial, build_electrical_frame(CMD_LINE, payload), 'motor local line-follow mode'):
                self._fault('failed to enable motor MCU local line-follow mode')
                return
            self.motor_line_mode_active = True
            self.last_motor_ping = 0.0
        # 每 100 ms 发一次 0x00 PING，避免电机固件 300 ms 看门狗停车。
        if now - self.last_motor_ping >= 0.1:
            if not self._write_serial(self.motor_serial, build_electrical_frame(CMD_PING), 'motor watchdog heartbeat'):
                self._fault('failed to refresh motor MCU heartbeat')
                return
            self.last_motor_ping = now

    # 发送第三版电机固件支持的 CMD_CHASSIS 左右轮速度帧。
    def _send_wheel_speeds(self, left, right):
        # 电机 UART 断开时不能安全控制底盘。
        if self.motor_serial is None:
            self._fault('motor UART disconnected while updating wheel speeds')
            return False
        # 将轮速裁剪到有符号 16 位范围并按小端打包为四字节负载。
        payload = struct.pack('<hh', max(-32768, min(32767, int(left))), max(-32768, min(32767, int(right))))
        # 组装 CMD_CHASSIS=0x01 CRC16 帧并写入电机 UART。
        ok = self._write_serial(self.motor_serial, build_electrical_frame(CMD_CHASSIS, payload), 'motor wheel speeds')
        # 缓存最后一次成功发送的目标轮速。
        if ok:
            self.last_wheel_command = (int(left), int(right))
            # CMD_CHASSIS 切换为手动速度模式，恢复时必须重新发 CMD_LINE。
            self.motor_line_mode_active = False
        return ok

    # 停车请求：先向电机 MCU 发送 STOP，再等待编码器速度持续为零。
    def _request_motor_stop(self):
        # 如果已经在等待停稳，避免重复停车命令刷新超时计时。
        if self.state == 'WAIT_MOTOR_STOP':
            # 结束重复停车请求。
            return
        # 要求电机 UART 已打开。
        if self.motor_serial is None:
            # 链路断开时不允许继续机械臂流程。
            self._fault('motor UART disconnected before stop request')
            # 结束停车过程。
            return
        # 第三版电机协议没有路线 STOP 码，左右目标速度置零即为停车指令。
        if not self._send_wheel_speeds(0, 0):
            # 零速帧写入失败时不能开始等待停稳状态。
            self._fault('failed to write zero-speed command to motor MCU')
            return
        # 清零已累积停稳样本。
        self.stopped_samples = 0
        # 清除首次停稳时刻。
        self.stopped_since = None
        # 保存本次停车请求开始时间。
        self.stop_wait_started = time.monotonic()
        # 切换到等待实测零速状态。
        self._dispatch('STOP_MARKER', 'STOP command sent; waiting for repeated encoder-zero status frames')

    # 电机停稳条件满足后才开始机械臂抓取/放置/复位序列。
    def _begin_arm_sequence(self):
        # 要求舵机 UART 当前仍然可用。
        if self.servo_serial is None:
            # 串口断开时进入故障，绝不假设机械臂已收到命令。
            self._fault('servo UART disconnected after vehicle stop')
            # 结束序列启动。
            return
        # 初始化动作序列索引为抓取动作。
        self.arm_action_index = 0
        # 发布机械臂执行阶段状态。
        if not self._dispatch('MOTOR_ZERO', 'vehicle stopped; starting grab, place, reset sequence'):
            return
        # 发送第一条机械臂动作命令。
        self._send_current_arm_action()

    # 将当前抓放序列动作发送至独立舵机 MCU。
    def _send_current_arm_action(self):
        # 动作序列全部完成后标记整个任务完成。
        if self.arm_action_index >= len(self.arm_actions):
            # 切换到完成状态。
            self._dispatch('ARM_SEQUENCE_DONE', 'grab, place and reset actions completed')
            # 不再发送动作。
            return
        # 读取当前要执行的动作代码。
        action = self.arm_actions[self.arm_action_index]
        # 确认舵机串口仍然在线。
        if self.servo_serial is None:
            # 链路消失时进入故障状态。
            self._fault('servo UART disconnected during arm sequence')
            # 结束当前动作发送。
            return
        # 重置动作 BUSY 确认标志，避免上一动作的 DONE 状态被复用。
        self.arm_saw_busy = False
        # 保存动作开始时刻用于超时检测。
        self.arm_action_started = time.monotonic()
        # 构造机械臂动作命令帧，数据区单字节为动作编号。
        frame = build_electrical_frame(CMD_ARM, bytes((action,)))
        # 写入独立舵机串口。
        self._write_serial(self.servo_serial, frame, f'servo action {action}')
        # 记录抓取、放置或复位动作编号。
        self.get_logger().info(f'sent servo MCU arm action={action}')

    # 在收到 BUSY 后收到 DONE 时推进机械臂序列。
    def _advance_arm_sequence(self):
        # 增加当前动作序列索引。
        self.arm_action_index += 1
        # 发送下一条抓取/放置/复位命令。
        self._send_current_arm_action()

    # 检查电机停稳确认的总等待超时。
    def _check_stop_timeout(self, now):
        # 仅在等待电机停稳时检查超时。
        if self.state != 'WAIT_MOTOR_STOP':
            # 其他状态不进行该超时处理。
            return
        # 超过设定时限仍未获得实测零速反馈时安全失败。
        if now - self.stop_wait_started > self.motor_stop_timeout:
            # 不启动舵机，记录电机未停稳或状态反馈缺失。
            self._fault('timeout waiting for measured motor zero-speed status; servo sequence was not started')

    # 检查机械臂 MCU 是否在规定时间内反馈动作完成。
    def _check_arm_timeout(self, now):
        # 仅当机械臂动作处于等待中时检查。
        if self.state != 'ARM_ACTION':
            # 其他状态无需机械臂动作计时。
            return
        # 超过动作完成时限且仍未收到 BUSY->DONE 时进入故障状态。
        if now - self.arm_action_started > self.arm_timeout:
            # 说明机械臂 MCU 必须实现 ARM_BUSY 与 ARM_DONE 状态反馈。
            self._fault(f'timeout waiting for arm action {self.arm_actions[self.arm_action_index]} completion')

    # 视觉断线或相机 1 数据超时时，驱动 MCU 停车并禁止机械臂继续动作。
    def _check_vision_timeout(self, now):
        # 未启动流程时不因无视觉数据而触发比赛状态故障。
        if not self.started or self.state not in ('LINE_FOLLOW', 'ROUTE_FOLLOW'):
            # 仅在车辆循迹和分支循迹阶段监视视觉数据。
            return
        # 读取相机 1 的目标列表最新时间。
        last_target = self.vision_last_seen.get((1, VISION_TARGET_LIST), 0.0)
        # 计算整条视觉链路最后有效帧时间是否过期。
        link_stale = now - self.vision_last_any > self.vision_link_timeout
        # 计算相机 1 目标列表是否超过 300 毫秒没有更新。
        source_stale = now - last_target > self.vision_source_timeout
        # 任意相关数据流超时后停车并锁定流程。
        if link_stale or source_stale:
            # 先尽力发 STOP，再切换故障状态并锁定后续舵机动作。
            self._fault('vision data timeout during active route following')

    # 发送电机紧急停止并切换到故障状态。
    def _fault(self, reason):
        # 记录故障原因并发布状态。
        self._dispatch('FAULT', reason)
        # 电机 UART 在线时发送第三版固件支持的 CRC16 急停帧。
        if self.motor_serial is not None:
            # 尝试发送 MCU 定义的急停命令。
            self._write_serial(self.motor_serial, build_electrical_frame(CMD_ESTOP), 'motor emergency stop')
        # 外设 UART 在线时取消尚未完成的舵机动作。
        if self.servo_serial is not None:
            self._write_serial(self.servo_serial, build_electrical_frame(CMD_ESTOP), 'peripheral emergency stop')
        # 将流程锁定为已启动，拒绝再次从头运行，需人工排查后重启节点。
        self.started = True

    # 按状态与事件查转移表，并执行统一的状态进入/退出处理。
    def _dispatch(self, event, detail):
        # 优先查找当前状态专属转移。
        specific_key = (self.state, event)
        # 精确判断键是否存在，不能用 or，因为 RESUME 的 None 是有意义的目标。
        if specific_key in TRANSITIONS:
            # 读取当前状态对应目标。
            target = TRANSITIONS[specific_key]
        # 未定义专属转移时再尝试全局通配转移。
        elif ('*', event) in TRANSITIONS:
            # 读取通配事件的目标状态。
            target = TRANSITIONS[('*', event)]
        # 未定义转移时拒绝该事件并记录，防止非法跳转被静默接受。
        else:
            # 写入调试日志便于发现遗漏的事件边。
            self.get_logger().warning(f'ignored event={event} in state={self.state}; no transition is defined')
            # 返回失败，不改变当前状态。
            return False
        # None 专用于恢复事件，回到暂停前保存的循迹状态。
        if target is None and event == 'RESUME':
            # 读取暂停前阶段作为恢复目标。
            target = self.paused_from_state
        # 拒绝缺失或不合法的恢复目标。
        if target is None:
            # 记录转移配置或暂停来源错误。
            self.get_logger().error(f'event={event} has no valid target from state={self.state}')
            # 保持现状，不执行危险的猜测跳转。
            return False
        # 执行状态退出动作，清理与旧状态相关的运行标志。
        self._on_exit(self.state, target)
        # 保存旧状态供诊断和进入动作使用。
        previous_state = self.state
        # 更新内部状态并发布统一状态消息。
        self._set_state(target, detail)
        # 执行新状态的进入动作。
        self._on_enter(target, previous_state)
        # 报告转移成功。
        return True

    # 新状态进入动作：只更新状态相关标志，串口命令仍由安全流程负责发送。
    def _on_enter(self, state, previous_state):
        # 进入任一循迹状态时，要求下一轮重新使能电机板 LINE 模式。
        if state in ('LINE_FOLLOW', 'ROUTE_FOLLOW'):
            # 让 _update_line_follow 在下一轮发送 CMD_LINE。
            self.motor_line_mode_active = False
        # 进入暂停或停车等待状态时，标记本地循迹模式未保持活动。
        elif state in ('PAUSED', 'WAIT_MOTOR_STOP', 'WAIT_ROUTE_MANEUVER', 'FAULT', 'COMPLETE'):
            # 后续恢复时必须显式重新使能循迹模式。
            self.motor_line_mode_active = False
        # 路口事件可能先于 MCU 完成制动，因此导航端也下发零速保持安全。
        if state == 'WAIT_ROUTE_MANEUVER':
            # 无电机链路时立即锁定故障状态。
            if self.motor_serial is None:
                self._fault('motor UART disconnected while waiting at junction')
            # 路口事件可能早于制动完成，确保零速命令写入。
            elif not self._send_wheel_speeds(0, 0):
                # 写入失败时进入故障并发送电机急停。
                self._fault('failed to send zero speed while waiting at junction')
        # 记录新状态进入动作，便于调试转移顺序。
        self.get_logger().debug(f'on_enter: {previous_state} -> {state}')

    # 旧状态退出动作：目前清除无需跨状态保留的瞬时控制标记。
    def _on_exit(self, state, target_state):
        # 离开循迹状态后不再认为电机板正处于可持续循迹模式。
        if state in ('LINE_FOLLOW', 'ROUTE_FOLLOW') and target_state not in ('LINE_FOLLOW', 'ROUTE_FOLLOW'):
            # 实际停车命令由暂停、STOP 标志或故障处理先行发送。
            self.motor_line_mode_active = False
        # 记录旧状态退出动作。
        self.get_logger().debug(f'on_exit: {state} -> {target_state}')

    # 更新内部状态并发布包含状态名和说明的 ROS String 消息。
    def _set_state(self, state, detail):
        # 保存最新状态名称。
        self.state = state
        # 创建 ROS 字符串消息。
        message = String()
        # 编码状态名、说明与停稳帧数便于 ROS 端监控。
        message.data = json.dumps({'state': state, 'detail': detail, 'stopped_samples': self.stopped_samples}, ensure_ascii=False)
        # 发布导航状态。
        self.navigation_state_pub.publish(message)
        # 将每次状态切换写入日志。
        self.get_logger().info(f'navigation state={state}: {detail}')

    # 将数据写入串口并记录错误。
    def _write_serial(self, link, data, label):
        # 串口句柄不存在时返回失败。
        if link is None:
            # 报告链路未打开。
            return False
        # 捕获串口拔出和写超时异常。
        try:
            # 写入完整视觉或电控协议帧。
            written = link.write(data)
            # 验证 PySerial 是否写入全部字节。
            if written != len(data):
                # 记录短写错误并返回失败。
                self.get_logger().error(f'{label}: partial UART write {written}/{len(data)}')
                # 报告写入不完整。
                return False
            # 报告写入成功。
            return True
        # 捕获设备拔出、写入超时和驱动错误。
        except (serial.SerialException, OSError) as exc:
            # 记录串口写错误。
            self.get_logger().error(f'{label}: UART write failed: {exc}')
            # 关闭对应无效串口句柄。
            self._close_serial(link)
            # 报告写入失败。
            return False

    # 清理指定串口句柄并更新节点字段。
    def _close_serial(self, link):
        # 尝试关闭已打开的串口。
        try:
            # PySerial close 在重复调用时也是安全的。
            link.close()
        # 忽略清理阶段的底层 I/O 错误。
        except (serial.SerialException, OSError):
            # 清理失败不应阻止节点安全退出。
            pass
        # 清空与关闭句柄相同的节点字段。
        if link is self.motor_serial:
            # 电机链路失联后清空句柄并复位解析器。
            self.motor_serial = None
            # 丢弃旧电机半帧。
            self.motor_parser.reset()
            # 活动流程中电机 UART 失联立即故障。
            if self.started:
                # 进入安全故障并尽力停止电机。
                self._fault('motor UART disconnected')
        # 清空舵机串口字段。
        if link is self.servo_serial:
            # 舵机 MCU 失联后清空句柄。
            self.servo_serial = None
            # 丢弃旧舵机状态半帧。
            self.servo_parser.reset()
            # 活动流程中舵机 UART 失联立即进入故障。
            if self.started:
                # 停止后续机械臂动作。
                self._fault('servo UART disconnected')

    # 关闭视觉链路并清除解析器和角色握手。
    def _close_vision_link(self):
        # 关闭 UDP 监听套接字（若存在）。
        if self.vision_udp_socket is not None:
            # 捕获套接字释放期间可能出现的系统错误。
            try:
                # 释放 UDP 监听端口。
                self.vision_udp_socket.close()
            # 清理失败不阻止其他链路释放。
            except OSError:
                # 忽略 UDP 关闭错误并继续清理。
                pass
            # 清空 UDP 套接字句柄。
            self.vision_udp_socket = None
            # 清空最后一个发送方地址。
            self.vision_udp_peer = None
        # 关闭 TCP 套接字（若存在）。
        if self.vision_socket is not None:
            # 捕获已经断开的 socket 清理错误。
            try:
                # 释放 TCP 套接字资源。
                self.vision_socket.close()
            # 忽略清理阶段的 socket 错误。
            except OSError:
                # 继续重置视觉会话。
                pass
            # 清空 TCP 套接字句柄。
            self.vision_socket = None
        # 关闭视觉串口句柄（若存在）。
        if self.vision_serial is not None:
            # 复用统一串口清理函数。
            self._close_serial(self.vision_serial)
            # 视觉串口字段置空。
            self.vision_serial = None
        # 清除视觉会话状态及旧半帧。
        self._reset_vision_session()

    # 重置视觉解析、在线角色和心跳时刻。
    def _reset_vision_session(self):
        # 标记视觉链路离线。
        self.vision_connected = False
        # 清空角色握手信息，重连后要求重新确认双相机。
        self.vision_roles.clear()
        # 重置角色握手状态，等待视觉端下次主动发送版本帧。
        self.vision_enabled = False
        # 清除可能跨连接残留的半帧。
        self.vision_parser.reset()
        # 清空每路每命令的超时记录。
        self.vision_last_seen.clear()
        # 重置有效帧时间。
        self.vision_last_any = 0.0

    # 释放所有 I/O 设备并关闭 ROS 节点。
    def destroy_node(self):
        # 关闭视觉 UDP、TCP 和串口链路。
        self._close_vision_link()
        # 关闭电机串口链路。
        if self.motor_serial is not None:
            # 释放电机 UART 句柄。
            self._close_serial(self.motor_serial)
        # 关闭舵机串口链路。
        if self.servo_serial is not None:
            # 释放舵机 UART 句柄。
            self._close_serial(self.servo_serial)
        # 调用 ROS 节点基类释放资源。
        super().destroy_node()

# 主程序入口：初始化 ROS、运行节点并在退出时清理资源。
def main(args=None):
    # 初始化 ROS 2 Python 客户端。
    rclpy.init(args=args)
    # 创建导航主控节点。
    node = NavigationNode()
    # 运行单线程 executor，所有 I/O 均为非阻塞轮询。
    try:
        # 持续处理定时器和服务回调。
        rclpy.spin(node)
    # 在 Ctrl+C 或 ROS 关停时释放硬件接口。
    except KeyboardInterrupt:
        # 记录正常的人工关停信号。
        node.get_logger().info('navigation node interrupted')
    # 确保节点资源被关闭。
    finally:
        # 清理串口、网络套接字和 ROS 句柄。
        node.destroy_node()
        # 关闭 ROS 2 上下文。
        rclpy.shutdown()
