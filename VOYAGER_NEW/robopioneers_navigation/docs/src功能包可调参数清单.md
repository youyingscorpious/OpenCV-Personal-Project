# src 功能包可调参数清单（Robopioneers 2027 · ROS 2 Humble）

> 适用范围：`C:\Users\39410\Desktop\VOYAGER\src\` 下三个包
>
> | 包名 | 角色 | 是否上车 |
> |---|---|---|
> | `robopioneers_navigation` | 真机导航主控（状态机 + 双 UART + 视觉链路） | 是 |
> | `robopioneers_nav_sim` | Gazebo 仿真 + 旧版单字节/CRC16 桥接 | 否（仿真/联调） |
> | `robopioneers_arm_tf` | 手眼 TF 变换 + 抓放笛卡尔目标点生成 | 是（机械臂侧） |

## 0. 参数分级与修改方式

| 级别 | 含义 | 修改位置 | 是否改代码 | 生效方式 |
|---|---|---|---|---|
| **A 类** | ROS 2 声明参数（`declare_parameter`） | `config/*.yaml` 或 launch 命令行覆盖 | 否 | 改 YAML 后重启节点；`--symlink-install` 下无需重编译，否则 `colcon build --packages-select <包名>` |
| **B 类** | 源码内硬编码阈值 / 周期 / 几何量 | `.py` / `.xacro` / 场地生成脚本 | 是 | 改完重新编译；URDF 重启 launch 即生效；场地脚本改完需重新运行生成 `.world` |
| **C 类** | 协议常量（帧头、CMD、状态码、事件位） | `protocols.py` / `*_protocol.py` | 一般不改 | 必须与 MCU 固件、视觉板三方对齐后才能动 |

**路径基准**：下文所有相对路径均以 `src/` 为根目录，例如 `robopioneers_navigation/config/navigation.yaml`。
**行号说明**：`YAML 行` 指配置文件行号，`declare 行` 指节点源码 `declare_parameter(参数名, 默认值)` 所在行号，二者成对出现便于双向核对。

---

# 1. robopioneers_navigation（真机导航主控）

## 1.1 A 类 ROS 2 参数（共 22 个）

- 配置文件：`robopioneers_navigation/config/navigation.yaml`
- 节点声明：`robopioneers_navigation/robopioneers_navigation/navigation_node.py`
- launch 文件：`robopioneers_navigation/launch/navigation.launch.py`
- 节点名：`robopioneers_navigation`（`navigation_node.py:43`）

| # | 参数名 | 默认值 | 含义 / 单位 / 取值范围 | YAML 行 | declare 行 | launch 可覆盖 |
|---:|---|---|---|---:|---:|:---:|
| 1 | `motor_port` | `/dev/ttyUSB0` | 电机板 UART 设备路径；建议用 `/dev/serial/by-id/<设备序列号>` 固定 | `navigation.yaml:6` | `navigation_node.py:45` | ✅ `navigation.launch.py:21` |
| 2 | `servo_port` | `/dev/ttyUSB1` | 外设/舵机板 UART 设备路径，**必须与电机板不同设备** | `navigation.yaml:8` | `navigation_node.py:47` | ✅ `navigation.launch.py:23` |
| 3 | `motor_baud` | `115200` | 电机板 UART 波特率 | `navigation.yaml:10` | `navigation_node.py:49` | ❌ |
| 4 | `servo_baud` | `115200` | 外设板 UART 波特率 | `navigation.yaml:12` | `navigation_node.py:51` | ❌ |
| 5 | `motor_profile` | `core_crc16` | 电机协议档；当前仅实现 `core_crc16`（校验见 `navigation_node.py:232`） | `navigation.yaml:14` | `navigation_node.py:53` | ✅ `navigation.launch.py:25` |
| 6 | `vision_transport` | `udp` | 视觉链路：`udp` / `tcp` / `serial`（校验见 `navigation_node.py:236`） | `navigation.yaml:16` | `navigation_node.py:55` | ✅ `navigation.launch.py:27` |
| 7 | `vision_host` | `192.168.4.1` | 视觉 TCP 服务端地址；**仅 TCP 模式使用**，UDP 模式不读取 | `navigation.yaml:18` | `navigation_node.py:57` | ❌ |
| 8 | `vision_port` | `5000` | 视觉 TCP 服务端口；**仅 TCP 模式使用** | `navigation.yaml:20` | `navigation_node.py:59` | ❌ |
| 9 | `vision_udp_bind_address` | `0.0.0.0` | UDP 本地绑定地址；`0.0.0.0` 监听所有网卡 | `navigation.yaml:22` | `navigation_node.py:61` | ✅ `navigation.launch.py:29` |
| 10 | `vision_udp_bind_port` | `5000` | UDP 监听端口；与视觉板 `peer_port=5000` 对应 | `navigation.yaml:24` | `navigation_node.py:63` | ✅ `navigation.launch.py:31` |
| 11 | `vision_port_device` | `/dev/ttyUSB2` | 视觉串口设备；**仅 `vision_transport=serial` 使用** | `navigation.yaml:26` | `navigation_node.py:65` | ❌ |
| 12 | `vision_baud` | `115200` | 视觉串口波特率；**仅 serial 模式使用** | `navigation.yaml:28` | `navigation_node.py:67` | ❌ |
| 13 | `motor_status_is_measured` | `true` | 电机状态帧速度是否为编码器实测值；`false` 时锁死舵机动作（`navigation_node.py:238-240`、`286-290`） | `navigation.yaml:30` | `navigation_node.py:69` | ❌ |
| 14 | `motor_line_base_speed_mm_s` | `150` | 电机板本地循迹基础速度，mm/s；节点内软件限幅 **100~800**（`navigation_node.py:115`） | `navigation.yaml:32` | `navigation_node.py:71` | ❌ |
| 15 | `motor_line_turn_limit_mm_s` | `300` | 电机板本地循迹转向修正限幅，mm/s；节点内软件限幅 **50~600**（`navigation_node.py:117`） | `navigation.yaml:34` | `navigation_node.py:73` | ❌ |
| 16 | `stopped_samples_required` | `3` | 连续多少帧实测零速反馈后才允许启动机械臂 | `navigation.yaml:36` | `navigation_node.py:75` | ❌ |
| 17 | `stopped_speed_threshold` | `10` | 判定零速的绝对速度阈值，单位须与 MCU 状态帧一致（mm/s） | `navigation.yaml:38` | `navigation_node.py:77` | ❌ |
| 18 | `stopped_dwell_seconds` | `0.5` | 实测零速需持续的最短时间，秒 | `navigation.yaml:40` | `navigation_node.py:79` | ❌ |
| 19 | `motor_stop_timeout_seconds` | `5.0` | 等待电机停稳确认的超时；超时后禁止舵机动作 | `navigation.yaml:42` | `navigation_node.py:81` | ❌ |
| 20 | `arm_action_timeout_seconds` | `15.0` | 单个机械臂动作等待 `BUSY→DONE` 的最长秒数 | `navigation.yaml:44` | `navigation_node.py:83` | ❌ |
| 21 | `vision_source_timeout_seconds` | `0.6` | 单路相机目标列表超时；兼容 20 Hz 限频与短时 UDP 丢包 | `navigation.yaml:46` | `navigation_node.py:85` | ❌ |
| 22 | `vision_link_timeout_seconds` | `1.5` | 整条视觉链路超时；给 1 Hz 自动心跳留调度余量 | `navigation.yaml:48` | `navigation_node.py:87` | ❌ |

> **launch 只能覆盖上表 6 个**（#1、#2、#5、#6、#9、#10），其余参数只能改 `navigation.yaml` 后重启/重编译。命令行覆盖示例：
>
> ```bash
> ros2 launch robopioneers_navigation navigation.launch.py \
>   motor_port:=/dev/serial/by-id/usb-<芯片序列号>-if00 \
>   servo_port:=/dev/serial/by-id/usb-<芯片序列号>-if01
> ```

## 1.2 B 类源码硬编码（`navigation_node.py`，需改代码）

| 项目 | 当前值 | 含义 / 触发位置 | 源码位置 |
|---|---:|---|---|
| 主通信轮询周期 | `0.02 s`（50 Hz） | 串口读写、视觉帧轮询、状态检查的统一定时器 | `navigation_node.py:228` |
| 串口断线重连间隔 | `1.0 s` | 下次重连等待时长；初始时刻 `next_reconnect=0.0`，判断 `now >= next_reconnect` | `navigation_node.py:151`、`427`、`433` |
| 串口写超时 | `0.05 s` | 电机板 / 外设板 / 视觉串口三处 `serial.Serial(port, baud, timeout=0, write_timeout=0.05)` | `navigation_node.py:456`、`470`、`541` |
| 循迹基础速度软件限幅 | `100 ~ 800` mm/s | `max(100, min(800, int(参数值)))` | `navigation_node.py:115` |
| 循迹转向限幅软件限幅 | `50 ~ 600` mm/s | `max(50, min(600, int(参数值)))` | `navigation_node.py:117` |
| 机械臂动作序列 | `[1, 2, 3]` | 1 抓取、2 放置、3 复位；对应外设板 `CMD_ARM=0x03` 动作字节 | `navigation_node.py:169`；发送处 `1057`、`1069` |
| 循迹遥测新鲜度阈值 | `0.35 s` | 底盘遥测超时即判故障（两处相同判断） | `navigation_node.py:395`、`971` |
| 电机 PING 心跳周期 | `0.1 s`（10 Hz） | 低于固件 300 ms 看门狗上限；`now - last_motor_ping >= 0.1` | `navigation_node.py:983` |
| 轮速 int16 裁剪 | `±32768 / ±32767` | 打包 `<hh>` 前的数值边界 | `navigation_node.py:996` |
| 强制双视觉角色 | `{1, 2}` | 启动前与版本握手阶段都要求两路相机角色齐全 | `navigation_node.py:278`、`809` |
| 颜色 / 路线编码 | `1 红、2 黄、3 蓝` | 视觉颜色码与电控路线码一致；只接受 1/2/3 | `navigation_node.py:926-928`、`939` |
| 状态迁移表 `TRANSITIONS` | — | 各状态允许的事件与目标状态；新增/修改赛程流程在此表 | `navigation_node.py:22-36` |
| 启动前置条件 | — | 电机状态帧、底盘遥测、外设状态帧、双视觉角色、实测速度标志逐项检查 | `navigation_node.py:273-290` |
| 发布话题名 | — | 目标列表 / 坐标 / 心跳 / 颜色 / 路线 / STOP / 电机状态 / 底盘遥测 / 循迹位 / 导航状态 | `navigation_node.py:202-220` |
| 服务名 | — | `/navigation/start`、`/navigation/stop`、`/navigation/resume` | `navigation_node.py:222-226` |
| 故障急停命令 | `CMD_ESTOP` | 故障时向电机板、外设板各发一帧急停 | `navigation_node.py:1128`、`1131`（引用见 `1122` `_fault`） |

## 1.3 C 类协议常量（`robopioneers_navigation/robopioneers_navigation/protocols.py`，勿随意改）

| 常量 | 值 | 含义 | 位置 |
|---|---:|---|---|
| `HEAD` | `AA 55` | 电控 UART 帧头 | `protocols.py:5` |
| `MAX_ELECTRICAL_DATA` | `16` | 电控帧 DATA 最大字节数 | `protocols.py:7` |
| `MAX_VISION_DATA` | `32` | 视觉帧 DATA 最大字节数 | `protocols.py:9` |
| `CMD_CHASSIS` | `0x01` | 导航→电机：左右轮目标速度 `<hh` | `protocols.py:11` |
| `CMD_PING` | `0x00` | 导航→电机：心跳 | `protocols.py:13` |
| `CMD_LINE` | `0x07` | 导航→电机：启动本地循迹 `<hh` | `protocols.py:15` |
| `CMD_CLEAR_FAULT` | `0x05` | 导航→电机/外设：清除故障 | `protocols.py:17` |
| `CMD_SERVO` | `0x02` | 导航→外设：舵机 ID + 角度 | `protocols.py:19` |
| `CMD_ARM` | `0x03` | 导航→外设：机械臂动作字节 | `protocols.py:21` |
| `CMD_STATUS` | `0x10` | 电机/外设→导航：状态回报 | `protocols.py:23` |
| `CMD_CHASSIS_STATUS` | `0x11` | 电机→导航：循迹遥测/事件 | `protocols.py:25` |
| `STATE_IDLE / STATE_RUNNING` | `0 / 1` | 电机状态码 | `protocols.py:27` |
| `STATE_FAULT / STATE_TIMEOUT` | `4 / 5` | 电机状态码 | `protocols.py:29` |
| `STATE_JUNCTION / STATE_LINE_LOST` | `6 / 7` | 电机状态码 | `protocols.py:31` |
| `STATE_ARM_BUSY / STATE_ARM_DONE` | `2 / 3` | 外设状态码 | `protocols.py:33` |
| `EVENT_JUNCTION` | `0x01` | 底盘事件位：路口 | `protocols.py:35` |
| `EVENT_LINE_LOST` | `0x02` | 底盘事件位：丢线 | `protocols.py:37` |
| `VISION_TARGET_LIST` | `0x01` | 视觉→导航：目标列表 | `protocols.py:39` |
| `VISION_TARGET_COORD` | `0x02` | 视觉→导航：目标坐标 | `protocols.py:41` |
| `VISION_HEARTBEAT` | `0x03` | 视觉→导航：心跳 | `protocols.py:43` |
| `VISION_ERROR` | `0x04` | 视觉→导航：错误上报 | `protocols.py:45` |
| `VISION_VERSION` | `0x05` | 视觉→导航：版本/角色握手 | `protocols.py:47` |
| `ROUTE_CODES` | `{1:1, 2:2, 3:3}` | 颜色码到路线码映射 | `protocols.py:49` |

---

# 2. robopioneers_nav_sim（仿真 + 旧版桥接）

## 2.1 A 类参数：链路节点 `navigation_links_node.py`

- 节点文件：`robopioneers_nav_sim/robopioneers_nav_sim/navigation_links_node.py`
- 该节点同时管理视觉 TCP、电控 UART、`cmd_vel` 差速换算与链路状态发布

| # | 参数名 | 默认值 | 含义 / 取值 | declare 行 |
|---:|---|---|---|---:|
| 1 | `vision_host` | `192.168.4.1` | 视觉板 AP 默认固定 IP（TCP 模式） | `navigation_links_node.py:39` |
| 2 | `vision_port` | `5000` | 视觉 TCP 服务端端口 | `navigation_links_node.py:40` |
| 3 | `uart_device` | `/dev/ttyUSB0` | 电控 USB-UART 设备路径 | `navigation_links_node.py:41` |
| 4 | `uart_baud` | `115200` | STM32 USART2 波特率 | `navigation_links_node.py:42` |
| 5 | `electrical_profile` | `route_byte` | 电控协议档：`route_byte`（单字节路线码）/ `core_crc16`（CRC16 轮速帧）；非法值拒绝启动（`59-60`） | `navigation_links_node.py:43` |
| 6 | `wheel_separation` | `0.18` | 差速底盘轮距，米；仅 `core_crc16` 模式使用；必须 > 0（`61-62`） | `navigation_links_node.py:44` |
| 7 | `max_wheel_speed` | `0.60` | 左右轮速度限幅，米/秒；必须 > 0（`61-62`） | `navigation_links_node.py:45` |
| 8 | `cmd_vel_timeout` | `0.25` | 多久没收到 `cmd_vel` 就向下发零速，秒 | `navigation_links_node.py:46` |
| 9 | `auto_start_vision` | `True` | 两路相机版本握手完成后自动发送开始识别命令 | `navigation_links_node.py:47` |
| 10 | `vision_mode` | `0` | 0 全开、1 仅颜色、2 仅坐标（随 `CMD_VISION_START` 下发）；非法值拒绝启动（`63-64`） | `navigation_links_node.py:48` |

> 注意：本包**没有独立 YAML 参数文件**，以上默认值写在 `declare_parameter` 里；如需覆盖，可在 `sim.launch.py` 中给该节点补 `parameters=[{参数名: 默认值}]`，或用 `ros2 param set`（注意串口设备只在打开时读取，改完需重连）。

## 2.2 A 类参数：单字节桥接节点 `uart_bridge_node.py`

- 节点文件：`robopioneers_nav_sim/robopioneers_nav_sim/uart_bridge_node.py`
- 节点名：`robopioneers_uart_bridge`（`uart_bridge_node.py:21`）

| # | 参数名 | 默认值 | 含义 | declare 行 |
|---:|---|---|---|---:|
| 1 | `device` | `/dev/ttyUSB0` | 单字节路线码 UART 设备 | `uart_bridge_node.py:22` |
| 2 | `baud` | `115200` | 串口波特率 | `uart_bridge_node.py:23` |
| 3 | `route_topic` | `/route_code` | 订阅的路线码话题（`UInt8`） | `uart_bridge_node.py:24` |
| 4 | `stop_on_shutdown` | `True` | 节点退出时是否补发一个 STOP 字节 | `uart_bridge_node.py:25` |

## 2.3 B 类源码硬编码

### 2.3.1 `navigation_links_node.py`

| 项目 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| 主周期定时器 | `0.02 s`（50 Hz） | 串口、速度命令、链路状态统一周期 | `navigation_links_node.py:97` |
| 路线码合法集合 | `(1, 2, 3, 4)` | 1 红、2 黄、3 蓝、4 STOP；其余丢弃 | `navigation_links_node.py:141` |
| 版本请求周期 | `1.0 s` | 缺少相机握手时每秒重发一次 `0x84` | `navigation_links_node.py:289` |
| 相机在线判定窗口 | `0.3 s` | 300 ms 内收到帧才算在线（两路相同） | `navigation_links_node.py:298-299` |
| 串口重连间隔 | `1.0 s` | 打开失败后至少等 1 秒再试（两处） | `navigation_links_node.py:200`、`274` |
| 串口写超时 | `0.05 s` | `serial.Serial(port, baud, timeout=0, write_timeout=0.05)` | `navigation_links_node.py:209` |
| 差速半轮距 | `wheel_separation / 2` | 由线速度/角速度换算左右轮速 | `navigation_links_node.py:247` |
| 轮速限幅公式 | `±max_wheel_speed` | 限幅后乘 1000 转 mm/s 打包 | `navigation_links_node.py:250-252` |
| 退出停车字节 | `0x04` | `route_byte` 模式退出时直接写 STOP 单字节 | `navigation_links_node.py:309` |

### 2.3.2 `uart_bridge_node.py`

| 项目 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| 定时器周期 | `0.2 s` | 重试串口打开与待发字节投递 | `uart_bridge_node.py:38` |
| 串口重连间隔 | `1.0 s` | 打开失败 / 关闭后等待时长（两处） | `uart_bridge_node.py:59`、`103` |
| 串口写超时 | `0.1 s` | `serial.Serial(port, baud, timeout=0, write_timeout=0.1)` | `uart_bridge_node.py:68` |
| 打开失败告警节流 | `5.0 s` | 两次告警最少间隔 5 秒，避免刷屏 | `uart_bridge_node.py:72` |

### 2.3.3 视觉 TCP 客户端 `vision_tcp_client.py`

| 项目 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| 默认 host | `192.168.4.1` | 构造函数默认视觉板 IP | `vision_tcp_client.py:26` |
| 默认 port | `5000` | 构造函数默认 TCP 端口 | `vision_tcp_client.py:27` |
| `reconnect_delay` | `1.0 s` | 断线后重连周期 | `vision_tcp_client.py:29` |
| 线程退出 join 超时 | `2.0 s` | 关闭时最多等待接收线程 2 秒 | `vision_tcp_client.py:60` |
| `connect` 超时 | `1.0 s` | 建立连接最长等待 | `vision_tcp_client.py:79` |
| `recv` 超时 | `0.2 s` | 接收循环每 200 ms 醒一次检查退出信号 | `vision_tcp_client.py:82` |

## 2.4 launch 参数与出生点（`robopioneers_nav_sim/launch/sim.launch.py`）

| 项目 | 当前值 | 含义 | 位置 |
|---|---|---|---|
| `world` 参数 | `worlds/competition.world` | Gazebo 世界文件；可命令行 `world:=<路径>` 覆盖 | 声明 `sim.launch.py:34`；默认路径 `sim.launch.py:13` |
| 出生点坐标 | `-x 0.0 -y -1.10 -z 0.0 -Y 1.5708` | **硬编码在 spawn 参数列表中**，不是 launch 参数；改出生点需改这里 | `sim.launch.py:29-30` |
| `use_sim_time` | `True` | 状态发布器固定使用仿真时钟（`sim.launch.py:24`） | `sim.launch.py:24` |

## 2.5 URDF 几何 / 物理参数（`robopioneers_nav_sim/urdf/robot.urdf.xacro`）

### 2.5.1 顶层 xacro 属性

| 属性 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| `pi` | `3.141592653589793` | 圆周率 | `robot.urdf.xacro:3` |
| `wheel_radius` | `0.050 m` | 主动轮半径 | `robot.urdf.xacro:4` |
| `wheel_width` | `0.026 m` | 主动轮宽度 | `robot.urdf.xacro:5` |
| `wheel_track` | `0.180 m` | 左右轮距（中心距） | `robot.urdf.xacro:6` |

### 2.5.2 车体与轮子惯量、尺寸

| 项目 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| 车体质量 / 惯量 | `2.2 kg`；`ixx=0.012, iyy=0.018, izz=0.022` | 底盘惯性参数 | `robot.urdf.xacro:15` |
| 车体外形 | `0.250 × 0.200 × 0.075 m`，中心 `z=0.105` | 视觉与碰撞盒（视觉、碰撞两处一致） | `robot.urdf.xacro:16-17` |
| 主动轮质量 / 惯量 | `0.18 kg`；`ixx=iyy=izz=0.00025` | 左右轮惯性参数 | `robot.urdf.xacro:23` |
| 主动轮关节阻尼 | `damping=0.02, friction=0.0` | 连续轮关节动力学 | `robot.urdf.xacro:27` |
| 万向轮 swivel 半径 / 长度 | `0.018 m / 0.035 m` | 垂直转向柱 | `robot.urdf.xacro:35-36` |
| 万向轮 swivel 位置 / 阻尼 | `(-0.075, 0, -0.005)`；`damping=0.1` | 转向关节位置与阻尼 | `robot.urdf.xacro:38` |
| 万向轮半径 / 长度 | `0.025 m / 0.018 m` | 滚动轮 | `robot.urdf.xacro:41-42` |
| 万向轮滚动关节阻尼 | `damping=0.01` | 滚动关节动力学 | `robot.urdf.xacro:44` |

### 2.5.3 相机安装与 Gazebo 接触参数

| 项目 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| 相机安装位姿 | `xyz=(0.105, 0, 0.135)` | `base_link → camera_link` 固定关节 | `robot.urdf.xacro:50` |
| 底盘摩擦 | `mu1=0.8, mu2=0.8` | 地面接触系数 | `robot.urdf.xacro:52` |
| 主动轮摩擦 / 刚度 / 阻尼 | `mu1=mu2=1.4, kp=100000, kd=1` | 左右轮相同（两处） | `robot.urdf.xacro:53-54` |
| 万向轮摩擦 | `mu1=mu2=0.2` | 低摩擦辅助轮 | `robot.urdf.xacro:55` |
| 相机帧率 | `15 Hz` | `update_rate` | `robot.urdf.xacro:59` |
| 相机视场 / 分辨率 / 裁剪 | `hfov=1.20 rad`；`640×480`；`near=0.05, far=10` | 相机成像参数 | `robot.urdf.xacro:60` |
| 关节状态发布频率 | `50 Hz` | `joint_state_publisher` 插件 | `robot.urdf.xacro:67` |
| 差速驱动频率 | `50 Hz` | `gazebo_ros_diff_drive` 插件 | `robot.urdf.xacro:73` |
| 差速轮距 / 轮径 | `${wheel_track}` / `${2*wheel_radius}` | 由顶层属性派生 | `robot.urdf.xacro:74` |
| 最大轮扭矩 / 最大角加速度 | `3.0 N·m` / `5.0 m/s²` | 驱动限制 | `robot.urdf.xacro:75` |
| 里程计 / TF 开关 | `publish_odom=true, publish_odom_tf=true, publish_wheel_tf=false` | 里程计发布选项 | `robot.urdf.xacro:76` |
| 里程系 / 基座系 | `odometry_frame=odom`；`robot_base_frame=base_footprint` | TF 坐标系名 | `robot.urdf.xacro:77` |

## 2.6 场地生成脚本（`robopioneers_nav_sim/tools/generate_world.py`）

> 改完脚本必须重新运行 `python3 tools/generate_world.py`，重新生成 `worlds/competition.world` 后再启动仿真。

### 2.6.1 线条通用参数

| 项目 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| 线条默认高度 / 抬升 | `z=0.002, height=0.003` | 地面标线视觉厚度 | `generate_world.py:9` |
| 路线默认线宽 | `0.020 m` | `add_path` 默认 `width` | `generate_world.py:18` |
| 黑色底层线宽 / 高度 | `0.032 m`；`z=0.0015` | 彩色路线下方黑色衬底 | `generate_world.py:21` |

### 2.6.2 STOP 标志（`octagon_sign`）

| 项目 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| 标志中心 `(cx, cy, cz)` | `(0.0, 0.63, 0.34)` | 八角牌在场地中的位置 | `generate_world.py:48` |
| 八角牌半径 | `0.055 m` | 八段红边外接圆半径 | `generate_world.py:49` |
| 立杆半径 / 长度 | `0.006 m / 0.28 m` | 支撑杆几何 | `generate_world.py:50` |
| 中心圆盘半径 / 厚度 | `0.050 m / 0.010 m` | 红色 STOP 牌面 | `generate_world.py:61` |
| 字母笔画宽度 | `0.0032 m` | STOP 白色笔画粗细 | `generate_world.py:62` |

### 2.6.3 圆环与放置架

| 项目 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| 圆环外 / 内半径 | `0.040 / 0.032 m` | `ring_marker` 默认参数（分叉点、汇合点两个圆环共用） | `generate_world.py:75` |
| 放置架位置 `(rack_x, rack_y)` | `(0.0, 1.005)` | 平台中心坐标 | `generate_world.py:85` |
| 支腿横向间距 | `±0.055 m`（两组、前后各两） | 四条腿相对架心位置 | `generate_world.py:86-87` |
| 支腿尺寸 | `0.010 × 0.010 × 0.195 m` | 四条腿几何 | `generate_world.py:88` |
| 台面尺寸 / 高度 | `0.120 × 0.120 × 0.005 m`；`z=0.1975` | 平台顶板 | `generate_world.py:89` |
| 围栏尺寸 / 高度 | `0.010 厚、0.120 长、0.100 高`；`z=0.250` | 四面围栏（左/右/前/后） | `generate_world.py:90-93` |

### 2.6.4 路线几何与色卡

| 项目 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| 入口直线 | `(0,-1.10) → (0,-1.00)` | 起点后的 100 mm 直道 | `generate_world.py:101` |
| S 弯半径 `radius` | `0.30 m` | L-R-L 三圆弧统一半径 | `generate_world.py:102` |
| S 弯起点 | `(0.0, -1.00)`；初始朝向 `π/2` | 第一条弧起点与航向 | `generate_world.py:103-104` |
| 三段弧 sweep / 段数 | `+π/2 / 24`、`-π / 48`、`+π/2 / 24` | 左-右-左三个圆弧采样段数 | `generate_world.py:105-107` |
| 出口直线 | `(0,0.20) → (0,0.30)` | S 弯后到分叉点的 100 mm 直道 | `generate_world.py:111` |
| 三条分支点列 | `left / middle / right` 各 6 点 | 红/黄/蓝支路折线顶点 | `generate_world.py:114-116` |
| 分支颜色 | 红 `0.90 0.05 0.04`、黄 `1.00 0.66 0.02`、蓝 `0.02 0.20 0.90` | 三色线路标颜色 | `generate_world.py:117-119` |
| 汇合后直线 | `(0,-0.25) → (0,0.92)` | 汇合点到取料区的公共路线 | `generate_world.py:122` |
| 色卡托盘位置 / 尺寸 | `(-0.330, 0.02, 0.002)`；`0.16 × 0.055 × 0.004` | 三色样片托盘 | `generate_world.py:124` |
| 色卡位置 / 半径 | x 依次 `-0.380, -0.330, -0.280`；半径 `0.012 m`，厚 `0.004 m` | 三个圆盘样片 | `generate_world.py:125-126` |
| 起点区 | 中心 `(0,-1.10)`；半宽 `0.125`、半长 `0.10`；线宽 `0.006` | 绿色起点方框 | `generate_world.py:128-129` |
| 取料区 | 中心 `(0,0.74)`；半宽 `0.09`、半长 `0.04`；线宽 `0.006` | 黄色取料方框 | `generate_world.py:130-131` |
| 场地板 | `1.5 × 2.5 × 0.05 m`；`z=-0.025` | 白色底板（上表面齐平 z=0） | `generate_world.py:134` |
| 取料立方体 | `0.05 m` 边长；质量 `0.02 kg`；位姿 `(0,0.82,0.025)` | 动态纸块 | `generate_world.py:136` |
| 重力 | `0 0 -9.81` | 世界重力向量 | `generate_world.py:141` |
| 物理步长 / 实时速率 | `max_step_size=0.001`；`real_time_update_rate=1000` | ODE 物理参数 | `generate_world.py:142` |
| 概述相机位姿 | `(0, -1.65, 3.1)`；俯仰 `1.40`、偏航 `1.5708` | Gazebo GUI 默认视角 | `generate_world.py:150` |

## 2.7 C 类协议常量（备查，勿随意改）

| 文件 | 常量 | 位置 |
|---|---|---|
| `robopioneers_nav_sim/robopioneers_nav_sim/stm32_uart_protocol.py` | `HEAD=AA55`、`MAX_DATA=48`、`CMD_CHASSIS=0x01`、`CMD_SERVO=0x02`、`CMD_ARM=0x03`、`CMD_ESTOP=0x04`、`CMD_CLEAR_FAULT=0x05`、`CMD_STATUS=0x10` | `stm32_uart_protocol.py:5-13` |
| `robopioneers_nav_sim/robopioneers_nav_sim/uart_protocol.py` | `ROUTE_RED=0x01`、`ROUTE_YELLOW=0x02`、`ROUTE_BLUE=0x03`、`ROUTE_STOP=0x04`、`VALID_ROUTE_CODES` | `uart_protocol.py:3-8` |
| `robopioneers_nav_sim/robopioneers_nav_sim/vision_protocol.py` | `HEAD=AA55`、`MAX_DATA=32`、`SRC_CAMERA_1=0x01`、`SRC_CAMERA_2=0x02`、上行 CMD `0x01~0x05`、下行 CMD `0x81~0x86` | `vision_protocol.py:6-22` |

---

# 3. robopioneers_arm_tf（手眼 TF 与放置目标）

## 3.1 A 类参数：静态 TF 节点 `handeye_tf_node.py`

- 配置文件：`robopioneers_arm_tf/config/handeye.yaml`（顶层 `handeye_tf:` 分组）
- 节点声明：`robopioneers_arm_tf/robopioneers_arm_tf/handeye_tf_node.py`
- 节点名：`handeye_tf`（`handeye_tf_node.py:28`）

| # | 参数名 | 默认值 | 含义 / 单位 | YAML 行 | declare 行 |
|---:|---|---|---|---:|---:|
| 1 | `base_frame` | `base_link` | 机器人基座坐标系名 | `handeye.yaml:3` | `handeye_tf_node.py:29` |
| 2 | `arm_frame` | `arm_base_link` | 机械臂规划坐标系名 | `handeye.yaml:4` | `handeye_tf_node.py:30` |
| 3 | `base_to_arm_xyz` | `[0.0, 0.0, 0.0]` | `base_link → arm_base_link` 平移，米（**占位值，必须实测标定**） | `handeye.yaml:6` | `handeye_tf_node.py:31` |
| 4 | `base_to_arm_rpy` | `[0.0, 0.0, 0.0]` | `base_link → arm_base_link` 姿态，弧度（**占位值，必须实测标定**） | `handeye.yaml:7` | `handeye_tf_node.py:32` |
| 5 | `publish_base_to_arm` | `True` | 是否发布 `base_link → arm_base_link` 静态 TF；URDF 已含该树时设 `False` 避免重复父变换 | `handeye.yaml:8` | `handeye_tf_node.py:33` |
| 6 | `camera_frame` | `camera_link` | 相机安装坐标系名 | `handeye.yaml:9` | `handeye_tf_node.py:34` |
| 7 | `optical_frame` | `camera_optical_frame` | 相机光学坐标系名（REP-103：X 右、Y 下、Z 前） | `handeye.yaml:10` | `handeye_tf_node.py:35` |
| 8 | `publish_camera_optical_frame` | `True` | 是否发布 `camera_link → camera_optical_frame` 静态 TF | `handeye.yaml:11` | `handeye_tf_node.py:37` |
| 9 | `camera_to_optical_xyz` | `[0.0, 0.0, 0.0]` | 相机机械系到光学系平移，米 | `handeye.yaml:12` | `handeye_tf_node.py:38` |
| 10 | `camera_to_optical_rpy` | `[-π/2, 0.0, -π/2]` | 机械系到光学系 RPY，弧度；默认值即 REP-103 标准换算 | `handeye.yaml:13` | `handeye_tf_node.py:39-41` |

## 3.2 A 类参数：放置目标节点 `placement_target_node.py`

- 配置文件：`robopioneers_arm_tf/config/handeye.yaml`（`placement_target:` 分组，分组结构注意见 3.5）
- 节点声明：`robopioneers_arm_tf/robopioneers_arm_tf/placement_target_node.py`
- 节点名：`placement_target`（`placement_target_node.py:51`）

| # | 参数名 | 默认值 | 含义 / 单位 | YAML 行 | declare 行 |
|---:|---|---|---|---:|---:|
| 1 | `vision_pose_topic` | `/vision/object_pose` | 订阅视觉物体位姿话题（`PoseStamped`） | `handeye.yaml:17` | `placement_target_node.py:52` |
| 2 | `planning_frame` | `arm_base_link` | 目标点输出所在规划坐标系 | `handeye.yaml:18` | `placement_target_node.py:53` |
| 3 | `grasp_pose_topic` | `/arm/grasp_pose` | 发布抓取位姿话题 | `handeye.yaml:19` | `placement_target_node.py:54` |
| 4 | `place_pose_topic` | `/arm/place_pose` | 发布放置位姿话题 | `handeye.yaml:20` | `placement_target_node.py:55` |
| 5 | `waypoints_topic` | `/arm/placement_waypoints` | 发布五路径点 `PoseArray` 话题 | `handeye.yaml:21` | `placement_target_node.py:56` |
| 6 | `place_target_xyz` | `[0.0, 0.0, 0.0]` | 平台上目标**方块中心**位姿平移，米（**占位值，必须实测标定**） | `handeye.yaml:24` | `placement_target_node.py:57` |
| 7 | `place_target_rpy` | `[0.0, 0.0, 0.0]` | 平台目标方块姿态，弧度（**占位值，必须实测标定**） | `handeye.yaml:25` | `placement_target_node.py:58` |
| 8 | `tool_to_object_offset` | `[0.0, 0.0, 0.0]` | 工具中心到所持方块中心在工具局部系下的偏移，米（**占位值，必须实测标定**） | `handeye.yaml:27` | `placement_target_node.py:59` |
| 9 | `approach_height` | `0.08 m` | 抓取前在 +Z 方向的接近高度 | `handeye.yaml:28` | `placement_target_node.py:60` |
| 10 | `lift_height` | `0.08 m` | 抓取后在 +Z 方向的抬起高度 | `handeye.yaml:29` | `placement_target_node.py:61` |
| 11 | `place_clearance` | `0.08 m` | 放置前在 +Z 方向的避让高度 | `handeye.yaml:30` | `placement_target_node.py:62` |
| 12 | `max_object_pose_age` | `0.50 s` | 缓存视觉位姿允许的最大年龄；超时拒绝生成目标点 | `handeye.yaml:31` | `placement_target_node.py:63` |

> 参数合法性自检（启动即校验，非法值抛 `ValueError`）：三个高度必须 ≥ 0（`placement_target_node.py:73-74`）；`max_object_pose_age` 必须 > 0（`75-76`）；每个 xyz/rpy 必须为 3 个值（`103-107`）。

## 3.3 launch 参数（`robopioneers_arm_tf/launch/handeye_place.launch.py`）

| 项目 | 默认值 | 含义 | 位置 |
|---|---|---|---|
| `config` | `share/robopioneers_arm_tf/config/handeye.yaml` | 手眼标定参数文件路径；可命令行 `config:=<路径>` 覆盖 | 声明 `handeye_place.launch.py:16-17`；默认路径 `12-14` |
| `use_sim_time` | `false` | 仿真时钟开关；Gazebo 下设 `true`，真机保持 `false` | 声明 `handeye_place.launch.py:20-21`；两个节点分别加载于 `27`、`32` |

## 3.4 B 类源码硬编码（`placement_target_node.py`）

| 项目 | 当前值 | 含义 | 位置 |
|---|---:|---|---|
| 位姿年龄未来时刻容差 | `-0.10 s` | 时间戳比当前晚 0.1 s 内视为合法，避免时钟抖动误判 | `placement_target_node.py:141` |
| TF 变换超时 | `0.25 s` | `tf_buffer.transform(位姿, 目标坐标系, timeout=0.25)` | `placement_target_node.py:151` |
| 五路径点顺序 | 抓取接近 → 抓取 → 抬起 → 放置接近 → 放置 | `PoseArray.poses` 固定顺序 | `placement_target_node.py:179-185` |
| 工具偏移换算方向 | 物体位姿局部系内将偏移向量取反 | 由检测到的物体中心反推工具中心目标 | `placement_target_node.py:39-43` |
| 抬升方向 | 规划系 `+Z` | 接近 / 抬起 / 避让高度均沿规划系 +Z 叠加 | `placement_target_node.py:43`、`161-162`、`167-168` |

## 3.5 配置结构注意（改参数前必读）

- `handeye.yaml:1` 顶层节点键是 `handeye_tf`，对应 `handeye_tf_node.py:28` 的节点名，参数可正常加载。
- `handeye.yaml:15` 的 `placement_target:` 块被**嵌套在 `handeye_tf:` 命名空间内部**，其下 `handeye.yaml:16` 才是 `ros__parameters`；而放置节点名是 `placement_target`（`placement_target_node.py:51`）。ROS 2 按「顶层键 = 节点名」匹配参数，因此放置节点实际读不到 `handeye.yaml:17-31` 这组值，只能回落到 `declare_parameter` 中的代码默认值。当前代码默认值与 YAML 数值恰好一致，表面运行正常；一旦在 YAML 里改了 #6~#12 标定值而不改代码，会出现「改了不生效」。
- **修正方式**：把 YAML 改成两个平级顶层节点键：

  ```yaml
  handeye_tf:
    ros__parameters:
      # 3.1 节参数
  placement_target:
    ros__parameters:
      # 3.2 节参数
  ```

---

# 4. 修改方式速查

| 想改什么 | 去哪里改 | 是否改代码 | 是否需要重编译 |
|---|---|:---:|---|
| 串口设备、波特率、网络模式、速度阈值、超时时间（导航包 #1~#22） | `robopioneers_navigation/config/navigation.yaml` | 否 | `--symlink-install` 重启即可；否则 `colcon build --packages-select robopioneers_navigation` |
| 导航包 6 个常用链路参数临时换值 | launch 命令行 `param:=value` | 否 | 否（重启 launch 生效） |
| 仿真链路 / 差速 / 视觉模式参数 | `navigation_links_node.py:39-48`、`uart_bridge_node.py:22-25` 的 `declare_parameter` | 是 | 是 |
| 定时器周期、心跳周期、超时、在线窗口等阈值 | 对应节点 `.py` 的 B 类清单 | 是 | 是 |
| 车体尺寸、轮距、相机安装位姿、摩擦、相机参数 | `robot.urdf.xacro` | 是 | 重启 launch 即生效（xacro 实时展开） |
| 场地路线、标志、放置架、立方体位置与尺寸 | `tools/generate_world.py` | 是 | 改完重新运行脚本生成 `.world`，再启动仿真 |
| 手眼标定外参、放置点、工具偏移、抬升高度 | `handeye.yaml`（注意先按 3.5 修正分组） | 否 | 重启 launch 生效 |
| 协议帧头、CMD 编号、状态码、事件位 | `*protocols.py` / `*_protocol.py` | 是 | **必须先与 MCU、视觉板固件同步修改** |

## 附：与参数直接相关的排障提示

1. **导航节点启动即报 `ImportError: cannot import name 'CMD_ESTOP'`**：`navigation_node.py:19` 从 `protocols.py` 导入 `CMD_ESTOP` 并在 `1128`、`1131` 使用，但 `protocols.py:11-25` 只定义到 `CMD_CHASSIS/CMD_PING/CMD_LINE/CMD_CLEAR_FAULT/CMD_SERVO/CMD_ARM/CMD_STATUS/CMD_CHASSIS_STATUS`，缺少 `CMD_ESTOP = 0x04`。需在 `protocols.py` 补齐该常量后节点才能启动。
2. **YAML 里改了导航参数但不生效**：先确认该参数是否属于 launch 可覆盖的 6 个之一；其余参数节点只在 `__init__` 读取一次（`navigation_node.py:89-131`），运行中 `ros2 param set` 不会重新打开串口或刷新阈值，需重启节点。
3. **串口编号漂移**：不同 USB 口、上电顺序都会改变 `/dev/ttyUSBx`；固定方案是使用 `/dev/serial/by-id/usb-<芯片序列号>` 路径（见 `robopioneers_navigation/docs/QUICK_START.md:339-345`）。
4. **单相机调试无法启动导航主控**：`navigation_node.py:278` 强制要求视觉角色 `{1, 2}` 两路齐全；仅接一路相机时会被 `start` 服务拒绝，提示 `waiting for vision roles 1 and 2`。
