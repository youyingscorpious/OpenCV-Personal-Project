# Robopioneers ROS 2 导航主控

适用 Ubuntu 22.04 + ROS 2 Humble。包按 `3rdversion_motor_pid` 和 `3rdversion_periph_pid` 固件实现：一个 ROS 节点管理电机 MCU、舵机/外设 MCU 两条 UART，并接收视觉组处理后的协议帧。导航端不启动摄像头、不处理图像、不使用 Nav2。

## 已实现内容

- 两路独立串口，默认 `/dev/ttyUSB0` 电机、`/dev/ttyUSB1` 外设，均为 115200 8N1。
- 电机板使用 `AA 55 LEN CMD DATA CRC16`；导航在开始循迹时发送一次 `CMD_LINE=0x07`，由电机板读取 PB12–PB15 并执行本地速度环和循迹 PID。
- 循迹时导航每 100 ms 发送 `CMD_PING=0x00`，防止电机板 300 ms 看门狗停车；`CMD_CHASSIS=0x01` 只用于人工暂停、STOP 停车等手动速度控制，不用于周期循迹。
- 电机板 `0x11 CHASSIS_STATUS` 提供 bit0/PB12（最左）到 bit3/PB15（最右）的循迹位、PID 遥测、里程、航向及路口/丢线事件；外设板循迹位不再用于导航控制。
- 视觉颜色 1/2/3 仍会被识别并发布，但当前电机固件的 `CMD_LINE` 没有红/黄/蓝分支选择参数。遇到电机板路口事件后，导航会保持停车并进入 `WAIT_ROUTE_MANEUVER`，需要电控端增加分支控制接口后才能自动驶入选定支路。
- 视觉 STOP 后发送左右轮零速，等待电机编码器实测速度连续为零并保持设定时间，再向外设板发送抓取、放置、复位动作。
- 电机和舵机状态超时或报错时进入故障状态，不继续执行抓放动作。
- 视觉链路默认 UDP 监听 `0.0.0.0:5000`，适配“手机开热点、Ubuntu 与视觉板连接热点”的网络；视觉板应向 Ubuntu 热点网卡地址单播，或向正确的热点子网广播。

电控 UART 字段、状态值与停车门控详见 [docs/MCU_INTERFACE.md](docs/MCU_INTERFACE.md)。

状态与事件转移规则详见 [导航状态机说明](docs/STATE_MACHINE.md)。此次整理仅调整 Python 节点内部的状态转移组织，不改变 ROS 2 Humble、Ubuntu 22.04、UART/视觉通信协议、参数或 ROS 服务接口。

## 安装

```bash
sudo apt update  # 更新 Ubuntu 软件包索引。
sudo apt install -y ros-humble-rclpy ros-humble-std-msgs ros-humble-std-srvs python3-serial  # 安装 ROS 2 和 UART 依赖。
cd ~/GPT_ws  # 进入 ROS 2 工作空间。
colcon build --packages-select robopioneers_navigation  # 编译导航包。
source /opt/ros/humble/setup.bash  # 加载 ROS 2 Humble。
source install/setup.bash  # 加载当前工作空间。
```

## 启动

先按实际设备确认串口名，再运行：

```bash
ros2 launch robopioneers_navigation navigation.launch.py motor_port:=/dev/ttyUSB0 servo_port:=/dev/ttyUSB1 motor_profile:=core_crc16 vision_transport:=udp vision_udp_bind_port:=5000  # 启动导航并连接 MCU 和 UDP 视觉端。
```

手机作为 Wi-Fi AP 时，Ubuntu 和视觉开发板均连接手机热点。视觉板的 UDP 目的地址必须设为 Ubuntu 在该热点下的 IP 地址，目的端口设为 `5000`；不要默认沿用 `192.168.4.255`，除非手机热点的实际网段确实对应这个广播地址。按 [手机热点连接配置与控制原理](docs/PHONE_HOTSPOT_SETUP.md) 配置并检查网络。TCP 仅作为另行配置的替代链路；若使用，导航端 `vision_host` 必须设置为视觉板热点地址。视觉组若使用 USB-UART，可设置 `vision_transport:=serial` 和独立的 `vision_port_device`。

视觉模块默认每秒自动发送版本和心跳，并由识别程序逐帧调用目标列表 API；它不处理导航端发来的启动命令，所以视觉识别循环应自行启动。若有两个视觉来源，必须分别发送 `SRC/role=1` 和 `SRC/role=2`，否则 `/navigation/start` 会等待缺失角色。

两块 MCU、视觉角色均连接后，启动比赛状态机：

```bash
ros2 service call /navigation/start std_srvs/srv/Trigger {}  # 请求状态机开始比赛。
ros2 topic echo /navigation/state  # 查看比赛流程状态。
ros2 topic echo /navigation/motor/line_bits  # 查看电机板 bit0..bit3 的四路循迹输入。
```

## 参数校准

`config/navigation.yaml` 中 `motor_line_base_speed_mm_s` 和 `motor_line_turn_limit_mm_s` 通过 `CMD_LINE` 传给电机固件；增量 PID 增益、传感器有效电平和转向符号需在电机固件中标定。当前导航包不会根据 `0x11` 循迹位自行计算左右轮速度。

外设固件中的夹爪角度仍是占位参数，实物抓放前由机械组和电控组完成舵机标定。

## ROS 话题和服务

| 接口 | 类型 | 用途 |
|---|---|---|
| `/navigation/start` | `std_srvs/srv/Trigger` | 启动比赛流程 |
| `/navigation/stop` | `std_srvs/srv/Trigger` | 受控暂停：保持零速、不触发抓放、可恢复 |
| `/navigation/resume` | `std_srvs/srv/Trigger` | 从 `PAUSED` 回到暂停前的循迹阶段 |
| `/navigation/state` | `std_msgs/msg/String` | 流程状态与故障详情 |
| `/navigation/vision/target_list` | `std_msgs/msg/String` | 视觉颜色、距离与 STOP 识别结果 |
| `/navigation/vision/target_coord` | `std_msgs/msg/String` | 视觉端处理后的目标坐标 |
| `/navigation/vision/heartbeat` | `std_msgs/msg/String` | 视觉状态、帧率、错误数和处理耗时 |
| `/navigation/vision/color` | `std_msgs/msg/UInt8` | 颜色编码：1 红、2 黄、3 蓝 |
| `/navigation/route` | `std_msgs/msg/UInt8` | 已选择路线色码 |
| `/navigation/vision/stop_detected` | `std_msgs/msg/Bool` | STOP 检测状态 |
| `/navigation/motor/state` | `std_msgs/msg/UInt8` | 电机 MCU 状态 |
| `/navigation/motor/line_bits` | `std_msgs/msg/UInt8` | 电机板 PB12–PB15 四路循迹输入位图 |
| `/navigation/motor/chassis_status` | `std_msgs/msg/String` | 电机 0x11 循迹误差、PID 修正、里程、航向和事件 JSON |
