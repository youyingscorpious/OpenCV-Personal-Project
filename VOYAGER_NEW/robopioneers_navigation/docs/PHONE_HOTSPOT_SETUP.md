# 手机热点连接配置与控制原理

本文适用于手机作为 Wi-Fi AP、Ubuntu 导航电脑和视觉开发板都作为 STA 接入同一热点的结构：

```text
手机热点（AP）
├── Ubuntu 22.04 + ROS 2（STA，导航端）
└── 视觉开发板（STA，视觉端）
```

## 1. 配置手机热点

1. 在手机上开启个人热点，设置热点名称和密码。
2. 允许至少两台设备接入；如果手机有“客户端隔离”“设备间不可见”之类选项，应关闭它。
3. 记下热点当前使用的 IP 网段。不同手机或系统版本的网段可能不同，不要假定一定是 `192.168.4.0/24`。
4. 建议在手机热点的已连接设备或 DHCP 设置中，为 Ubuntu 电脑保留固定租约；这样视觉板的 UDP 目的 IP 不会因重连而变化。

## 2. 让两台设备接入同一热点

1. 在 Ubuntu 桌面 Wi-Fi 菜单中连接手机热点。
2. 将视觉开发板配置成 STA 模式，并用相同热点名称和密码连接。
3. 在手机热点设备列表中确认 Ubuntu 和开发板都已连接。
4. 在 Ubuntu 终端查看热点网卡地址：

```bash
ip -4 -brief address  # 查看所有网卡的 IPv4 地址，找到连接手机热点的无线网卡。
ip route              # 查看默认路由和当前热点对应的网络接口。
```

记下 Ubuntu 热点网卡的 IPv4 地址，例如 `192.168.43.20`。示例地址仅用于说明，必须替换成你实际查到的地址。

## 3. 配置视觉板发送地址

在视觉端 `Communication_comm.py` 的 UDP 配置中设置：

- UDP 目标 IP：Ubuntu 热点网卡的实际 IPv4 地址，例如 `192.168.43.20`。
- UDP 目标端口：`5000`。
- 传输方式：UDP。
- 两个视觉来源分别使用 `SRC/role=1` 和 `SRC/role=2`。

优先使用发往 Ubuntu 的单播。不要直接沿用默认 `192.168.4.255`，除非 Ubuntu 所在热点子网确实是 `192.168.4.0/24` 且该地址确为广播地址。若选择广播，必须使用手机热点实际子网的广播地址，并确认热点允许客户端互相通信。

导航端当前监听 `0.0.0.0:5000`，表示在所有本机网卡上接收端口 `5000` 的 UDP 报文；它可以接收视觉板发来的单播，不要求把监听地址改成视觉板 IP。

## 4. 编译并启动导航包

在 Ubuntu 工作空间中编译并加载包：

```bash
cd ~/GPT_ws  # 进入 ROS 2 工作空间。
source /opt/ros/humble/setup.bash  # 加载 ROS 2 Humble 环境。
colcon build --packages-select robopioneers_navigation  # 编译导航功能包。
source install/setup.bash  # 加载刚编译的功能包。
```

通过 UDP 启动节点，并保留默认的全网卡监听：

```bash
ros2 launch robopioneers_navigation navigation.launch.py motor_port:=/dev/ttyUSB0 servo_port:=/dev/ttyUSB1 vision_transport:=udp vision_udp_bind_address:=0.0.0.0 vision_udp_bind_port:=5000  # 连接两块 MCU，并监听视觉 UDP 数据。
```

如果希望只监听手机热点网卡，也可将 `vision_udp_bind_address` 替换为 Ubuntu 热点网卡的实际 IP，例如：

```bash
ros2 launch robopioneers_navigation navigation.launch.py motor_port:=/dev/ttyUSB0 servo_port:=/dev/ttyUSB1 vision_transport:=udp vision_udp_bind_address:=192.168.43.20 vision_udp_bind_port:=5000  # 示例地址应替换成 Ubuntu 实际热点 IP。
```

推荐先使用 `0.0.0.0`，这样无需因热点 DHCP 地址变化而修改导航端配置。视觉板的 UDP 目标地址仍要指向 Ubuntu 当前热点 IP。

## 5. 检查通信

先确认导航节点日志出现 `vision UDP listening: 0.0.0.0:5000`。再开另一个终端检查网卡是否收到 UDP 数据：

```bash
sudo tcpdump -ni any udp port 5000  # 观察是否有视觉板发到 Ubuntu 的 UDP 报文。
```

再查看导航端解码后的消息：

```bash
ros2 topic echo /navigation/vision/heartbeat  # 查看视觉端心跳、帧率和处理耗时。
ros2 topic echo /navigation/vision/target_list  # 查看颜色、距离和 STOP 识别结果。
ros2 topic echo /navigation/vision/target_coord  # 查看视觉端发布的目标坐标。
```

若 `tcpdump` 看不到报文，检查视觉板目标 IP/端口、热点设备间通信和 Ubuntu 是否连在正确热点。若 `tcpdump` 能看到报文但 ROS 话题没有有效数据，检查帧格式、SUM8 校验、SRC/role 和负载长度。若系统启用了 UFW，可允许接收端口：

```bash
sudo ufw allow 5000/udp  # 允许 Ubuntu 接收导航程序使用的 UDP 端口。
```

手机热点本身负责给两台 STA 分配地址和转发同网段流量；两台设备在同一热点下通信不需要配置公网端口映射。

## 6. 控制原理

```text
视觉板识别并调用通信 API
          │ UDP 单播：Ubuntu热点IP:5000
          ▼
导航端校验 AA55 帧、长度和 SUM8
          │
          ├── SRC/role 1、2 的版本帧齐全：允许启动流程
          ├── 颜色 1/2/3：分别选红左/黄中/蓝右路线目标
          └── STOP 标志：要求底盘停车

电机 MCU 本地采样 PB12–PB15 并运行循迹 PID
          │
          ▼
导航端计算线位置误差与左右轮速度
          │ UART CMD_CHASSIS
          ▼
电机 MCU 执行循迹和驱动

收到 STOP → 下发零轮速 → 检查编码器实测速度和停稳时间
          → 车停稳后才通过另一 UART 请求舵机抓取、放置和复位
```

手机热点只承担网络接入与转发，不参与比赛状态机。视觉板通过无线网络把已经处理好的协议帧推送给导航端；导航节点不接摄像头图像，也不向视觉板发送启动命令。

收到视觉帧后，导航节点先验证 `AA 55` 帧头、数据长度和 SUM8，再根据 `SRC` 分辨视觉来源。版本帧中的 `role` 必须与 `SRC` 一致；角色 1 和 2 都上线后，`/navigation/start` 才允许开始。视觉程序应主动调用版本/心跳接口，并持续调用目标列表接口；没有目标时也要发送空目标列表。

比赛运行时，导航发送 `0x07 LINE` 启动电机 MCU 本地循迹 PID，并每 100 ms 发送 `0x00 PING`。电机 MCU 通过 `0x11 CHASSIS_STATUS` 回报循迹位、PID 遥测、里程、航向及事件。视觉颜色仍由导航记录，但当前电机固件没有红/黄/蓝分支选择命令；路口事件发生后导航会保持停车，直到电控端提供分支控制接口。收到 STOP 后，导航下发零速，并等待电机状态帧报告编码器实测左右轮速度连续低于阈值且达到停稳时间；确认后才启动舵机动作序列。视觉或底盘遥测超时、MCU 故障或停稳确认超时都会进入故障状态，阻止继续抓放。

## 7. 网络和安全边界

- Ubuntu 和视觉板必须在同一个手机热点子网内；仅仅都能上网不代表彼此一定可达。
- 优先使用 Ubuntu DHCP 保留地址加 UDP 单播，减少广播被热点过滤的可能。
- 如果热点启用了客户端隔离，UDP 报文无法在 Ubuntu 与视觉板之间直达，需要关闭隔离或换用允许客户端互通的接入点。
- 当前导航节点把心跳和目标列表超时纳入运行保护；比赛中断网会让导航端进入故障并停车，不会继续机械臂动作。
- 热点断开后，Ubuntu 若切换到其他网络，IP 可能变化；需同步更新视觉板目标 IP，或重新连接手机热点后再开始比赛。
