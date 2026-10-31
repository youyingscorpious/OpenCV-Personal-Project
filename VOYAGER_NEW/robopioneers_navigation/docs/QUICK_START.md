# 快速上手：从零把车跑起来

本文是一个**可照着敲的完整实例**，覆盖「确认串口 → 编译 → 带覆盖启动 → 验收 → 开赛」全流程。

适用：Ubuntu 22.04 + ROS 2 Humble，第三版电机固件 `3rdversion_motor_new` 与外设固件 `3rdversion_periph`。

---

## 场景设定

为了让每一步都有具体对象，本文假设以下现状（请替换成你的实际值）：

| 项目 | 值 |
|---|---|
| 电机板物理位置 | 左边 USB 口 |
| 外设板物理位置 | 右边 USB 口 |
| 实际枚举结果 | **电机板 = `/dev/ttyUSB1`，外设板 = `/dev/ttyUSB0`**（与默认值相反） |
| Ubuntu 热点 IP | `192.168.43.20` |
| 目标 | **不改任何代码和 YAML**，仅用命令行把端口对调 |

---

## 第 1 步：确认谁是谁

### 1.1 查看稳定的设备 ID

```bash
ls -l /dev/serial/by-id/
```

输出示例（序列号已简写）：

```
usb-STMicroelectronics_STM32_Virtual_ComPort_2054376C3034-if00 -> ../../ttyACM0
usb-STMicroelectronics_STM32_Virtual_ComPort_2063377A3034-if00 -> ../../ttyACM1
```

### 1.2 确认 tty 前缀

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

输出示例：

```
/dev/ttyUSB0  /dev/ttyUSB1
```

> ⚠️ **这一步不能跳过。** STM32 虚拟串口在不同驱动/系统上可能枚举成 `ttyACM*` 而不是 `ttyUSB*`。如果实际是 `ttyACM`，YAML 默认的 `/dev/ttyUSB0` 永远打不开，日志会一直刷 `UART unavailable`。

### 1.3 用「单插法」确认身份

这是唯一可靠的办法——一次只插一块板：

```bash
# ① 拔掉所有板子，只插电机板
ls /dev/ttyUSB*
```

假设输出 `/dev/ttyUSB1` → **电机板 = USB1**。

```bash
# ② 拔掉电机板，只插外设板
ls /dev/ttyUSB*
```

假设输出 `/dev/ttyUSB0` → **外设板 = USB0**。

结论：**两块板的编号与默认值（USB0 电机 / USB1 外设）正好相反**，所以后面必须覆盖。

> 如果测出来和默认值一致，那么第 4 步可以省略覆盖参数，直接 `ros2 launch robopioneers_navigation navigation.launch.py` 即可。

### 1.4 检查权限

```bash
groups | grep dialout
```

没有输出说明不在串口组，执行：

```bash
sudo usermod -aG dialout $USER
```

**必须重新登录（或重启）才生效**，`newgrp dialout` 只对当前 shell 有效。

---

## 第 2 步：编译

```bash
source /opt/ros/humble/setup.bash
cd ~/GPT_ws
colcon build --packages-select robopioneers_navigation
source install/setup.bash
```

成功标志：

```
Summary: 1 package finished [x.xxs]
```

---

## 第 3 步：确认 launch 参数

```bash
ros2 launch robopioneers_navigation navigation.launch.py --show-args
```

输出（节选）：

```
Arguments (pass arguments as '<name>:=<value>'):

    'motor_port':
        Motor MCU UART device.
        (default: '/dev/ttyUSB0')

    'servo_port':
        Servo MCU UART device.
        (default: '/dev/ttyUSB1')

    'motor_profile':
        Motor protocol profile.
        (default: 'core_crc16')

    ...（共 6 个）
```

**可用 launch 覆盖的只有这 6 个**：

| 参数 | 默认值 |
|---|---|
| `motor_port` | `/dev/ttyUSB0` |
| `servo_port` | `/dev/ttyUSB1` |
| `motor_profile` | `core_crc16` |
| `vision_transport` | `udp` |
| `vision_udp_bind_address` | `0.0.0.0` |
| `vision_udp_bind_port` | `5000` |

其余参数（如 `motor_line_base_speed_mm_s`、`stopped_dwell_seconds`）**只能改 `config/navigation.yaml` 后重新编译**。

---

## 第 4 步：带覆盖启动（核心动作）

```bash
ros2 launch robopioneers_navigation navigation.launch.py \
  motor_port:=/dev/ttyUSB1 \
  servo_port:=/dev/ttyUSB0
```

### 成功日志

```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [navigation_node-1]: process started with pid [8891]
[navigation_node-1] [INFO] [...] [robopioneers_navigation]: motor UART connected: /dev/ttyUSB1
[navigation_node-1] [INFO] [...] [robopioneers_navigation]: servo UART connected: /dev/ttyUSB0
[navigation_node-1] [INFO] [...] [robopioneers_navigation]: vision UDP listening: 0.0.0.0:5000
[navigation_node-1] [INFO] [...] [robopioneers_navigation]: navigation node ready; motor_profile=core_crc16, vision_transport=udp
```

**验收要点**：第 3、4 行的端口号必须是**对调后的**（`motor=USB1`、`servo=USB0`），证明覆盖生效了。

### 视觉上线后继续追加

视觉板开始推送后约几秒内出现：

```
[navigation_node-1] [INFO] [...] vision role 1 online; firmware=1.2, protocol=1.0
[navigation_node-1] [INFO] [...] vision role 2 online; firmware=1.2, protocol=1.0
[navigation_node-1] [INFO] [...] both vision roles online; waiting for API-pushed target frames
```

看到这三行，三条链路（两串口 + 一网络）就齐了。

### 语法规则

- `:=` **两侧不能有空格**，写成 `motor_port = /dev/ttyUSB1` 会报错
- 参数必须写在 launch 文件路径**之后**
- 多个参数用空格分隔，可任意组合

---

## 第 5 步：交叉验证

**新开终端 A** —— 查询实际生效的参数：

```bash
source ~/GPT_ws/install/setup.bash

ros2 param get /robopioneers_navigation motor_port
```

```
String value is: /dev/ttyUSB1
```

```bash
ros2 param get /robopioneers_navigation servo_port
```

```
String value is: /dev/ttyUSB0
```

再查一个**未被覆盖**的参数，证明 YAML 其余部分仍然加载：

```bash
ros2 param get /robopioneers_navigation motor_line_base_speed_mm_s
```

```
Integer value is: 150
```

> 这正好演示了「覆盖 YAML 的单个键，但未声明的参数无法覆盖」的边界。循迹 PID 增益仍由电机固件配置；导航只设置基础速度和转向限幅。

**新开终端 B** —— 确认 MCU 在汇报：

```bash
ros2 topic echo /navigation/motor/state
```

持续输出即串口通：

```
data: 0
---
data: 0
---
```

```bash
ros2 topic echo /navigation/motor/line_bits
```

用手指遮挡**最左边**的传感器，应从 `0` 变 `1`：

```
data: 1
---
data: 0
---
```

若遮挡最左反而让 bit3 变化，说明传感器左右装反了，应检查电机板 PB12..PB15 接线或传感器安装方向。

---

## 第 6 步：启动比赛

```bash
ros2 service call /navigation/start std_srvs/srv/Trigger {}
```

### 成功

```
response:
std_srvs.srv.Trigger_Response(success=True, message='started; waiting for route color and then STOP marker')
```

状态切到 `LINE_FOLLOW`，车开始循迹。用 `ros2 topic echo /navigation/state` 看完整流程：

```
LINE_FOLLOW → ROUTE_FOLLOW → WAIT_MOTOR_STOP → ARM_ACTION → COMPLETE
```

### 失败对照

失败时 `message` 会明确点出缺什么：

| message | 原因 | 处理 |
|---|---|---|
| `motor UART is not open` | 电机串口没打开 | 查端口名、`dialout` 权限、USB 线 |
| `servo UART is not open` | 外设串口没打开 | 同上 |
| `waiting for initial motor and peripheral status frames` | 已连上但没收到状态帧 | 查固件是否烧录、波特率是否 115200 |
| `waiting for vision roles 1 and 2; got [1]` | 只有一路视觉上线 | 另一路需配置 `SRC/role=2` |
| `already started: LINE_FOLLOW` | 重复启动 | 正常，流程只能启动一次 |
| `already started: FAULT` | 已进故障 | **必须重启节点**，不能靠再次 start 恢复 |

例如把 `servo_port` 打成不存在的 `/dev/ttyUSB9`，日志会持续刷：

```
[WARN] [...] motor UART unavailable: could not open port /dev/ttyUSB9: ...
```

调用 start 则返回 `servo UART is not open`。`Ctrl+C` 停掉、改对端口、重启即可。

---

## 常见错误与后果

### 错误 1：两个端口写成同一个

```bash
ros2 launch ... motor_port:=/dev/ttyUSB0 servo_port:=/dev/ttyUSB0   # ❌
```

日志看似正常：

```
motor UART connected: /dev/ttyUSB0
servo UART connected: /dev/ttyUSB0
```

实际症状：

- `/navigation/motor/state` 时有时无
- 间歇刷 `ignored motor status with invalid payload length`
- 间歇刷 `ignored peripheral status with invalid payload length`

原因：Linux 下 pyserial **允许同一 tty 被打开两次**，两个 `serial.Serial` 抢同一条字节流，互相读走对方的数据，CRC/长度校验失败后丢弃。

> **代码未做此校验**，YAML 注释里的「必须与电机 UART 为不同设备」只是人工约定，必须靠操作保证。

### 错误 2：用 `ros2 param set` 改端口（无效）

```bash
ros2 param set /robopioneers_navigation motor_port /dev/ttyUSB5
```

返回 `Set parameter successful`，`param get` 也显示 `/dev/ttyUSB5`。**但毫无作用**：

- `_open_serial_links()` 只在 `motor_serial is None` 时才重连
- 它读的是 `__init__` 里缓存进 `self.motor_port` 的属性值，不是参数服务器

节点不会重新打开串口，车仍用旧端口运行。改端口只能「改 YAML 重编译」或「launch 覆盖后重启」。

### 错误 3：串口编号漂移

`/dev/ttyUSB0`、`ttyUSB1` 的编号取决于枚举顺序，两台 MCU 同时上电时谁先谁后**不保证稳定**。今天对，明天换 USB 口就反了。

**推荐用 `by-id` 固定路径**：

```bash
ros2 launch robopioneers_navigation navigation.launch.py \
  motor_port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_2054376C3034-if00 \
  servo_port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_2063377A3034-if00
```

`by-id` 路径绑定芯片序列号，换 USB 口、重新插拔都不会变。

---

## 完整命令速查

```bash
# 1. 定位设备
ls -l /dev/serial/by-id/
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
groups | grep dialout

# 2. 编译
source /opt/ros/humble/setup.bash
cd ~/GPT_ws && colcon build --packages-select robopioneers_navigation
source install/setup.bash

# 3. 查看可用参数
ros2 launch robopioneers_navigation navigation.launch.py --show-args

# 4. 启动（端口对调）
ros2 launch robopioneers_navigation navigation.launch.py \
  motor_port:=/dev/ttyUSB1 \
  servo_port:=/dev/ttyUSB0

# 5. 验证参数（新终端）
ros2 param get /robopioneers_navigation motor_port
ros2 param get /robopioneers_navigation servo_port

# 6. 验证链路（新终端）
ros2 topic echo /navigation/motor/state
ros2 topic echo /navigation/motor/line_bits
ros2 topic echo /navigation/vision/heartbeat

# 7. 开赛
ros2 service call /navigation/start std_srvs/srv/Trigger {}

# 8. 监控
ros2 topic echo /navigation/state
ros2 topic echo /navigation/route
```

---

## 相关文档

| 文档 | 内容 |
|---|---|
| `README.md` | 包概览与最小启动示例 |
| `docs/MCU_INTERFACE.md` | 两套 UART 协议字段、状态值、停车门控 |
| `docs/PHONE_HOTSPOT_SETUP.md` | 手机热点组网与 UDP 联调 |
| `config/navigation.yaml` | 全部 26 个参数及中文注释 |
