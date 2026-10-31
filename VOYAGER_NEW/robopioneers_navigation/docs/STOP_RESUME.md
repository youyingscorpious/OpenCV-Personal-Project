# 暂停与恢复（stop / resume）操作步骤

本文说明如何用 `/navigation/stop` 和 `/navigation/resume` 让小车**受控停下并原地继续**，无需重启节点、不丢失赛程进度。

适用：Ubuntu 22.04 + ROS 2 Humble，`robopioneers_navigation` 含 stop/resume 服务版本。

---

## 一、它能解决什么问题

原有版本只有 `/navigation/start` 一个服务，想停车只能用 `Ctrl+C` 或拔线，而 `Ctrl+C` **并不发送停车指令**——`destroy_node()` 只关闭串口，车靠电机固件的 300 ms 指令看门狗自行停车，会多滑行最多 300 ms，并且流程彻底作废。

现在有了两个服务：

| 服务 | 作用 | 后续影响 |
|---|---|---|
| `/navigation/stop` | 立即零速并**持续保持零速**，进入 `PAUSED` | **不触发抓放**，可随时 `resume` |
| `/navigation/resume` | 回到暂停前的循迹阶段继续跑 | 无需重新握手视觉与 MCU |

与既有停车方式的对比：

| 方式 | 响应 | 发零速 | 流程是否作废 |
|---|---|---|---|
| **`/navigation/stop`** | 立即（本周期） | ✅ | ❌ 可恢复 |
| `/navigation/resume` 配合使用 | — | — | — |
| 遮挡四路传感器 | 立即 | ✅ | 是，0.35 s 后 `FAULT` |
| 视觉端发 STOP 帧 | ≤50 ms | ✅ | 否，但会**触发抓放序列** |
| `Ctrl+C` | ~300 ms | ❌ 靠固件看门狗 | 是（需重启） |
| 拔电机 USB | 立即 | ❌ 靠固件看门狗 | 是 |

---

## 二、状态机变化

```
WAIT_START
    │ /navigation/start
    ▼
LINE_FOLLOW ⇄ /navigation/stop → [PAUSED] → /navigation/resume → 回到原阶段
    │                                 │
    │ 识别到色卡                       │ （PAUSED 期间每周期持续刷新零速帧）
    ▼                                 │
ROUTE_FOLLOW ⇄ /navigation/stop ──────┘
    │
    │ 视觉 STOP
    ▼
WAIT_MOTOR_STOP → ARM_ACTION → COMPLETE
```

`PAUSED` **只挂起，不结束**：`started` 仍为 `True`，视觉角色握手、MCU 状态帧、路线选择结果（`selected_route`）全部保留，因此恢复后不需要重新等待任何握手。

---

## 三、关键设计：为什么暂停时要一直发零速

这是本功能最容易踩坑的地方，必须理解。

电机固件有一个 **300 ms 指令看门狗**：持续运行时超过 300 ms 没收到 `CMD_CHASSIS`（`0x01`），固件会自行停车并把状态位置为 `5 CMD_TIMEOUT`。

而节点在 [navigation_node.py:573](C:\Users\39410\Desktop\ws_all\GPT_ws\src\robopioneers_navigation\robopioneers_navigation\navigation_node.py#L573) 附近有这段逻辑：

```python
if status[2] in (STATE_FAULT, STATE_TIMEOUT) and self.started:
    self._fault(f'motor controller reported state={status[2]}')
```

**如果暂停时简单地"停止发送速度帧"，那么 300 ms 后固件就会报 `CMD_TIMEOUT`，节点立即进入 `FAULT`——暂停会变成一个不可恢复的故障。**

因此实现上，`_update_line_follow()` 增加了一个专门的暂停分支，**每个 20 ms 周期继续下发零速帧**：

```python
def _update_line_follow(self, now):
    # 暂停期间必须持续下发零速，否则电机固件 300 ms 看门狗会停车并报 CMD_TIMEOUT。
    if self.state == 'PAUSED':
        if not self._send_wheel_speeds(0, 0):
            self._fault('failed to refresh zero-speed command while paused')
        return
    ...
```

这样既喂了看门狗、保持车辆静止，又不会像 `WAIT_MOTOR_STOP` 那样进入抓放序列。

---

## 四、暂停期间仍然生效的保护

暂停时**不会**因为下列原因误判故障——三个看门狗都有状态门控：

| 检查 | 门控条件 | 暂停时 |
|---|---|---|
| `_check_vision_timeout` | 仅 `LINE_FOLLOW`/`ROUTE_FOLLOW` | ✅ 不触发，视觉断线不会让暂停变故障 |
| `_check_stop_timeout` | 仅 `WAIT_MOTOR_STOP` | ✅ 不触发 |
| `_check_arm_timeout` | 仅 `ARM_ACTION` | ✅ 不触发 |

但**仍然**可能进入 `FAULT` 的情况（都属于真实链路故障，应当报错）：

- 电机串口写入零速帧失败（如 USB 被拔出）
- 电机板状态帧报告 `STATE_FAULT`(4) 或 `STATE_TIMEOUT`(5)
- 外设板报告 `STATE_FAULT`(4)

---

## 五、操作步骤

### 步骤 1：编译

```bash
source /opt/ros/humble/setup.bash
cd ~/GPT_ws
colcon build --packages-select robopioneers_navigation
source install/setup.bash
```

### 步骤 2：启动并开赛

```bash
ros2 launch robopioneers_navigation navigation.launch.py \
  motor_port:=/dev/ttyUSB0 \
  servo_port:=/dev/ttyUSB1
```

链路齐全（两串口 + 双视觉角色）后开赛：

```bash
ros2 service call /navigation/start std_srvs/srv/Trigger {}
```

确认状态：

```bash
ros2 topic echo /navigation/state
```

期望出现 `LINE_FOLLOW` 或 `ROUTE_FOLLOW`。

### 步骤 3：暂停

```bash
ros2 service call /navigation/stop std_srvs/srv/Trigger {}
```

成功返回：

```
response:
std_srvs.srv.Trigger_Response(success=True, message='paused from LINE_FOLLOW; call /navigation/resume to continue')
```

节点日志同时出现：

```
[navigation state=PAUSED: paused from LINE_FOLLOW; holding zero wheel speed]
```

车辆在当前 20 ms 周期内收到零速，之后持续保持零速。

### 步骤 4：暂停期间监控

```bash
ros2 topic echo /navigation/state          # 持续显示 PAUSED
ros2 topic echo /navigation/motor/state    # 应为 0 (IDLE)，不应出现 5 (CMD_TIMEOUT)
```

**验收要点**：`/navigation/motor/state` 必须稳定为 `0`。如果变成 `5`，说明零速帧没有持续刷新，暂停逻辑失效。

想确认车辆确实在静止，可对比左右轮实测速度（状态帧里带）：电机状态应保持 `IDLE`。

### 步骤 5：恢复

```bash
ros2 service call /navigation/resume std_srvs/srv/Trigger {}
```

成功返回：

```
std_srvs.srv.Trigger_Response(success=True, message='resumed to LINE_FOLLOW')
```

日志：

```
[navigation state=LINE_FOLLOW: resumed from PAUSED; waiting for fresh line sensor data]
```

下一个 20 ms 周期即恢复循迹输出。

### 步骤 6：可重复暂停

暂停与恢复可任意次交替，`started`、视觉握手、`selected_route` 都保留：

```bash
ros2 service call /navigation/stop std_srvs/srv/Trigger {}
ros2 service call /navigation/resume std_srvs/srv/Trigger {}
# ... 反复执行
```

重复调用 `stop` 是幂等的，返回 `already paused from LINE_FOLLOW`，不会出错。

---

## 六、恢复的前置条件（重要）

`/navigation/resume` **不是无条件执行**的。它会先检查三个链路是否新鲜，任意一项不满足就拒绝恢复、并保持 `PAUSED`：

| 检查项 | 阈值 | 失败返回 |
|---|---|---|
| 相机 1 目标列表 | `vision_source_timeout_seconds` = 0.6 s | `vision target list stale (1.23s); stay paused` |
| 电机板底盘遥测 | `0x11` 状态帧需在 0.35 s 内更新 | `motor chassis telemetry stale; stay paused` |
| 电机串口 | 必须已打开 | `motor UART is not open` |

**设计意图**：避免在视觉或循迹已失效时放开行驶，导致车辆失控冲出去。这也是为什么暂停时间不能太长——如果暂停期间视觉板掉线，恢复会一直被拒绝。

拒绝恢复时**保持暂停是正常行为**，不是 bug。此时应当：

```bash
ros2 topic echo /navigation/vision/target_list   # 检查视觉是否恢复推送
ros2 topic echo /navigation/motor/line_bits       # 检查电机板循迹遥测是否恢复
```

等问题排除后再调 `resume`。

---

## 七、失败返回对照表

### `/navigation/stop`

| message | 原因 | 处理 |
|---|---|---|
| `not started: WAIT_START` | 流程尚未启动 | 先调 `/navigation/start` |
| `cannot pause in state FAULT; only LINE_FOLLOW/ROUTE_FOLLOW` | 已进故障 | 必须重启节点，故障不可恢复 |
| `cannot pause in state WAIT_MOTOR_STOP; only LINE_FOLLOW/ROUTE_FOLLOW` | 正在停稳确认 | 等待流程走完，或接受它继续抓放 |
| `cannot pause in state ARM_ACTION; only LINE_FOLLOW/ROUTE_FOLLOW` | 机械臂动作中 | 同上，不能打断动作序列 |
| `cannot pause in state COMPLETE; only LINE_FOLLOW/ROUTE_FOLLOW` | 任务已完成 | 无意义 |
| `motor UART is not open` | 电机串口未连接 | 查 USB 与 `dialout` 权限 |
| `failed to send zero speed; entered FAULT` | 零速帧写入失败 | 查串口，随后必须重启节点 |

### `/navigation/resume`

| message | 原因 | 处理 |
|---|---|---|
| `not paused: LINE_FOLLOW` | 当前不在暂停状态 | 先调 `/navigation/stop` |
| `not paused: FAULT` | 已进故障 | 重启节点 |
| `unknown pause origin: None` | 暂停来源记录异常 | 重启节点 |
| `vision target list stale (...)s; stay paused` | 相机 1 数据过期 | 检查视觉端 |
| `peripheral line-sensor status stale; stay paused` | 循迹帧过期 | 检查外设板串口 |
| `motor UART is not open` | 电机串口断开 | 检查 USB |

---

## 八、为什么暂停只允许在行驶阶段

代码把暂停限制在 `LINE_FOLLOW` 和 `ROUTE_FOLLOW` 两个阶段：

```python
if self.state not in ('LINE_FOLLOW', 'ROUTE_FOLLOW'):
    response.success = False
    response.message = f'cannot pause in state {self.state}; only LINE_FOLLOW/ROUTE_FOLLOW'
```

原因是另外几个阶段都有**正在进行中的时序约束**，强行插入暂停会破坏安全逻辑：

- **`WAIT_MOTOR_STOP`**：正在等编码器实测零速（连续 3 帧 + 持续 0.5 s）。暂停会打断计数，且该阶段本身就在停车，暂停无意义。
- **`ARM_ACTION`**：要求观测到 `ARM_BUSY` 再收到 `ARM_DONE` 才能推进。中途暂停会让 `arm_action_timeout_seconds`（15 s）超时，直接 `FAULT`。
- **`COMPLETE` / `FAULT`**：终态，无可暂停的内容。

**想在中途急停，应该用 `Ctrl+C` 或断电，而不是 `stop`。**

---

## 九、与固件的关系（是否需要电控配合）

**不需要修改电机或外设固件。** 本功能完全在 ROS 侧实现，使用的都是固件**已经支持**的命令：

| 用到的命令 | 是否新增 |
|---|---|
| `CMD_CHASSIS` (`0x01`) 发送左右轮 0 速 | ❌ 已有，原本就在用 |
| `CMD_STATUS` (`0x10`) 读取状态判断停稳/故障 | ❌ 已有 |
| `CMD_ESTOP` (`0x04`) 故障时急停 | ❌ 已有 |

没有任何新增命令字、没有改动帧格式、没有要求固件新增状态值。对电控端是**零改动**。

唯一需要电控端知晓的是**行为变化**：

- 暂停期间电机板会**持续收到零速帧**（而不是像 `Ctrl+C` 那样断流），所以状态位会稳定停在 `0 IDLE`，不会进入 `5 CMD_TIMEOUT`。
- 如果电控端有上位机监控逻辑，看到 `IDLE` 且速度为零时，属于**正常的暂停状态**，不是掉线。

> ⚠️ 反过来说：如果哪天发现暂停时电机报 `CMD_TIMEOUT`，那是 ROS 侧的 bug（零速帧没刷新），不要去改固件看门狗。

---

## 十、新增接口汇总

| 接口 | 类型 | 说明 |
|---|---|---|
| `/navigation/stop` | `std_srvs/srv/Trigger` | 受控暂停，保持零速，不触发抓放，可恢复 |
| `/navigation/resume` | `std_srvs/srv/Trigger` | 从 `PAUSED` 回到暂停前阶段 |

`/navigation/state` 的 JSON 中会新增 `PAUSED` 状态值。

---

## 十一、命令速查

```bash
# 开赛
ros2 service call /navigation/start std_srvs/srv/Trigger {}

# 暂停
ros2 service call /navigation/stop std_srvs/srv/Trigger {}

# 查看状态（应为 PAUSED）
ros2 topic echo /navigation/state

# 确认电机空闲且未超时（应为 0）
ros2 topic echo /navigation/motor/state

# 恢复
ros2 service call /navigation/resume std_srvs/srv/Trigger {}

# 暂停后视觉异常时，先查这两条再恢复
ros2 topic echo /navigation/vision/target_list
ros2 topic echo /navigation/motor/line_bits

# 列出所有可用服务确认接口存在
ros2 service list | grep navigation
ros2 service type /navigation/stop
```

---

## 十二、相关文档

| 文档 | 内容 |
|---|---|
| `docs/QUICK_START.md` | 从零启动的完整实例（串口覆盖、验收、开赛） |
| `docs/MCU_INTERFACE.md` | 两套 UART 协议字段、状态值、停车门控 |
| `docs/PHONE_HOTSPOT_SETUP.md` | 手机热点组网与 UDP 联调 |
| `config/navigation.yaml` | 全部参数及中文注释（含恢复用到的两个超时阈值） |
