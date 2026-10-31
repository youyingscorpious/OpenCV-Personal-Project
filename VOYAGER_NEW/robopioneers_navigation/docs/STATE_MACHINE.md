# 导航状态机说明

本包继续使用 Ubuntu 22.04 / ROS 2 Humble 的 Python `rclpy` 节点。此文档描述导航节点内部状态组织；不引入 SMACC2，也不改变 ROS、UART、视觉网络、消息协议、参数或服务接口。

## 工作方式

导航节点用 `self.state` 保存当前状态。`TRANSITIONS` 是 `(当前状态, 事件) -> 目标状态` 的规则表，不负责保存/封装状态；`_dispatch(event, detail)` 查表并执行状态转移。状态进入和退出时分别调用 `_on_enter()`、`_on_exit()`。串口收发、视觉帧解析、20 ms 轮询及 ROS 服务仍由原来的 `NavigationNode` 负责。

`PAUSED -> RESUME` 的目标为 `None`，代表恢复到 `paused_from_state`。派发器通过键存在性检查转移，避免把这个合法的 `None` 当作“没有规则”。通配规则 `('*', 'FAULT')` 允许任意状态进入故障状态。

## 主要转移

| 当前状态 | 事件 | 目标状态 | 触发来源/动作 |
|---|---|---|---|
| `WAIT_START` | `START` | `LINE_FOLLOW` | `/navigation/start` 服务通过链路与视觉就绪检查后触发 |
| `LINE_FOLLOW` | `ROUTE_SELECTED` | `ROUTE_FOLLOW` | 收到有效路线颜色；路线控制能力仍受当前电机固件接口限制 |
| `LINE_FOLLOW` / `ROUTE_FOLLOW` | `PAUSE` | `PAUSED` | `/navigation/stop` 先发送零速，再进入暂停并持续刷新零速 |
| `PAUSED` | `RESUME` | 暂停前状态 | `/navigation/resume` 检查数据新鲜度后触发，随后重新启用 `CMD_LINE` |
| `LINE_FOLLOW` / `ROUTE_FOLLOW` | `JUNCTION_DETECTED` | `WAIT_ROUTE_MANEUVER` | 电机状态或底盘遥测检测到路口；进入状态时补发零速 |
| `LINE_FOLLOW` / `ROUTE_FOLLOW` / `WAIT_ROUTE_MANEUVER` | `STOP_MARKER` | `WAIT_MOTOR_STOP` | 视觉检测 STOP 后发零速并开始停稳确认 |
| `WAIT_MOTOR_STOP` | `MOTOR_ZERO` | `ARM_ACTION` | 编码器速度连续满足阈值和停稳时长后触发 |
| `ARM_ACTION` | `ARM_SEQUENCE_DONE` | `COMPLETE` | 抓取、放置、复位序列完成 |
| 任意状态 | `FAULT` | `FAULT` | UART、视觉、电机或动作异常触发；随后发送现有急停帧 |

## 安全要点

- 状态切换本身不会证明车辆已停稳。机械臂仍须等待电机状态帧中的编码器实测速度连续满足样本数、速度阈值和持续时间条件。
- 暂停和 STOP 流程继续发送现有 `CMD_CHASSIS` 零速；进入循迹状态后由原轮询重新发送 `CMD_LINE`，并按原周期发送 `CMD_PING`。
- 路口分支选择接口仍未由当前电机固件提供；收到路口事件后维持停车等待，不会凭颜色猜测转向。
- `TRANSITIONS` 是本包 Python 内部规则表，不是 ROS 2 内置 API，也不是 SMACC2 的正交模块。串口、视觉和定时器仍未拆成独立 orthogonal。

## 不变接口

ROS 2 发行版仍为 Humble，系统仍为 Ubuntu 22.04。`/navigation/start`、`/navigation/stop`、`/navigation/resume` 三个 Trigger 服务、`/navigation/state` 话题、两路 UART 及视觉传输协议和配置均保持原样。