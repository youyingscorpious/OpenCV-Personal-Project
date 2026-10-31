# voyager_gate · 路口「停车 → 色卡识别」闸门

可直接 `colcon build` 的 ROS 2 (Humble) 功能包。实现方案 A：**纯导航端闸门，零协议改动**。

---

## 1. 它解决什么

赛程阶段二要求「**停在三岔路口 → 识别色卡 → 驶入对应支路**」。改造前的链路是：

```
电机循迹完成 ─▶ 导航发零速 ─▶ 视觉一直在推颜色 ─▶ 拿到就用 ─▶ 转向
                                        ↑
                          用的是「整个赛程第一次见到的颜色」
```

本包插入三道闸门：

```
电机循迹完成 ─▶ ①导航→电控 停车 ─▶ ②停稳证据 ─▶ ③识别窗口 ─▶ ④多帧投票 ─▶ 转向
```

因此**行进期间的颜色帧一律丢弃**，只有「确认停稳之后」的识别结果才可能决定支路。

---

## 2. 继承链（原件零修改）

```
robopioneers_navigation.NavigationNode          ← 原件，只读，不复制不改写
  └── CompetitionNode  (voyager_gate/competition_base.py)   ← 模板副本：定角转向 + STOP 定距
        └── VoyagerNode (voyager_gate/voyager_node.py)      ← 本包：闸门接线
```

`competition_base.py` 由 `robopioneers_competition/.../route_controller.py` **原样复制**，
只改了文件头注释、删掉了自带的 `main()`。这样做的好处：

- 本包只依赖**真正的原件** `robopioneers_navigation`；
- 不依赖 `robopioneers_competition`（两者可独立编译、互不干扰，也可只装其一）。

---

## 3. 构建

```bash
# 放进同一个工作空间
cp -r robopioneers_navigation  ~/ws/src/     # 原件，未改动
cp -r VOYAGER_NEW/voyager_gate ~/ws/src/

cd ~/ws
colcon build --packages-select robopioneers_navigation voyager_gate
source install/setup.bash

# 离线算法自检（不需要硬件 / ROS 图）
python3 src/voyager_gate/tools/gate_selftest.py
#   期望：13/13 checks passed / ALL SCENARIOS PASSED
```

依赖：`ros-humble-rclpy`、`ros-humble-std-msgs`、`ros-humble-std-srvs`、`python3-serial`。

---

## 4. 运行

### 4.1 上车（默认：板端直发视觉帧）

```bash
# PC 连板子热点 RoboVision_2027 / 12345678
ros2 launch voyager_gate voyager_gate.launch.py

# 确认视觉已到
ros2 topic echo /navigation/vision/color

# 开赛
ros2 service call /navigation/start std_srvs/srv/Trigger {}
```

> ⚠️ **不要**同时启动 `navigation.launch.py` 或 `competition.launch.py`：
> 三者抢同一组串口（ttyUSB0/1）和 UDP 5000。

### 4.2 台架（无相机，手工注入颜色）

```bash
ros2 launch voyager_gate voyager_gate.launch.py \
  start_vision_bridge:=true vision_source:=manual vision_module_path:=''

ros2 topic pub --once /vision_test/color std_msgs/msg/UInt8 "{data: 1}"
ros2 topic pub --once /vision_test/stop  std_msgs/msg/Bool  "{data: true}"
```

### 4.3 只想退回改造前

```bash
ros2 launch voyager_gate voyager_gate.launch.py \
  junction_gate_enabled:=false route_color_vote_frames:=1
```

---

## 5. 观测

```bash
ros2 topic echo /navigation/competition/status      # 相位 JSON
ros2 topic echo /navigation/junction/color_request  # true = 识别窗口已开（std_msgs/Bool）
ros2 topic echo /navigation/state
```

`/navigation/competition/status` 的 `phase` 应依次出现：

```
JUNCTION_PARKED → JUNCTION_STOP_CONFIRMED → COLOR_WINDOW_OPEN
                → ROUTE_COLOR_READY → ROUTE_TURN_BEGIN
                → ROUTE_TURN_REACHED → ROUTE_FOLLOW
```

出现 `COLOR_REJECTED` 是**正常**的：说明闸门确实丢掉了停稳前的颜色帧。

---

## 6. 参数

写在 `config/voyager_gate.yaml`，也可用 `--ros-args -p` 覆盖。

| 参数 | 默认 | 语义 |
|---|---|---|
| `junction_gate_enabled` | `true` | 闸门总开关 |
| `junction_stop_confirm_samples` | `3` | 连续零速帧数下界（0x10 约 100 ms/帧） |
| `junction_stop_confirm_dwell_s` | `0.25` | 停稳持续时间下界（s） |
| `junction_stop_confirm_timeout_s` | `2.0` | 停稳确认等待上界（s） |
| `junction_stop_confirm_timeout_action` | `proceed` | `proceed` 强制开窗 / `fault` 进 FAULT |
| `route_color_require_after_stop` | `true` | 只采信停稳之后的颜色帧 |
| `route_color_vote_frames` | `3` | 锁定所需连续同色帧数 |
| `route_color_vote_timeout_s` | `0.6` | 投票有效期（s） |
| `route_color_reject_log_hz` | `2.0` | 丢弃告警限频（Hz） |

其余参数与 `robopioneers_competition` 完全一致（`route_angle_*_deg`、`route_heading_sign`、
`stop_delay_mm`、串口与视觉链路等），标定方法见
[`../04_验证与标定.md`](../04_验证与标定.md)。

---

## 7. 目录

```
voyager_gate/
├── package.xml
├── setup.py / setup.cfg
├── resource/voyager_gate
├── config/voyager_gate.yaml        # 参数（通信参数与原包逐项一致 + 9 个闸门参数）
├── launch/voyager_gate.launch.py   # 一键启动（可选 PC 端视觉桥接）
├── tools/gate_selftest.py          # 离线自检，6 场景 13 断言，无需 ROS
└── voyager_gate/
    ├── __init__.py
    ├── junction_gate.py            # 闸门判据（停稳确认 / 识别窗口 / 颜色投票）
    ├── competition_base.py         # 模板副本：定角转向 + STOP 定距停车
    └── voyager_node.py             # 接线后的可执行节点（唯一入口）
```

---

## 8. 设计文档

算法推导、不变量、时序预算、降级矩阵在上一级目录：

- [`../01_算法设计.md`](../01_算法设计.md)
- [`../02_改动清单.md`](../02_改动清单.md)（本包已按该清单实现，无需再手工改）
- [`../04_验证与标定.md`](../04_验证与标定.md)
- [`../05_风险与回退.md`](../05_风险与回退.md)
