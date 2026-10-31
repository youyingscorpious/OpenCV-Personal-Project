# VOYAGER_NEW2 · 在原闸门包上增量接入同进程视觉

> 本目录是 `VOYAGER_NEW` 的**副本 + 增量**。原 `VOYAGER_NEW/` 一个字节都没改。
> 本文档经三轮代码 review 后重写，把每一轮发现的硬阻塞、修法和残留风险全部写明。

## 一、和原版的差异

| 项 | VOYAGER_NEW（原版） | VOYAGER_NEW2（本目录） |
|---|---|---|
| 视觉来源 | 板端识别 → UDP 5000 发 AA55 编号帧 | 板端发 JPEG → PC 进程内 `nav_vision` 识别 |
| 视觉进程 | 无（板端在另一块板上算） | 同进程后台 daemon 线程（`pc_receiver` 收 UDP 5002） |
| 新增文件 | — | `voyager_gate/voyager_gate/vision_adapter.py` |
| 修改文件 | — | `voyager_node.py`、`launch`、`yaml`、`package.xml`、`__init__.py` |
| 闸门 G0–G4 | — | **一行没动**（`junction_gate.py` 字节级保留） |
| 定角转向 / STOP 定距 | — | **一行没动**（`competition_base.py` 字节级保留） |
| 默认行为 | — | `vision_inline_enabled=false`，与原版逐字段一致 |

## 二、三轮 review 修了什么

### 第一轮：三个硬阻塞（B1/B2/F1）

对照父类 `navigation_node.py` 源码发现：

| 问题 | 现象 | 修法 |
|---|---|---|
| **B1** `/navigation/start` 拒赛 | 父类 `:278` 要求 `vision_roles == {1,2}`，inline 不发 AA55 VERSION 帧 → 永远 `got []` | `_vision_init` 末尾 `vision_roles={1,2}; vision_enabled=True` |
| **B2** 开赛后第一个 tick 就 FAULT | 父类 `:1113` 要求 `vision_last_any` 1.5s / `vision_last_seen[(1,0x01)]` 0.6s 内刷新，inline 不发 AA55 → 恒为 0 | 每 30ms tick 喂这两个时间戳 |
| **F1** 多帧投票被架空 | `nav_vision` 结果槽 0.5s 保鲜，30ms 轮询把同一帧读 16 次，3 票在 90ms 内凑齐 | 按 `nv.status()['last_color_age_ms']` 边沿触发：age 变小才算新帧 |

### 第二轮：六个新问题（P1–P6）

修好 B1/B2/F1 后，离线仿真又发现：

| 问题 | 现象 | 修法 |
|---|---|---|
| **P1** 启动顺序锁死 ⚠️ | `nv.start()` 在节点构造时就置 `_started_ts`，板端 2s 内没出图 → `color()="fuck"` → 5 个 tick 后 `_fault()` → `_fault` 把 `started=True`（`:1133`）→ 之后 `/navigation/start` 永远返回 `already started: FAULT`，只能重启节点 | `color()=="fuck"` 且 `not self.started` 时不计数、不 fault，只把 `color_raw` 压成 `"0"`；未开赛前车不会动，跳过没有安全损失 |
| **P2** `_fault` 不 latch | 计数器只增不减，条件一旦成立永远成立 → 33Hz 反复 `_fault()`，每帧给两个串口发 `CMD_ESTOP`、反复 `_dispatch('FAULT')` | fault 条件加 `and self.state != 'FAULT'`，只 fault 一次 |
| **P3** 错误提示误导 | import 失败时提示"回退到原 UDP 5000 链路"，但本部署板端只发 JPEG，根本没有这条链路；且 B1 seed 也不跑，表现像板子没上电 | 文案改成实话：没有后备链路，此时 `/navigation/start` 会因 `vision_roles` 为空被拒 |
| **P4** `_reset_vision_session()` 清掉 seed | UDP `OSError` 时父类会 `clear()` `vision_roles`，而 B1 seed 只在 `_vision_init` 跑过一次 | 每 tick 重新断言 `if vision_roles != {1,2}: vision_roles = {1,2}` |
| **P5** `status()` 异常 → 投票失效 | `color_age_ms` 取不到时走"新帧"分支，退化成每 tick 一票，F1 保护消失 | `color_age_ms is None` 时 `is_new_color_frame = False`，宁可晚锁不要乱锁 |
| **P6** `vision_stream_lost_fault: 0` 现在危险 | B2 胶水让父类 1.5s/0.6s 看门狗永久失效，0 = 相机全死也不停车 | **不要设 0**；保持默认 5，或至少 3 |

## 三、新增的 6 个参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `vision_inline_enabled` | `false` | 总开关。true = 本进程内 import nav_vision |
| `vision_module_path` | `''` | `nav_vision.py` 所在目录（launch 里叫 `vision_inline_module_path`） |
| `vision_poll_period_s` | `0.03` | 轮询周期 |
| `vision_stream_lost_fault` | `5` | `color()` 连续 `"fuck"` 几次后进 FAULT。**不要设 0**（见 P6） |
| `vision_suppress_color_on_stop` | `true` | `stop()=="1"` 时屏蔽当帧 color，防红色 STOP 牌误判红卡 |
| `vision_status_log_hz` | `1.0` | VISION_STATUS 诊断发布频率 |

## 四、怎么跑

### 1. 离线闸门自检

```bash
python3 VOYAGER_NEW2/voyager_gate/tools/gate_selftest.py
# 期望：13/13 checks passed / ALL SCENARIOS PASSED
```

注意：这个自检**不 import vision_adapter**，通过只说明闸门没坏，不证明 inline 链路通。

### 2. 仍然用原方案（板端直发编号）

```bash
ros2 launch voyager_gate voyager_gate.launch.py
```

### 3. 启用同进程视觉

```bash
ros2 launch voyager_gate voyager_gate.launch.py \
    vision_inline_enabled:=true \
    vision_inline_module_path:=/home/scurry/Desktop/VOYAGER_WS/VISION/VISION_WS_doc/src
```

板端跑 `board_main.py`（只发 CSI2 JPEG 到 UDP 5002）。PC 端 `pip install opencv-python numpy`。
**同一台 PC 不要同时跑 `main.py` 调试**（UDP 5001/5002 会被抢，且 `pc_receiver` 没设 `SO_REUSEADDR`，inline 和 bridge 不能共存）。

### 4. 观测

```bash
ros2 topic echo /navigation/competition/status
```

inline 模式每秒一条：
```
VISION_STATUS  fps=25.0 frame_age=40ms color=1 stop=0 err=None
```

## 五、调用链

```
庐山派 CSI2 640×480 JPEG
   │ UDP 5002
   ▼
nav_vision 后台线程（本进程内，daemon）
   ├─ color_core → slot.set("1"/"2"/"3"/"0")
   └─ stop_sign_core → slot.set("1"/"0")
   │
   │  30 ms 定时器 _vision_tick()
   │  ├─ B2：vision_last_any / vision_last_seen[(1,0x01)] = now
   │  ├─ P4：每 tick 重断言 vision_roles == {1,2}
   │  ├─ P1：not started 时 fuck 不计数（板端开机慢不算故障）
   │  ├─ P2：state==FAULT 后不再重复 fault
   │  ├─ stop 边沿触发 → _request_motor_stop()
   │  └─ F1+P5：age 变小 且 age 非 None 才算新帧
   ▼
nv.color() == "1"/"2"/"3" 且 is_new_color_frame
   → self._select_route(int(c))
        ↓ 共用闸门投票（I2/I3 不变量仍生效）
        ↓ 连续 3 次独立识别同色 → selected_route = c
```

## 六、残留风险（上车前必读）

1. **胶水是在和父类看门狗对话**：B1/B2/P4 的字段写入绕过了 AA55 协议握手。
   如果日后父类升级加了新的握手校验，这里要跟着加。嫌脏就用退路（见下）。
2. **退路：直接用 vision_bridge**。原包 `robopioneers_competition` 已实现完整的
   AA55 桥（VERSION 握手 + role2 shim + TARGET_LIST 推送），一行胶水都不用写：
   ```bash
   ros2 launch voyager_gate voyager_gate.launch.py \
       start_vision_bridge:=true vision_source:=nav_vision \
       vision_module_path:=/home/robot/ws/src/vision_nav/导航用的
   ```
   这时 `vision_inline_enabled` 保持 false，本包这层 adapter 完全不工作。
   代价是多一个节点进程，但 B1/B2 天然满足；F1 要在 bridge 侧按帧去重。
3. **启动顺序**：补了 P1 之后，先 `ros2 launch` 后上电也不会锁死了（未开赛不计数）。
   但仍建议先让板端出图、`VISION_STATUS` 心跳稳定后再 `/navigation/start`。
4. **两个路径参数别填错**：launch 参数 `vision_inline_module_path`（给 inline）
   vs `vision_module_path`（给 bridge），名字太像。填错的表现是 import 失败 +
   `/navigation/start` 返回 `got []`。
5. **红色 STOP 牌 ≠ 红色色卡**：`suppress_color_on_stop=true` 已在 stop=="1"
   的当帧屏蔽 color，但 STOP 牌刚消失的几帧仍可能漏出红卡。
6. **`vision_stream_lost_fault` 不要设 0**（P6）：B2 胶水让父类视觉看门狗永久
   失效，0 = 相机全死也不停车。保持默认 5。
7. **13/13 自检不算 inline 的证据**：`gate_selftest.py` 不 import `vision_adapter`。
   inline 链路必须上车台架看到 `VISION_STATUS` 心跳和 `ROUTE_COLOR_READY` 才算通。
