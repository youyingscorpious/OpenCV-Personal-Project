# vision_nav — 三岔路口色卡 + STOP 牌（PC 端）

庐山派只负责采集和发图；PC 端用 `pc_receiver` 收图，两个识别模块共用 CSI1 一路画面。
导航组只 import `nav_vision`，同进程调用，不走 UDP。

```
庐山派 board_main.py ──UDP 5002──> PC pc_receiver ──> nav_vision
   CSI1 前视 640x480 q75                                ├─ color_core   → 1/2/3
                                                        └─ stop_sign_core → 1/0
```

## 文件

| 文件 | 作用 |
|---|---|
| `nav_vision.py` | 导航组唯一入口：`color()` / `stop()` / `status()` |
| `color_core.py` | 三岔路口色卡核心（从 `三岔路口.py` 抽出，已重标 640×480） |
| `stop_sign_core.py` | STOP 牌核心，原样复制 |
| `pc_receiver.py` | 板端 UDP 图像流接收 API，原样复制 |
| `main.py` | PC 调试入口：单窗口同时画色卡框和 STOP 框 |
| `board_main.py` | 板端副本，默认只开 CSI1 |
| `selftest.py` | 无硬件自测：合成色卡 / STOP 牌，验证两个核心和字符串契约 |

## 一、板端

把 `board_main.py` 复制到板子 `/sdcard/`（或 CanMV IDE 直接运行），改顶部：

```python
WIFI_SSID = "你的热点名称"
WIFI_PASSWORD = "你的热点密码"

CSI_PNP = 1        # 逻辑通道 1 -> 物理 CSI1
CSI_COLOR = 2      # 逻辑通道 2 -> 物理 CSI2（默认关闭）

ENABLE_CSI1 = True    # 默认只用 CSI1：color + STOP
ENABLE_CSI2 = False   # CSI2 留给 PnP/备用
```

需要 PnP/第二路时，把 `ENABLE_CSI2` 改 `True`，并让逻辑通道 2 指向物理 CSI2；
PC 端同时改用通道 2。

正常日志：

```
[board] cameras c1=on c2=off, 640x480, q1=75 q2=75
[board] CSI1 configured (640x480)
[board] c1 JPEG self-test ok: 3xxxx bytes
[board] waiting for PC HELLO ...
[board] PC registered: ('192.168.x.y', 5002)
```

Windows 防火墙放行 UDP 5001/5002（管理员 PowerShell）：

```powershell
New-NetFirewallRule -DisplayName "K230 UDP 5001/5002" -Direction Inbound `
  -Protocol UDP -LocalPort 5001,5002 -Action Allow -Profile Any
```

## 二、PC 依赖

```bash
pip install opencv-python numpy
```

## 三、调试

```bash
python main.py                                  # 读板端 ch1
python main.py --board-ip 192.168.209.193       # 收不到广播时直连
python main.py --source 0                       # 本地摄像头
python main.py --source test.mp4                # 视频
python main.py --no-gui                         # Ubuntu Server / 无 X
python main.py --debug                          # 画 ROI 和未确认候选
```

单窗口里同时显示色卡框和 STOP 框，`q` 或 ESC 退出。

无硬件自测（合成图，不需要板子）：

```bash
uv run --with opencv-python --with numpy python selftest.py
```

## 四、导航组接入

把 `vision_nav` 目录加入 `sys.path`，然后：

```python
import nav_vision as nv

a = nv.color()      # "1" 红 / "2" 黄 / "3" 蓝 / "0" 无 / "fuck" 故障
b = nv.stop()       # "1" 有 STOP / "0" 无 / "fuck" 故障
```

首次调用 `color()` 只激活色卡检测；首次调用 `stop()` 只激活 STOP 检测；
`nv.start()` 可以一次激活两者。

### 广播连不上时：单播扫描（默认）

手机热点、AP 隔离、虚拟机 NAT、防火墙都可能过滤 UDP 广播，但单播通常没问题。
`nav_vision` 在没有 `board_ip` 时默认对 PC 所在 /24 做单播扫描：

- 每轮向默认网段（本机 /24 + `192.168.145.0/24`）的各地址发一个 `HELLO`；
- 板子收到后开始推流；
- PC 从第一个视频/CALR 包的源地址学习板子 IP，然后停止扫描。

默认 `NAV_VISION_SCAN_SUBNET=auto,192.168.145.0/24`，会同时扫 PC 当前 /24
和当前热点的 `192.168.145.0/24`，自动去重。PC 有 VPN/多网卡时 `auto` 可能选错，
但显式的 `192.168.145.0/24` 仍能兜住。**换热点后前缀不同时**，把新前缀加进去：

```bash
export NAV_VISION_SCAN_SUBNET=auto,192.168.137.0/24
```

也可以只写一个网段：

```bash
export NAV_VISION_SCAN_SUBNET=192.168.145.0/24
```

关闭扫描：`NAV_VISION_SCAN_SUBNET=off`。`main.py` / `pc_receiver.py` 的
`--scan-subnet` 默认也是 `auto,192.168.145.0/24`。

### 直接指定板子 IP（兜底）

如果扫描找不到，或者你知道板子 IP，在首次 `color()` / `stop()` / `start()` 之前：

```python
import nav_vision as nv

nv.set_board_ip("192.168.145.222")   # 你的板子 IP
a = nv.color()
b = nv.stop()
```

也可以提前设置环境变量：

```bash
export NAV_VISION_BOARD_IP=192.168.145.222      # Linux / macOS
set NAV_VISION_BOARD_IP=192.168.145.222         # Windows cmd
```

`main.py` 对应参数是 `--board-ip 192.168.145.222`。

直连模式只要求板子在 IP 层可达，不依赖广播。`set_board_ip()` 必须在第一次
`color()/stop()/start()` 之前调用；运行中要换 IP 先 `nv.close()`。

广播排查：

- PC 和板子是否同网段（`192.168.145.x`），`ping 192.168.145.222`。
- 虚拟机是否桥接；NAT 模式不转发广播。
- 防火墙是否放行 UDP 5001（发现）和 5002（视频）。
- 抓包确认：`sudo tcpdump -ni any udp port 5001`，看有没有来自板子 IP 的包。
- PC 有多个网卡/VPN 时，广播可能走错接口；直连按路由走，更稳。

### 返回值

| 函数 | 正常 | 当前无目标 | 链路/线程故障 |
|---|---|---|---|
| `color()` | `"1"` / `"2"` / `"3"` | `"0"` | `"fuck"` |
| `stop()` | `"1"` | `"0"` | `"fuck"` |

**注意：所有返回值都是非空字符串，`"0"` 和 `"fuck"` 在 Python 里都是真值。**
判断必须写显式比较：

```python
if nv.color() == "1":
    ...
if nv.stop() == "1":
    ...
```

不要写 `if nv.stop():`，否则 `"0"` 也会被当成真。

### 生命周期

- 首次调用自动启动后台线程，不需要手动 `start()`。
- 线程是 daemon，进程退出时 `atexit` 自动清理。
- `color()`/`stop()` 不会向导航抛异常；故障通过 `"fuck"` 和 `status()` 暴露。
- `status()` 返回链路诊断：`running / board_ip / board_status / fps / frames /
  last_frame_age_ms / last_color / last_color_age_ms / last_color_error /
  last_stop / last_stop_age_ms / last_stop_error / last_error`。

### 超时规则

| 情况 | 返回 |
|---|---|
| 识别结果超过 `STALE_RESULT_S`（0.5s）没更新 | `"0"` |
| 完全没有新帧超过 `DEAD_STREAM_S`（2.0s） | `"fuck"` |
| 刚启动、第一帧还没到（2s 内） | `"0"` |

## 五、两个模块如何做到互不影响

- 只开一个 `receiver(2)` 读取线程，不出现两个线程抢同一路帧。
- 每帧顺序跑 color、stop 两个 detector。
- 每个 detector 独立 `try/except`：色卡算法异常不会中断 STOP，反之亦然。
- 两者各自维护结果槽和时间戳，互不共享跟踪状态。

## 六、调参

`color_core.py`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `MIN_AREA_RATIO` | 0.0135 | 最小色块面积占整帧比例；640×480 下约 4147 px |
| `MIN_VOTES` | 4 | 10 帧窗口内至少命中几帧才确认 |
| `LOST_TOLERANCE` | 5 | 已确认后连续丢几帧才取消 |
| `COLOR_DEFS` | 红/黄/蓝 | HSV 阈值和 BGR 绘制色 |

`stop_sign_core.py`：保持原参数（`MIN_VOTES=6`、`WINDOW=10`）。JPEG 4:2:0 会抬升色度误差，
阈值应在 PC 解码后的画面上标定，不要直接沿用摄像头原始画面的值。

## 七、没有包含

- 距离 `dist_mm`：当前不估距。
- `stop_info()` / 坐标输出：只给 `color()` 和 `stop()` 两个字符串。
- `Communication_comm.py` / UDP 结果上报：已作废，不参与本方案。
- PnP/第二路：默认关闭；需要时打开 `ENABLE_CSI2`，把逻辑通道 2 指向 CSI2，
  PC 端改用通道 2。
- 红色 STOP 牌本身也是红色块，`color()` 会把它当成红色色卡。若任务中两者可能同框，
  需要在导航层按阶段屏蔽，或给 `color_core.py` 的红色跟踪加形状过滤。

## 八、排障

| 现象 | 处理 |
|---|---|
| `color()` 一直 `"fuck"` | 看 `status()["last_error"]`；确认板端在发 ch1、防火墙放行 5001/5002 |
| `status()["board_status"]` 是 `c1=fail` | 检查 CSI1 排线/座子；只接 CSI1 单独验证 |
| 有画面但色卡不出框 | 调小 `MIN_AREA_RATIO`，或 `--debug` 看候选/ROI |
| 有画面但 STOP 不出框 | 看 `--debug` 的拒绝原因；检查红色阈值和白边比例 |
| 同一台 PC 同时跑 `main.py` 和导航程序 | 端口 5001/5002 会冲突，只能留一个接收进程 |
| 帧率只有 5–10 | 确认只开了一路（`ENABLE_CSI2=False`）；v1.8 软件 JPEG 仍是瓶颈 |
