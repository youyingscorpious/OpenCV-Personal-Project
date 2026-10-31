# 导入 struct：按协议规定的小端格式编码和解码多字节字段。
import struct  # noqa: F401  # 协议字段编码使用小端二进制布局。

# 定义共享帧头：电控与视觉协议都使用 AA 55。
HEAD = b'\xAA\x55'
# 定义电控协议最大数据区长度：与 STM32 protocol.h 的上限对齐。
MAX_ELECTRICAL_DATA = 16
# 定义视觉协议最大数据区长度：与视觉端协议表一致。
MAX_VISION_DATA = 32
# 定义底盘速度命令：数据区为左右轮两个 int16 小端值。
CMD_CHASSIS = 0x01
# 定义电机板心跳命令：运动期间至少每 200 ms 刷新一次。
CMD_PING = 0x00
# 定义电机板本地循迹命令：两个可选 int16 参数为基础速度和转向限幅。
CMD_LINE = 0x07
# 定义清除电机急停、超时、路口和丢线状态的命令。
CMD_CLEAR_FAULT = 0x05
# 定义舵机角度命令：数据区为舵机 ID 和角度 x10。
CMD_SERVO = 0x02
# 定义机械臂动作命令：1 抓取、2 放置、3 复位。
CMD_ARM = 0x03
# 定义急停命令：电机板锁存急停，外设板中止舵机动作序列，见 docs/MCU_INTERFACE.md。
CMD_ESTOP = 0x04
# 定义电控状态回报命令：状态数据为左右轮速度与状态字节。
CMD_STATUS = 0x10
# 定义电机板循迹遥测状态帧命令。
CMD_CHASSIS_STATUS = 0x11
# 定义电机固件状态码：空闲、运行。
STATE_IDLE, STATE_RUNNING = 0, 1
# 定义电机固件状态码：急停/故障和命令超时。
STATE_FAULT, STATE_TIMEOUT = 4, 5
# 定义电机固件状态码：路口停车和循迹丢线。
STATE_JUNCTION, STATE_LINE_LOST = 6, 7
# 定义外设板状态码：机械臂忙和动作完成。
STATE_ARM_BUSY, STATE_ARM_DONE = 2, 3
# 定义 0x11 状态帧中的路口事件位。
EVENT_JUNCTION = 0x01
# 定义 0x11 状态帧中的循迹丢失事件位。
EVENT_LINE_LOST = 0x02
# 定义视觉目标列表命令：颜色、距离、置信度和 STOP 位域。
VISION_TARGET_LIST = 0x01
# 定义视觉目标坐标命令：目标三维坐标和偏航角。
VISION_TARGET_COORD = 0x02
# 定义视觉心跳命令：用于检测相机/模型/网络状态。
VISION_HEARTBEAT = 0x03
# 定义视觉错误命令：记录视觉模块错误。
VISION_ERROR = 0x04
# 定义视觉版本命令：用于确认两路视觉角色上线。
VISION_VERSION = 0x05
# 定义路线颜色编码：1 红左、2 黄中、3 蓝右、4 停车。
ROUTE_CODES = {1: 1, 2: 2, 3: 3}

# 计算 CRC-16/MODBUS：输入为 LEN、CMD 和 DATA，低字节先发送。
def crc16_modbus(data):
    # 初始化 CRC 寄存器为协议规定的 0xFFFF。
    crc = 0xFFFF
    # 对输入缓冲区逐字节执行 CRC 更新。
    for value in data:
        # 将当前字节异或到 CRC 低位。
        crc ^= value
        # 对当前字节的 8 个比特逐位计算。
        for _ in range(8):
            # 低位为 1 时右移并异或 MODBUS 多项式。
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    # 将 CRC 限制为 16 位无符号数。
    return crc & 0xFFFF

# 构造电控 UART 帧：AA 55 LEN CMD DATA CRC_L CRC_H。
def build_electrical_frame(cmd, data=b''):
    # 将输入数据转换成不可变字节串，防止错误类型进入协议。
    data = bytes(data)
    # LEN 包含 CMD 和 DATA，不包含帧头、LEN 自身及 CRC。
    length = len(data) + 1
    # 拒绝超出 MCU 接收缓冲区的负载。
    if len(data) > MAX_ELECTRICAL_DATA:
        # 抛出明确异常，避免发送不完整或越界帧。
        raise ValueError('electrical payload exceeds 16 bytes')
    # CRC 覆盖 LEN、CMD 和 DATA 三部分。
    body = bytes((length, cmd & 0xFF)) + data
    # 计算 MODBUS CRC 并以低字节在前追加。
    crc = crc16_modbus(body)
    # 返回完整电控协议帧。
    return HEAD + body + struct.pack('<H', crc)

# 增量解析任意 TCP/串口字节片段中的连续帧。
class StreamParser:
    # 初始化解析器并选择视觉 SUM8 或电控 CRC16 模式。
    def __init__(self, protocol):
        # 保存解析模式字符串。
        self.protocol = protocol
        # 创建接收缓冲区以处理粘包和半包。
        self.buffer = bytearray()
        # 记录校验错误数，供日志和 ROS 诊断使用。
        self.errors = 0

    # 清除缓冲区：用于断线重连后丢弃旧半帧。
    def reset(self):
        # 清空尚未组成完整帧的字节。
        self.buffer.clear()

    # 将新收到的数据追加到解析缓冲区并返回所有完整有效帧。
    def feed(self, chunk):
        # 将新字节追加至缓冲区。
        self.buffer.extend(chunk)
        # 初始化当前调用的输出帧列表。
        frames = []
        # 只要缓冲区仍可能含有完整帧，就继续尝试解析。
        while True:
            # 寻找帧头首字节 AA。
            start = self.buffer.find(HEAD)
            # 未找到完整帧头时，保留可能作为下个帧头的末尾 AA。
            if start < 0:
                # 检查尾字节是否可能是跨片段的帧头首字节。
                keep_tail = bool(self.buffer and self.buffer[-1] == HEAD[0])
                # 清除无效前缀，同时保留潜在帧头字节。
                self.buffer[:] = self.buffer[-1:] if keep_tail else b''
                # 当前没有可解析帧，结束本轮解析。
                break
            # 删除帧头前的噪声数据。
            if start:
                # 只留下从 AA 开始的数据。
                del self.buffer[:start]
            # 视觉帧固定头部加校验总计至少 7 字节。
            if len(self.buffer) < 6:
                # 数据不足时等待下一次 feed。
                break
            # 读取数据长度字段。
            length = self.buffer[2]
            # 根据协议类型设置长度上限和固定帧开销。
            limit = MAX_VISION_DATA if self.protocol == 'vision' else MAX_ELECTRICAL_DATA + 1
            # 丢弃长度非法的帧头并继续从后续数据重新同步。
            if length > limit or (self.protocol == 'electrical' and length < 1):
                # 计数非法长度错误。
                self.errors += 1
                # 丢弃一个字节以搜索下一个候选帧头。
                del self.buffer[0]
                # 继续解析剩余字节。
                continue
            # 视觉帧由 2 头 + 4 字段 + DATA + 1 SUM 组成。
            total = 7 + length if self.protocol == 'vision' else 5 + length
            # 如果当前片段未包含整帧，则保留缓冲区等待后续数据。
            if len(self.buffer) < total:
                # 本次收到的数据尚不完整。
                break
            # 复制一帧字节，避免后续删缓冲区影响字段解析。
            raw = bytes(self.buffer[:total])
            # 删除当前候选帧，为解析后续拼接帧做准备。
            del self.buffer[:total]
            # 对视觉协议校验 LEN+SRC+CMD+SEQ+DATA 的 SUM8。
            if self.protocol == 'vision':
                # 取出视觉协议的校验范围。
                fields = raw[2:-1]
                # 校验最后一个字节是否等于字段和低 8 位。
                valid = (sum(fields) & 0xFF) == raw[-1]
                # 校验失败时计数并丢弃该候选帧。
                if not valid:
                    # 累加校验错误数。
                    self.errors += 1
                    # 继续解析缓冲区中的后续帧。
                    continue
                # 提取视觉帧的来源、命令、序号和数据区。
                frames.append((raw[3], raw[4], raw[5], raw[6:-1]))
            # 对电控协议校验 CRC-16/MODBUS。
            else:
                # 取出 LEN、CMD 和 DATA 部分参与 CRC 计算。
                body = raw[2:-2]
                # 从帧尾读取小端 CRC16。
                received_crc = struct.unpack('<H', raw[-2:])[0]
                # 比较计算所得 CRC 与帧内 CRC。
                if crc16_modbus(body) != received_crc:
                    # 累加 CRC 错误数。
                    self.errors += 1
                    # 继续搜索后续完整帧。
                    continue
                # 提取电控帧的命令字与 DATA。
                frames.append((raw[3], raw[4:-2]))
        # 返回本次字节片段中解析成功的帧列表。
        return frames

# 解析电控 CMD_STATUS 数据：两个 int16 速度和一个状态字节。
def parse_status(data):
    # 要求状态负载必须恰好包含五字节。
    if len(data) != 5:
        # 返回无效结果，不让错误长度改变导航状态。
        return None
    # 以小端读取左右轮速度，并保留最后一个状态字节。
    left, right = struct.unpack('<hh', data[:4])
    # 返回便于状态机判断的三元组。
    return left, right, data[4]

# 解析电机 CMD_CHASSIS_STATUS：循迹位、PID 量、里程、航向和事件。
def parse_chassis_status(data):
    # 固件负载布局是 B,h,h,i,h,B，共 12 字节，小端序。
    if len(data) != 12:
        # 长度错误时拒绝更新遥测。
        return None
    # 依次解码传感器位、误差、修正量、里程、航向 x10 和事件。
    line_bits, error_x10, correction, distance_mm, heading_x10, event = struct.unpack('<BhhihB', data)
    # 返回单位明确的字段，并屏蔽未使用的高四位。
    return {'line_bits': line_bits & 0x0F, 'error_x10': error_x10, 'correction': correction, 'distance_mm': distance_mm, 'heading_deg': heading_x10 / 10.0, 'event': event}

# 解析外设板 CMD_STATUS：三个舵机角度、外设状态和四路循迹位。
def parse_peripheral_status(data):
    # 外设固件状态负载固定为 8 字节，所有角度均为 int16 小端。
    if len(data) != 8:
        # 长度不符时不更新状态机或循迹输入。
        return None
    # 解码舵机角度 x10、机械臂状态和 PB12..PB15 位图。
    servo1, servo2, servo3 = struct.unpack_from('<hhh', data, 0)
    # 返回统一字段顺序，角度单位保持固件定义的 0.1 度。
    return servo1, servo2, servo3, data[6], data[7] & 0x0F

# 解析视觉 CMD_TARGET_LIST 的 12 字节数据。
def parse_target_list(data):
    # 拒绝长度不符合协议的视觉数据。
    if len(data) != 12:
        # 返回无效数据，不触发路线或停车动作。
        return None
    # 解析目标数、两个目标和状态位域。
    count = min(data[0], 2)
    # 解码第一个目标的 ID、颜色、距离和置信度。
    target1 = {'id': data[1], 'color': data[2], 'distance_mm': struct.unpack_from('<H', data, 3)[0], 'confidence': data[5]}
    # 解码第二个目标的 ID、颜色、距离和置信度。
    target2 = {'id': data[6], 'color': data[7], 'distance_mm': struct.unpack_from('<H', data, 8)[0], 'confidence': data[10]}
    # 读取状态位域：bit0 颜色有效、bit1 距离有效、bit2 STOP。
    status = data[11]
    # 返回解析结果供 ROS 发布与路线状态机使用。
    return {'count': count, 'targets': [target1, target2][:count], 'status': status, 'color_valid': bool(status & 0x01), 'distance_valid': bool(status & 0x02), 'stop_detected': bool(status & 0x04), 'calibrated': bool(status & 0x08)}

# 解析视觉 CMD_TARGET_COORD 的 12 字节数据。
def parse_target_coord(data):
    # 拒绝长度不符合协议的坐标数据。
    if len(data) != 12:
        # 返回无效数据，不把格式错误的坐标交给下游。
        return None
    # 按协议布局解析 ID、valid、XYZ、yaw、confidence 和 status。
    target_id, valid, x_mm, y_mm, z_mm, yaw_ddeg, confidence, status = struct.unpack('<BBhhhhBB', data)
    # 将 0x8000 的有符号等价值 -32768 转换为无效偏航角。
    yaw_value = None if yaw_ddeg == -32768 else yaw_ddeg / 10.0
    # 返回坐标、单位、有效标志和状态位。
    return {'target_id': target_id, 'valid': bool(valid), 'x_mm': x_mm, 'y_mm': y_mm, 'z_mm': z_mm, 'yaw_deg': yaw_value, 'confidence': confidence, 'status': status, 'in_range': bool(status & 0x01), 'calibrated': bool(status & 0x02), 'static': bool(status & 0x04)}

# 解析视觉 CMD_HEARTBEAT：状态、FPS、错误数、处理耗时和协议版本。
def parse_vision_heartbeat(data):
    # 心跳负载固定 8 字节，字段布局与 Communication_comm.py 的 <BBHHBB 一致。
    if len(data) != 8:
        # 返回无效结果，避免畸形心跳更新诊断状态。
        return None
    # 解码状态位、帧率、累计错误、处理毫秒和协议主次版本。
    status, fps, err_count, proc_ms, proto_major, proto_minor = struct.unpack('<BBHHBB', data)
    # 返回具名映射以便日志或 ROS 话题清楚表达字段含义。
    return {'status': status, 'fps': fps, 'err_count': err_count, 'proc_ms': proc_ms, 'protocol_major': proto_major, 'protocol_minor': proto_minor}

# 解析视觉 CMD_ERROR：错误码、细节码和累计计数。
def parse_vision_error(data):
    # 错误负载固定 4 字节，采用小端 uint16 计数。
    if len(data) != 4:
        # 返回无效结果，不记录格式错误内容为有效错误码。
        return None
    # 按 <BBH 布局解码错误类别、附加细节和计数。
    err_code, detail, counter = struct.unpack('<BBH', data)
    # 返回字段映射供导航节点写日志。
    return {'err_code': err_code, 'detail': detail, 'counter': counter}
