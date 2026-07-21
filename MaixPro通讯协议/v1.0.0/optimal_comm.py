# optimal_comm.py — MaixPro 最优串口通讯驱动库
#
# 融合两大优势的"究极形态"：
#   1. 二进制结构体协议（源自 serial_protocol.py）——TI 端 C 结构体可直接强转解析，
#      无字符串解析开销，无内存溢出风险。
#   2. 预分配内存池（源自 uart.py）——发送用 struct.pack_into 写入固定 bytearray，
#      彻底告别每帧 new 对象，杜绝 Python GC 掉帧。
#
# 数据帧格式（小端）：
#   帧头(0xAA) + 数据长度(2字节) + 数据域 + 校验和(1字节) + 帧尾(0x55)
#   校验和 = (长度2字节 + 数据域) 逐字节累加 & 0xFF
#
# TI(C语言)端接收结构示意：
#   typedef struct { uint8_t head; uint16_t len; uint8_t payload[N];
#                    uint8_t checksum; uint8_t tail; } Frame;

import struct
from maix import uart


class OptimalComm:
    HEAD = 0xAA
    TAIL = 0x55

    # 发送缓冲池容量：帧头1 + 长度2 + 数据域 + 校验1 + 帧尾1。
    # 数据域预留 64 字节，足够坐标框/多目标等常规负载；如需更大自行调高。
    _MAX_PAYLOAD = 64
    _BUF_SIZE = _MAX_PAYLOAD + 5

    def __init__(self, device="/dev/ttyS0", baudrate=115200):
        '''
        参数：
            device：串口设备节点（MaixCam A16/A17 对应 /dev/ttyS0）
            baudrate：波特率，需与 TI 端一致
        '''
        self.serial = uart.UART(device, baudrate)

        # 核心优化 ①：预分配发送缓冲区，全程复用同一块内存，不触发 GC。
        self._tx_buffer = bytearray(self._BUF_SIZE)
        self._tx_buffer[0] = self.HEAD

        # 核心优化 ②：接收滚动缓冲区，累积字节流，逐帧解析后丢弃已消费部分。
        self._rx_buffer = bytearray()

    # ==========================================================
    # 校验
    # ==========================================================
    def _checksum(self, data):
        '''逐字节累加取低 8 位（data 支持 memoryview 以避免切片拷贝）'''
        check_sum = 0
        for a in data:
            check_sum = (check_sum + a) & 0xFF
        return check_sum

    # ==========================================================
    # 发送：全部走预分配内存池，零动态分配
    # ==========================================================
    def _send_buffer(self, payload_len):
        '''
        按 payload_len 填充长度域、校验和、帧尾，并发送 buffer 有效切片。
        调用前须已把数据域写入 self._tx_buffer[3 : 3+payload_len]。
        '''
        # 长度域（小端 2 字节）写入偏移 1
        struct.pack_into('<H', self._tx_buffer, 1, payload_len)
        # 校验域 = 对 [长度域 + 数据域] 累加，即 buffer[1 : 3+payload_len]
        csum = self._checksum(memoryview(self._tx_buffer)[1:3 + payload_len])
        self._tx_buffer[3 + payload_len] = csum
        self._tx_buffer[4 + payload_len] = self.TAIL
        # bytes() 仅拷贝有效切片，长度可控
        self.serial.write(bytes(self._tx_buffer[:5 + payload_len]))

    def send_simple_cmd(self, cmd_code):
        '''发送单字节指令，如动作码 0x01 / 0x03'''
        self._tx_buffer[3] = cmd_code & 0xFF
        self._send_buffer(1)

    def send_payload(self, payload):
        '''
        发送任意字节负载（bytes / bytearray）。
        超出预分配池容量时截断到 _MAX_PAYLOAD，保证内存边界安全。
        '''
        n = len(payload)
        if n > self._MAX_PAYLOAD:
            n = self._MAX_PAYLOAD
        self._tx_buffer[3:3 + n] = payload[:n]
        self._send_buffer(n)

    def send_ints(self, *values):
        '''
        发送若干 16 位有符号整数（小端），典型用途：坐标框 x, y, w, h。
        直接 pack_into 进内存池，无中间 bytes 对象。
        '''
        n = len(values) * 2
        if n > self._MAX_PAYLOAD:
            return
        struct.pack_into('<{}h'.format(len(values)), self._tx_buffer, 3, *values)
        self._send_buffer(n)

    def send_tagged_ints(self, tag, *values):
        '''
        发送 "类型标志字节 + 若干 16 位有符号整数（小端）"。
        数据域布局：payload[0]=tag，其后依次为各 int16。
        TI 端读 payload[0] 即可区分帧类型（如坐标帧 / 无目标帧）。
        同样全程 pack_into 进内存池，零动态分配。
        '''
        n = 1 + len(values) * 2
        if n > self._MAX_PAYLOAD:
            return
        self._tx_buffer[3] = tag & 0xFF
        if values:
            struct.pack_into('<{}h'.format(len(values)), self._tx_buffer, 4, *values)
        self._send_buffer(n)

    # ==========================================================
    # 接收：滚动缓冲 + 逐帧校验解析
    # ==========================================================
    def poll(self):
        '''
        非阻塞读取串口并解析。每次调用返回一条已通过校验的 payload（bytes），
        无完整有效帧时返回 None。需在主循环中持续调用以清空缓冲。
        '''
        data = self.serial.read()
        if data:
            self._rx_buffer.extend(data)

        buf = self._rx_buffer

        # 定位帧头，丢弃帧头之前的冗余字节
        head_idx = -1
        for i, b in enumerate(buf):
            if b == self.HEAD:
                head_idx = i
                break
        if head_idx < 0:
            # 无帧头，全部是噪声，清空
            if len(buf) > 0:
                del buf[:]
            return None
        if head_idx > 0:
            del buf[:head_idx]

        # 至少要有 帧头1+长度2 才能读出 payload 长度
        if len(buf) < 3:
            return None

        payload_len = struct.unpack('<H', bytes(buf[1:3]))[0]
        frame_len = payload_len + 5  # 头1 + 长2 + 数据 + 校验1 + 尾1

        # 数据尚未收全，等待下次 poll
        if len(buf) < frame_len:
            return None

        checksum = buf[3 + payload_len]
        tail = buf[4 + payload_len]

        if tail == self.TAIL and self._checksum(memoryview(buf)[1:3 + payload_len]) == checksum:
            payload = bytes(buf[3:3 + payload_len])
            del buf[:frame_len]          # 消费掉这一整帧
            return payload
        else:
            # 校验/帧尾错误：跳过当前帧头一个字节，下次重新找帧头
            del buf[:1]
            return None
