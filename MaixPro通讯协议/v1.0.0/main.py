# main.py — MaixPro 通用通讯模板
#
# 功能：打开摄像头持续显示，并通过串口（A16=TX / A17=RX，/dev/ttyS0）
#       持续收发二进制协议帧。发送走预分配内存池，接收逐帧校验分发。
#
# 使用说明：
#   1. 将本文件与 optimal_comm.py 一同放入 MaixCam 设备，用 MaixVision 运行。
#   2. 视觉算法请填入下方【视觉处理占位】区域，把结果通过 comm.send_* 发出。
#   3. TI 端下发的指令在 handle_command() 中分发处理。

from maix import camera, display, image, app, pinmap, time
from optimal_comm import OptimalComm

# ==========================================================
# 全局配置
# ==========================================================
_image_width = 320
_image_height = 240

_uart_device = "/dev/ttyS0"   # A16/A17 对应 UART0
_uart_baudrate = 115200

# 下行指令码约定（TI → Maix，与 TI 端保持一致）
CMD_HEARTBEAT = 0x00          # 心跳
CMD_START = 0x01              # 开始
CMD_STOP = 0x03              # 停止

# 上行帧类型码（Maix → TI，payload[0]，TI 端据此区分帧类型）
TAG_NO_TARGET = 0x00          # 无目标帧（数据域仅此 1 字节）
TAG_TARGET = 0x01             # 坐标帧（其后为 int16 x, y, w, h）

# 发送节流：控制上行数据帧频率，避免刷屏挤占带宽（毫秒）
_SEND_INTERVAL_MS = 50


# ==========================================================
# 接收指令分发
# ==========================================================
def handle_command(payload):
    '''
    处理一条已通过校验的下行 payload。
    payload[0] 约定为指令码，其余为参数。
    '''
    if len(payload) == 0:
        return
    cmd = payload[0]
    if cmd == CMD_HEARTBEAT:
        print("收到心跳")
    elif cmd == CMD_START:
        print("收到开始指令")
    elif cmd == CMD_STOP:
        print("收到停止指令")
    else:
        print("未知指令: 0x{:02X}, payload={}".format(cmd, payload.hex()))


# ==========================================================
# 主程序
# ==========================================================
def main():
    # 引脚映射：A16→UART0_TX，A17→UART0_RX
    pinmap.set_pin_function("A16", "UART0_TX")
    pinmap.set_pin_function("A17", "UART0_RX")

    comm = OptimalComm(_uart_device, _uart_baudrate)
    cam = camera.Camera(_image_width, _image_height)
    disp = display.Display()

    last_send = time.ticks_ms()

    while not app.need_exit():
        # ---- 1. 读取图像并显示 ----
        img = cam.read()

        # ==================================================
        # 【视觉处理占位】
        # 在此填入圆形识别 / 黑白柱识别等算法，得到目标坐标框。
        # 有目标 → target = (x, y, w, h)；无目标 → target = None。
        # 可用 img.draw_rect(...) / img.draw_cross(...) 叠加到画面上。
        # ==================================================
        target = (100, 50, 20, 10)  # 临时测试用，验证完改回视觉算法结果

        disp.show(img)

        # ---- 2. 持续接收：清空缓冲，逐帧分发 ----
        while True:
            payload = comm.poll()
            if payload is None:
                break
            handle_command(payload)

        # ---- 3. 定频上行发送：有目标发坐标帧，无目标发标志帧 ----
        now = time.ticks_ms()
        if now - last_send >= _SEND_INTERVAL_MS:
            last_send = now
            if target is not None:
                x, y, w, h = target
                comm.send_tagged_ints(TAG_TARGET, x, y, w, h)
            else:
                comm.send_tagged_ints(TAG_NO_TARGET)


if __name__ == '__main__':
    main()
