# =============================================================================
#  OpenMV 端程序
#  1) 保留原功能：通过 UART3 向电机以 7 字节数据包 (0xBB, found, dx, dy, 0xFF)
#                 发送识别结果。
#  2) 新增功能：通过 USB VCP 把 "图像 + 识别信息" 发送给 PC 上位机，
#                 并在图像上绘制：目标框、目标中心十字、图像中心十字。
#
#  USB VCP 协议 (小端序):
#    Header(2) = 0xAA 0x55
#    Length(2) = Info(17) + 4 + JpegLen
#    Info(17)  = <B h h h h h h h h>  -> found, dx, dy, cx, cy, w, h, img_w, img_h
#    JpegLen(4)
#    JPEG bytes
#    Tail(2)   = 0x55 0xAA
# =============================================================================

import sensor, image, time, pyb, ustruct

# -------------- 摄像头 --------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)      # 160 x 120
sensor.skip_frames(time=500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# -------------- 通信 --------------
uart = pyb.UART(3, 1000000, timeout_char=10)   # 原有：发给电机
usb  = pyb.USB_VCP()                           # 新增：发给 PC 上位机
clock = time.clock()

# -------------- 识别参数 --------------
red_threshold = (13, 49, 18, 61, 6, 47)
IMG_W = 160
IMG_H = 120
CENTER_X = IMG_W // 2    # 80
CENTER_Y = IMG_H // 2    # 60


def send_uart(found, dx, dy):
    """原有功能：通过 UART3 将 7 字节数据包发给电机 ID=2。"""
    try:
        data = ustruct.pack("<BBhhB", 0xBB, found, int(dx), int(dy), 0xFF)
        uart.write(data)
    except Exception as e:
        print("UART Write Error:", e)


def send_usb(found, dx, dy, cx, cy, bw, bh, img):
    """新增功能：通过 USB VCP 将图像和识别信息一并发送到 PC。"""
    # 没有主机在读就别发，防止阻塞
    if not usb.isconnected():
        return
    try:
        cimg = img.compressed(quality=60)
        try:
            jpeg_bytes = cimg.bytearray()
        except AttributeError:
            jpeg_bytes = bytes(cimg)

        info = ustruct.pack(
            "<Bhhhhhhhh",
            int(found),
            int(dx), int(dy),
            int(cx), int(cy),
            int(bw), int(bh),
            IMG_W, IMG_H
        )

        jpeg_len = len(jpeg_bytes)
        total_len = len(info) + 4 + jpeg_len

        # 每次 send 都加 100ms 超时，主机来不及读就丢帧，别阻塞主循环
        usb.send(b'\xAA\x55', timeout=100)
        usb.send(ustruct.pack("<H", total_len), timeout=100)
        usb.send(info, timeout=100)
        usb.send(ustruct.pack("<I", jpeg_len), timeout=100)
        usb.send(jpeg_bytes, timeout=200)
        usb.send(b'\x55\xAA', timeout=100)
    except Exception:
        # USB 断开/缓冲区忙等，直接丢本帧
        pass


# -------------- 主循环 --------------
while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([red_threshold],
                           pixels_threshold=30,
                           area_threshold=30,
                           merge=True)

    if blobs:
        max_blob = max(blobs, key=lambda b: b.pixels())
        cx, cy = max_blob.cx(), max_blob.cy()
        bw, bh = max_blob.w(), max_blob.h()
        dx = cx - CENTER_X
        dy = cy - CENTER_Y
        found_flag = 1

        # 目标框(绿) + 目标中心十字(红)
        img.draw_rectangle(max_blob.rect(), color=(0, 255, 0), thickness=2)
        img.draw_cross(cx, cy, color=(255, 0, 0), size=8, thickness=2)
    else:
        cx, cy = 0, 0
        bw, bh = 0, 0
        dx, dy = 0, 0
        found_flag = 0

    # 图像中心十字(蓝)
    img.draw_cross(CENTER_X, CENTER_Y, color=(0, 0, 255), size=10, thickness=2)
    img.draw_string(2, 2,
                    "FOUND" if found_flag else "LOST",
                    color=(255, 255, 0), scale=1)

    # 两路同时发送
    send_uart(found_flag, dx, dy)
    send_usb(found_flag, dx, dy, cx, cy, bw, bh, img)
