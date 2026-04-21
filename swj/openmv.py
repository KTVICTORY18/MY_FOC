# 红色小球代码 可以使用 并且去掉了灯闪
import sensor, image, time, pyb, ustruct, gc
pyb.delay(2000)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)    # 160x120 数据量更小，USB 传输更快
sensor.skip_frames(time=500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = pyb.UART(3, 1000000, timeout_char=10)
usb  = pyb.USB_VCP()
clock = time.clock()

red_threshold = (13, 49, 18, 61, 6, 47)
IMG_W, IMG_H = 160, 120
CENTER_X, CENTER_Y = 80, 60

# 最近一次识别结果（用在 snap 的响应里）
_last = {
    "found": 0, "dx": 0, "dy": 0,
    "cx": 0, "cy": 0, "bw": 0, "bh": 0,
}


def send_uart(found, dx, dy):
    try:
        data = ustruct.pack("<BBhhB", 0xBB, found, int(dx), int(dy), 0xFF)
        uart.write(data)
    except:
        pass


# ============================================================
# 主循环 —— 参考官方 USB VCP 示例：阻塞等 'snap'，带超时
# 超时返回就跑一次识别 + UART；收到 'snap' 就回一帧图像
# ============================================================
while True:
    # ------ 识别 + 发电机 ------
    clock.tick()
    img = sensor.snapshot()
    # QQVGA 分辨率下物体像素更少，降低阈值以保持检出率
    blobs = img.find_blobs([red_threshold],
                           pixels_threshold=30,
                           area_threshold=30,
                           merge=True)
    if blobs:
        b = max(blobs, key=lambda b: b.pixels())
        cx, cy, bw, bh = b.cx(), b.cy(), b.w(), b.h()
        dx, dy = cx - CENTER_X, cy - CENTER_Y
        found_flag = 1
        img.draw_rectangle(b.rect(), color=(0, 255, 0), thickness=1)
        img.draw_cross(cx, cy, color=(255, 0, 0), size=6, thickness=1)
    else:
        cx = cy = bw = bh = dx = dy = 0
        found_flag = 0
    img.draw_cross(CENTER_X, CENTER_Y, color=(0, 0, 255), size=8, thickness=1)

    send_uart(found_flag, dx, dy)

    _last["found"] = found_flag
    _last["dx"] = dx; _last["dy"] = dy
    _last["cx"] = cx; _last["cy"] = cy
    _last["bw"] = bw; _last["bh"] = bh

    # ------ USB 请求/响应：参考官方 demo，阻塞等 snap (含短超时) ------
    cmd = usb.recv(4, timeout=5)
    if cmd == b'snap':
        # 先压 JPEG
        cimg = img.compress(quality=70)
        jpeg_len = cimg.size()
        # Info (17B) + <L>(4B) 合并一次发送，减少一次 USB 端点握手延时
        header = ustruct.pack("<BhhhhhhhhL",
                              int(_last["found"]),
                              int(_last["dx"]), int(_last["dy"]),
                              int(_last["cx"]), int(_last["cy"]),
                              int(_last["bw"]), int(_last["bh"]),
                              IMG_W, IMG_H,
                              jpeg_len)
        try:
            usb.send(header)
            usb.send(cimg)
        except:
            pass
        del cimg
    gc.collect()
