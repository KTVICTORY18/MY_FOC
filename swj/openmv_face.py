# 目前还识别不出来不知道是什么原因
# 人脸识别例程（稳定版）
# - HQVGA + GRAYSCALE，速度和精度折中
# - stages=10（cascade 文件实际只有 15 级，取 10 提速）
# - threshold/scale 调宽松，距离 30~60cm、正脸可检出
import sensor, time, image, gc

# ------ 感光元件 ------
sensor.reset()
sensor.set_framesize(sensor.HQVGA)        # 240x160
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)                # 自动增益，适应不同光照
sensor.set_auto_whitebal(True)

# ------ 加载 Haar 算子 ------
gc.collect()
face_cascade = image.HaarCascade("/flash/frontalface.cascade", stages=10)

# ------ 主循环 ------
clock = time.clock()

while True:
    clock.tick()
    gc.collect()

    img = sensor.snapshot()

    # 阈值越小检测越宽松；scale 越接近 1 搜索越细（越慢但更准）
    objects = img.find_features(face_cascade,
                                threshold=0.5,
                                scale=1.25)

    for r in objects:
        img.draw_rectangle(r, color=255, thickness=2)

    if objects:
        print("FPS=%.1f  faces=%d  %s" % (clock.fps(), len(objects), objects))
    else:
        print("FPS=%.1f  no face" % clock.fps())
