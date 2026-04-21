# 原来红色小球代码的使用，不带图像传输
import sensor, image, time, pyb, ustruct
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
uart = pyb.UART(3, 1000000, timeout_char=10)
clock = time.clock()
red_threshold = (13, 49, 18, 61, 6, 47)
CENTER_X = 80
CENTER_Y = 60
def send_packet(found, dx, dy):
	try:
		data = ustruct.pack("<BBhhB", 0xBB, found, int(dx), int(dy), 0xFF)
		uart.write(data)
	except Exception as e:
		print("UART Write Error:", e)
while(True):
	clock.tick()
	img = sensor.snapshot()
	blobs = img.find_blobs([red_threshold], pixels_threshold=30, area_threshold=30, merge=True)
	if blobs:
		max_blob = max(blobs, key=lambda b: b.pixels())
		dx = max_blob.cx() - CENTER_X
		dy = max_blob.cy() - CENTER_Y
		found_flag = 1
	else:
		dx = 0
		dy = 0
		found_flag = 0
	send_packet(found_flag, dx, dy)