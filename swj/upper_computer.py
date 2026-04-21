# -*- coding: utf-8 -*-
"""
云台追踪上位机 (PyQt5) —— 仅连接 OpenMV USB
- 左侧：显示 OpenMV 通过 USB VCP 推送的实时图像（带框和十字标记）
- 右侧：显示识别信息
    1) 图像中心点坐标       (固定)
    2) 物体中心点坐标       (来自 OpenMV)
    3) 是否检测到物体       (来自 OpenMV)
    4) 物体移动速度 px/s    (PC 端用位移/时间差计算)
    5) x/y 轴偏移量         (来自 OpenMV)
    6) 捕获耗时 ms          (PC 端位置跳变方案：
                             跳变条件——|Δdx|或|Δdy|大于阈值；
                             起点——跳变前一帧的时间戳；
                             终点——|dx|,|dy| 同时 <5 的一帧)
"""

import sys
import time
import struct

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QComboBox, QHBoxLayout, QVBoxLayout, QGridLayout, QGroupBox,
    QStatusBar, QFrame, QSizePolicy
)

import serial
import serial.tools.list_ports

# ---------------- 阈值参数（可按需调整）----------------
JUMP_TH_DIFF = 15   # 像素，帧间差分阈值（判定跳变）
LOCK_TH      = 5    # 像素，|dx|,|dy| 同时 <LOCK_TH 视为追上
IMG_W        = 160
IMG_H        = 120
CENTER_X     = IMG_W // 2
CENTER_Y     = IMG_H // 2

# ============================================================
#                    OpenMV USB 串口读取线程
# ============================================================
class OpenMVReader(QThread):
    """
    解析 OpenMV USB VCP 协议：
      Header(2)=AA 55 | Len(2) | Info(17) | JpegLen(4) | JPEG | Tail(2)=55 AA
    """
    frame_received = pyqtSignal(dict)       # {image: bytes, found, dx, dy, cx, cy, w, h}
    status         = pyqtSignal(str)

    HEADER = b'\xAA\x55'
    TAIL   = b'\x55\xAA'

    def __init__(self, port, baud=921600, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._running = True
        self._buf = bytearray()

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:
            self.status.emit(f"[OpenMV] 打开失败: {e}")
            return
        self.status.emit(f"[OpenMV] 已连接 {self.port}")

        while self._running:
            try:
                data = ser.read(4096)
                if data:
                    self._buf.extend(data)
                    self._parse()
            except Exception as e:
                self.status.emit(f"[OpenMV] 读取错误: {e}")
                break
        try:
            ser.close()
        except Exception:
            pass
        self.status.emit("[OpenMV] 已断开")

    def _parse(self):
        while True:
            idx = self._buf.find(self.HEADER)
            if idx < 0:
                # 防止缓冲区膨胀
                if len(self._buf) > 65536:
                    del self._buf[:-2]
                return
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < 4:
                return
            total_len = struct.unpack("<H", bytes(self._buf[2:4]))[0]
            frame_total = 2 + 2 + total_len + 2
            if len(self._buf) < frame_total:
                return
            frame = bytes(self._buf[:frame_total])
            del self._buf[:frame_total]
            if frame[-2:] != self.TAIL:
                # 丢弃这一帧
                continue
            try:
                payload = frame[4:4 + total_len]
                info = payload[:17]
                found, dx, dy, cx, cy, bw, bh, img_w, img_h = struct.unpack("<Bhhhhhhhh", info)
                jpeg_len = struct.unpack("<I", payload[17:21])[0]
                jpeg = payload[21:21 + jpeg_len]
                self.frame_received.emit({
                    "jpeg": jpeg,
                    "found": found,
                    "dx": dx, "dy": dy,
                    "cx": cx, "cy": cy,
                    "w": bw, "h": bh,
                    "img_w": img_w, "img_h": img_h,
                })
            except Exception as e:
                self.status.emit(f"[OpenMV] 帧解析错误: {e}")

    def stop(self):
        self._running = False


# ============================================================
#                          主窗口
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("云台追踪上位机 — MY_FOC")
        self.resize(1100, 620)

        self.openmv_thread = None

        # --- 跳变检测用 ---
        self._last_dx = None
        self._last_dy = None
        self._last_ts = None        # 上一帧 PC 时间戳
        self._prev_ts = None        # 上上一帧时间戳（作为跳变起点）
        self._jump_start_ts = None  # 本次跳变起点
        self._in_tracking = False
        self._last_capture_ms = 0.0
        # --- 速度计算 ---
        self._last_cx = None
        self._last_cy = None
        self._speed_px_s = 0.0

        self._build_ui()
        self.refresh_ports()

    # ---------------- UI ----------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # 顶部连接栏
        top = QHBoxLayout()
        self.cb_openmv = QComboBox(); self.cb_openmv.setMinimumWidth(220)
        self.btn_refresh = QPushButton("刷新串口")
        self.btn_conn_om = QPushButton("连接 OpenMV")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn_om.clicked.connect(self.toggle_openmv)


        top.addWidget(QLabel("OpenMV 端口："));  top.addWidget(self.cb_openmv)
        top.addWidget(self.btn_conn_om)
        top.addStretch(1)
        top.addWidget(self.btn_refresh)
        root.addLayout(top)

        # 主体：左图像 + 右数据
        body = QHBoxLayout()
        body.setSpacing(12)

        # --- 左：图像区 ---
        left_box = QGroupBox("实时图像 (OpenMV USB)")
        left_lay = QVBoxLayout(left_box)
        self.lbl_image = QLabel("等待图像…")
        self.lbl_image.setAlignment(Qt.AlignCenter)
        self.lbl_image.setMinimumSize(640, 480)
        self.lbl_image.setStyleSheet(
            "background-color:#1e1e1e; color:#888; border:1px solid #333; border-radius:6px;")
        left_lay.addWidget(self.lbl_image)
        self.lbl_fps = QLabel("FPS: --")
        self.lbl_fps.setStyleSheet("color:#6cf;")
        left_lay.addWidget(self.lbl_fps, 0, Qt.AlignRight)
        body.addWidget(left_box, stretch=3)

        # --- 右：数据面板 ---
        right_box = QGroupBox("追踪数据")
        right_lay = QVBoxLayout(right_box)
        right_lay.setSpacing(10)

        self.val = {}
        def add_item(key, label_text):
            card = QFrame()
            card.setStyleSheet(
                "QFrame{background:#2a2d34;border:1px solid #3a3f48;border-radius:8px;}")
            lay = QVBoxLayout(card); lay.setContentsMargins(12, 8, 12, 8); lay.setSpacing(2)
            t = QLabel(label_text); t.setStyleSheet("color:#9aa5b1;font-size:12px;")
            v = QLabel("—");         v.setStyleSheet("color:#e6edf3;font-size:18px;font-weight:600;")
            lay.addWidget(t); lay.addWidget(v)
            self.val[key] = v
            right_lay.addWidget(card)

        add_item("img_center",  "图像中心点 (px)")
        add_item("obj_center",  "物体中心点 (px)")
        add_item("found",       "是否检测到物体")
        add_item("speed",       "物体移动速度 (px/s)")
        add_item("offset",      "x / y 轴偏移量 (px)")
        add_item("capture",     "追上耗时 (ms)  · 跳变起点方案")

        right_lay.addStretch(1)
        body.addWidget(right_box, stretch=2)
        root.addLayout(body)

        # 状态栏
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("就绪")

        # 初始化固定项
        self.val["img_center"].setText(f"({CENTER_X}, {CENTER_Y})")

        # FPS 计时
        self._fps_frames = 0
        self._fps_t0 = time.time()
        self._fps_timer = QTimer(self); self._fps_timer.timeout.connect(self._tick_fps)
        self._fps_timer.start(500)

        self._apply_style()

    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow { background:#1b1d23; }
            QGroupBox {
                color:#c9d1d9; font-weight:600;
                border:1px solid #333a44; border-radius:10px;
                margin-top:12px; padding:8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin; left:12px; padding:0 6px;
            }
            QLabel { color:#c9d1d9; }
            QPushButton {
                background:#2f81f7; color:white; border:none; border-radius:6px;
                padding:6px 14px; font-weight:600;
            }
            QPushButton:hover  { background:#4593ff; }
            QPushButton:pressed{ background:#1f6feb; }
            QComboBox {
                background:#2a2d34; color:#e6edf3; border:1px solid #3a3f48;
                border-radius:6px; padding:4px 8px; min-height:24px;
            }
            QStatusBar { color:#8b949e; }
        """)

    # ---------------- 端口管理 ----------------
    def refresh_ports(self):
        ports = [p.device + " — " + (p.description or "") for p in serial.tools.list_ports.comports()]
        self.cb_openmv.clear()
        if not ports:
            self.cb_openmv.addItem("(无串口)")
        else:
            self.cb_openmv.addItems(ports)
        self.statusBar().showMessage(f"找到 {len(ports)} 个串口")

    def _selected_port(self, combo):
        txt = combo.currentText()
        if "—" in txt:
            return txt.split("—", 1)[0].strip()
        return txt.strip() if txt else ""

    # ---------------- 连接切换 ----------------
    def toggle_openmv(self):
        if self.openmv_thread and self.openmv_thread.isRunning():
            self.openmv_thread.stop(); self.openmv_thread.wait(1000)
            self.openmv_thread = None
            self.btn_conn_om.setText("连接 OpenMV")
            return
        port = self._selected_port(self.cb_openmv)
        if not port or port.startswith("("):
            self.statusBar().showMessage("请先选择 OpenMV 串口"); return
        self.openmv_thread = OpenMVReader(port)
        self.openmv_thread.frame_received.connect(self.on_openmv_frame)
        self.openmv_thread.status.connect(self.statusBar().showMessage)
        self.openmv_thread.start()
        self.btn_conn_om.setText("断开 OpenMV")

    # ---------------- 数据回调 ----------------
    def on_openmv_frame(self, f):
        # 1) 显示 JPEG
        img = QImage.fromData(f["jpeg"], "JPG")
        if not img.isNull():
            pm = QPixmap.fromImage(img).scaled(
                self.lbl_image.width(), self.lbl_image.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.lbl_image.setPixmap(pm)
            self._fps_frames += 1

        # 2) 更新数据
        found = f["found"]; dx = f["dx"]; dy = f["dy"]
        cx = f["cx"]; cy = f["cy"]

        self.val["obj_center"].setText(f"({cx}, {cy})" if found else "—")
        self.val["found"].setText("✅ FOUND" if found else "❌ LOST")
        self.val["found"].setStyleSheet(
            "color:#3fb950;font-size:18px;font-weight:700;" if found
            else "color:#f85149;font-size:18px;font-weight:700;")
        self.val["offset"].setText(f"dx = {dx:+d}   dy = {dy:+d}")

        now = time.monotonic()

        # 3) 速度（像素/秒）：仅在连续找到目标时才计算
        if found and self._last_cx is not None and self._last_ts is not None:
            dt = now - self._last_ts
            if dt > 1e-4:
                import math
                dist = math.hypot(cx - self._last_cx, cy - self._last_cy)
                self._speed_px_s = dist / dt
        if found:
            self._last_cx, self._last_cy = cx, cy
        else:
            self._last_cx = self._last_cy = None
            self._speed_px_s = 0.0
        self.val["speed"].setText(f"{self._speed_px_s:7.1f}")

        # 4) 位置跳变检测 & 捕获耗时
        #    跳变条件：|dx - last_dx| > TH 或 |dy - last_dy| > TH
        #    起点：跳变帧的上一帧时间戳（prev_ts）
        #    终点：首次 |dx|,|dy| 同 <LOCK_TH
        if self._last_dx is not None:
            ddx = abs(dx - self._last_dx); ddy = abs(dy - self._last_dy)
            if (ddx > JUMP_TH_DIFF or ddy > JUMP_TH_DIFF):
                # 发生跳变 → 开始追的起点是上一帧时间
                self._jump_start_ts = self._prev_ts if self._prev_ts is not None else self._last_ts
                self._in_tracking = True

        if self._in_tracking and self._jump_start_ts is not None:
            if abs(dx) < LOCK_TH and abs(dy) < LOCK_TH and found:
                self._last_capture_ms = (now - self._jump_start_ts) * 1000.0
                self._in_tracking = False
                self._jump_start_ts = None

        status_txt = "追踪中…" if self._in_tracking else "已锁定"
        self.val["capture"].setText(
            f"{self._last_capture_ms:7.1f}   ({status_txt})")

        # 保存历史时间戳
        self._prev_ts = self._last_ts
        self._last_ts = now
        self._last_dx, self._last_dy = dx, dy

    # ---------------- FPS ----------------
    def _tick_fps(self):
        t = time.time(); dt = t - self._fps_t0
        if dt > 0:
            self.lbl_fps.setText(f"FPS: {self._fps_frames/dt:5.1f}")
        self._fps_frames = 0; self._fps_t0 = t

    # ---------------- 关闭 ----------------
    def closeEvent(self, e):
        if self.openmv_thread and self.openmv_thread.isRunning():
            self.openmv_thread.stop(); self.openmv_thread.wait(1000)
        super().closeEvent(e)


# ============================================================
def main():
    app = QApplication(sys.argv)
    app.setFont(QFont("Microsoft YaHei UI", 10))
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
