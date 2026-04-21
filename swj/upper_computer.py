# -*- coding: utf-8 -*-
"""
STM32 目标追踪云台系统数据监控 (上位机)
- 左：OpenMV USB 实时图像
- 右：追踪数据（dx / dy 分开显示）
"""

import sys
import time
import math
import struct

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont, QMouseEvent
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QComboBox, QHBoxLayout, QVBoxLayout, QGroupBox,
    QFrame, QSizePolicy
)

import serial
import serial.tools.list_ports

# ---------------- 阈值参数（可按需调整）----------------
JUMP_TH_DIFF = 15          # QQVGA 分辨率下像素减半，跳变阈值同步缩小
LOCK_TH      = 5
IMG_W        = 160         # QQVGA
IMG_H        = 120
CENTER_X     = IMG_W // 2
CENTER_Y     = IMG_H // 2


# ============================================================
#                    OpenMV USB 串口读取线程（拉模式）
# ============================================================
class OpenMVReader(QThread):
    """
    协议：
      PC  -> OMV :  b'snap'
      OMV -> PC  :  Info(17) + JpegLen(4, <L) + JPEG bytes
        Info = <B h h h h h h h h>
               found, dx, dy, cx, cy, w, h, img_w, img_h
    """
    frame_received = pyqtSignal(dict)
    status         = pyqtSignal(str)

    INFO_SIZE = 17

    def __init__(self, port, baud=115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._running = True

    def _read_exact(self, ser, n, deadline):
        buf = bytearray()
        while len(buf) < n:
            if not self._running:
                return None
            if time.monotonic() > deadline:
                return None
            chunk = ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud,
                                timeout=0.1, write_timeout=0.5)
            try:
                ser.dtr = True
                ser.rts = True
            except Exception:
                pass
        except Exception as e:
            self.status.emit(f"[OpenMV] 打开失败: {e}")
            return
        self.status.emit(f"[OpenMV] 已连接 {self.port}")

        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass

        timeout_cnt = 0
        while self._running:
            try:
                ser.write(b'snap')
                try:
                    ser.flush()
                except Exception:
                    pass

                deadline = time.monotonic() + 1.0
                info = self._read_exact(ser, self.INFO_SIZE, deadline)
                if info is None:
                    timeout_cnt += 1
                    if timeout_cnt == 1 or timeout_cnt % 10 == 0:
                        self.status.emit(
                            f"[OpenMV] 无响应 {timeout_cnt} 次 "
                            f"(确认 OpenMV IDE 已关闭 / 串口是否正确)")
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    self.msleep(80)
                    continue
                timeout_cnt = 0

                found, dx, dy, cx, cy, bw, bh, img_w, img_h = struct.unpack(
                    "<Bhhhhhhhh", info)

                jpeg_len_bytes = self._read_exact(ser, 4, deadline + 0.5)
                if jpeg_len_bytes is None:
                    continue
                jpeg_len = struct.unpack("<I", jpeg_len_bytes)[0]
                if jpeg_len == 0 or jpeg_len > 1024 * 1024:
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    continue

                jpeg = self._read_exact(ser, jpeg_len, deadline + 2.0)
                if jpeg is None:
                    continue
                if len(jpeg) < 4 or jpeg[:2] != b'\xff\xd8':
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    continue

                self.frame_received.emit({
                    "jpeg": jpeg,
                    "found": found,
                    "dx": dx, "dy": dy,
                    "cx": cx, "cy": cy,
                    "w": bw, "h": bh,
                    "img_w": img_w, "img_h": img_h,
                })
            except Exception as e:
                self.status.emit(f"[OpenMV] 读取错误: {e}")
                break

        try:
            ser.close()
        except Exception:
            pass
        self.status.emit("[OpenMV] 已断开")

    def stop(self):
        self._running = False


# ============================================================
#                       自定义标题栏
# ============================================================
TITLE_BAR_H  = 52
BG_COLOR     = "#1b1d23"
BG_COLOR_ALT = "#2a2d34"
TITLE_BG     = "#1b1d23"   # 与主背景完全一致，整窗一体
BORDER_COLOR = "#333a44"
TEXT_COLOR   = "#e6edf3"
SUB_COLOR    = "#9aa5b1"
ACCENT       = "#2f81f7"


class TitleBar(QWidget):
    def __init__(self, parent, title_text):
        super().__init__(parent)
        self._win = parent
        self.setFixedHeight(TITLE_BAR_H)
        self.setAutoFillBackground(True)
        self.setStyleSheet(
            f"TitleBar,QWidget{{background:{TITLE_BG};}}"
            f"TitleBar{{border:none;}}")

        lay = QHBoxLayout(self)
        lay.setContentsMargins(16, 0, 0, 0)
        lay.setSpacing(0)

        # 左侧占位（和右侧 3 个按钮对称，使标题视觉上居中）
        left_spacer = QWidget()
        left_spacer.setFixedWidth(3 * 46 + 16)
        lay.addWidget(left_spacer)
        lay.addStretch(1)

        self.lbl_title = QLabel(title_text)
        self.lbl_title.setAlignment(Qt.AlignCenter)
        self.lbl_title.setStyleSheet(
            f"color:{TEXT_COLOR}; font-size:17px; font-weight:800;"
            f"letter-spacing:2px;")
        lay.addWidget(self.lbl_title)
        lay.addStretch(1)

        btn_style = (
            f"QPushButton{{background:transparent;color:{TEXT_COLOR};"
            f"border:none;font-family:'Segoe UI Symbol','Microsoft YaHei UI';"
            f"font-size:14px;font-weight:600;}}"
            f"QPushButton:hover{{background:#3a4050;color:#ffffff;}}"
            f"QPushButton#closeBtn:hover{{background:#e5484d;color:#ffffff;}}"
        )

        # 用字形稳定且不容易裁切的字符
        self.btn_min  = QPushButton("—")       # em dash
        self.btn_max  = QPushButton("□")       # white square
        self.btn_cls  = QPushButton("✕")       # multiplication X
        for b in (self.btn_min, self.btn_max, self.btn_cls):
            b.setFixedSize(46, TITLE_BAR_H)
            b.setStyleSheet(btn_style)
            b.setCursor(Qt.PointingHandCursor)
            b.setFocusPolicy(Qt.NoFocus)
        self.btn_cls.setObjectName("closeBtn")

        self.btn_min.clicked.connect(self._win.showMinimized)
        self.btn_max.clicked.connect(self._toggle_max)
        self.btn_cls.clicked.connect(self._win.close)

        lay.addWidget(self.btn_min)
        lay.addWidget(self.btn_max)
        lay.addWidget(self.btn_cls)

        self._drag_pos = None

    def _toggle_max(self):
        if self._win.isMaximized():
            self._win.showNormal()
        else:
            self._win.showMaximized()

    def mousePressEvent(self, e: QMouseEvent):
        if e.button() == Qt.LeftButton and not self._win.isMaximized():
            self._drag_pos = e.globalPos() - self._win.frameGeometry().topLeft()
            e.accept()

    def mouseMoveEvent(self, e: QMouseEvent):
        if self._drag_pos is not None and e.buttons() & Qt.LeftButton:
            self._win.move(e.globalPos() - self._drag_pos)
            e.accept()

    def mouseReleaseEvent(self, e: QMouseEvent):
        self._drag_pos = None

    def mouseDoubleClickEvent(self, e: QMouseEvent):
        if e.button() == Qt.LeftButton:
            self._toggle_max()


# ============================================================
#                          主窗口
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowFlag(Qt.FramelessWindowHint)
        self.setWindowTitle("STM32 目标追踪云台系统数据监控")
        self.resize(1480, 860)

        self.openmv_thread = None

        # 跳变检测
        self._last_dx = None
        self._last_dy = None
        self._last_ts = None
        self._prev_ts = None
        self._jump_start_ts = None
        self._in_tracking = False
        self._last_capture_ms = 0.0
        # 速度
        self._last_cx = None
        self._last_cy = None
        self._speed_px_s = 0.0

        self._build_ui()
        self.refresh_ports()

    # ---------------- UI ----------------
    def _build_ui(self):
        central = QWidget()
        central.setStyleSheet(f"background:{BG_COLOR};")
        self.setCentralWidget(central)

        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # 自定义标题栏
        self.title_bar = TitleBar(self, "STM32 目标追踪云台系统数据监控")
        outer.addWidget(self.title_bar)

        # 顶部导航：与标题栏共享背景色，视觉上接成一条
        top_card = QFrame()
        top_card.setObjectName("TopBar")
        top_card.setFixedHeight(60)
        top_card.setAutoFillBackground(True)
        top_card.setStyleSheet(
            f"QFrame#TopBar{{background:{TITLE_BG};border:none;"
            f"border-bottom:1px solid {BORDER_COLOR};}}"
            f"QFrame#TopBar QLabel{{background:transparent;}}")
        top = QHBoxLayout(top_card)
        top.setContentsMargins(20, 10, 20, 10)
        top.setSpacing(12)

        lbl_port = QLabel("🔌  OpenMV 端口")
        lbl_port.setStyleSheet(
            f"color:{TEXT_COLOR};font-size:15px;font-weight:700;"
            f"background:transparent;")
        top.addWidget(lbl_port)

        self.cb_openmv = QComboBox()
        self.cb_openmv.setMinimumWidth(340)
        self.cb_openmv.setFixedHeight(38)
        top.addWidget(self.cb_openmv)

        self.btn_refresh = QPushButton("↻  刷新串口")
        self.btn_conn_om = QPushButton("▶  连接 OpenMV")
        self.btn_refresh.setObjectName("BtnSecondary")
        self.btn_conn_om.setObjectName("BtnPrimary")
        self.btn_refresh.setFixedHeight(38)
        self.btn_conn_om.setFixedHeight(38)
        self.btn_refresh.setMinimumWidth(130)
        self.btn_conn_om.setMinimumWidth(170)
        self.btn_refresh.setCursor(Qt.PointingHandCursor)
        self.btn_conn_om.setCursor(Qt.PointingHandCursor)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn_om.clicked.connect(self.toggle_openmv)

        top.addWidget(self.btn_refresh)
        top.addWidget(self.btn_conn_om)
        top.addStretch(1)

        # 状态指示圆点 + 文字
        self.status_dot = QLabel("●")
        self.status_dot.setStyleSheet(
            "color:#f85149;font-size:16px;background:transparent;")
        self.lbl_status = QLabel("就绪")
        self.lbl_status.setStyleSheet(
            f"color:{TEXT_COLOR};font-size:14px;font-weight:600;"
            f"background:transparent;")
        top.addWidget(self.status_dot)
        top.addWidget(self.lbl_status)

        outer.addWidget(top_card)

        root = QVBoxLayout()
        root.setContentsMargins(16, 14, 16, 16)
        root.setSpacing(12)
        outer.addLayout(root, 1)

        # 主体
        body = QHBoxLayout()
        body.setSpacing(14)

        # --- 左：图像 ---
        left_box = QGroupBox("实时图像 (OpenMV USB)")
        left_lay = QVBoxLayout(left_box)
        self.lbl_image = QLabel("等待图像…")
        self.lbl_image.setAlignment(Qt.AlignCenter)
        self.lbl_image.setMinimumSize(900, 680)
        self.lbl_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.lbl_image.setStyleSheet(
            f"background-color:#101217;color:#888;"
            f"border:1px solid {BORDER_COLOR};border-radius:6px;")
        left_lay.addWidget(self.lbl_image, 1)
        self.lbl_fps = QLabel("FPS: --")
        self.lbl_fps.setStyleSheet("color:#6cf;font-weight:600;")
        left_lay.addWidget(self.lbl_fps, 0, Qt.AlignRight)
        body.addWidget(left_box, stretch=3)

        # --- 右：数据面板 ---
        right_box = QGroupBox("追踪数据")
        right_lay = QVBoxLayout(right_box)
        right_lay.setSpacing(10)

        self.val = {}

        def add_card(key, label_text, big=False):
            card = QFrame()
            card.setStyleSheet(
                f"QFrame{{background:{BG_COLOR_ALT};"
                f"border:1px solid {BORDER_COLOR};border-radius:10px;}}")
            lay = QVBoxLayout(card)
            lay.setContentsMargins(14, 10, 14, 10); lay.setSpacing(4)
            t = QLabel(label_text)
            t.setStyleSheet(f"color:{SUB_COLOR};font-size:12px;")
            v = QLabel("—")
            v.setStyleSheet(
                f"color:{TEXT_COLOR};font-size:{22 if big else 18}px;"
                f"font-weight:700;")
            lay.addWidget(t); lay.addWidget(v)
            self.val[key] = v
            return card

        # dx / dy 分开显示（并排两张大卡）
        dxdy_row = QHBoxLayout(); dxdy_row.setSpacing(10)
        dxdy_row.addWidget(add_card("dx", "x 轴偏移量 dx (px)", big=True))
        dxdy_row.addWidget(add_card("dy", "y 轴偏移量 dy (px)", big=True))
        right_lay.addLayout(dxdy_row)

        right_lay.addWidget(add_card("img_center", "图像中心点 (px)"))
        right_lay.addWidget(add_card("obj_center", "物体中心点 (px)"))
        right_lay.addWidget(add_card("found",      "是否检测到物体"))
        right_lay.addWidget(add_card("speed",      "物体移动速度 (px/s)"))
        right_lay.addWidget(add_card("capture",    "追上耗时 (ms) · 跳变起点方案"))

        right_lay.addStretch(1)
        body.addWidget(right_box, stretch=2)
        root.addLayout(body, 1)

        # 固定项
        self.val["img_center"].setText(f"({CENTER_X}, {CENTER_Y})")

        # FPS 定时
        self._fps_frames = 0
        self._fps_t0 = time.time()
        self._fps_timer = QTimer(self); self._fps_timer.timeout.connect(self._tick_fps)
        self._fps_timer.start(500)

        self._apply_style()

    def _apply_style(self):
        self.setStyleSheet(f"""
            QMainWindow {{ background:{BG_COLOR}; }}
            QGroupBox {{
                color:{TEXT_COLOR}; font-weight:700; font-size:15px;
                border:1px solid {BORDER_COLOR}; border-radius:12px;
                margin-top:14px; padding:10px;
                background:{BG_COLOR};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left:14px; padding:2px 10px;
                color:{TEXT_COLOR};
                background:{BG_COLOR_ALT};
                border:1px solid {BORDER_COLOR};
                border-radius:6px;
            }}
            QLabel {{ color:{TEXT_COLOR}; }}

            /* 主按钮：蓝色实心 */
            QPushButton#BtnPrimary {{
                background:#2f81f7; color:#ffffff; border:1px solid #2f81f7;
                border-radius:8px; padding:0 20px;
                font-weight:800; font-size:15px; letter-spacing:1px;
            }}
            QPushButton#BtnPrimary:hover  {{ background:#4593ff; border-color:#4593ff; }}
            QPushButton#BtnPrimary:pressed{{ background:#1f6feb; border-color:#1f6feb; }}

            /* 次要按钮：描边 */
            QPushButton#BtnSecondary {{
                background:{BG_COLOR}; color:{TEXT_COLOR};
                border:1.5px solid #3a4a66; border-radius:8px;
                padding:0 18px; font-weight:700; font-size:14px;
            }}
            QPushButton#BtnSecondary:hover  {{
                background:#2f81f7; color:#ffffff; border-color:#2f81f7;
            }}
            QPushButton#BtnSecondary:pressed{{ background:#1f6feb; border-color:#1f6feb; }}

            /* 默认按钮（厂备） */
            QPushButton {{
                background:#2f81f7; color:white; border:none; border-radius:6px;
                padding:6px 18px; font-weight:700; font-size:15px;
            }}
            QPushButton:hover  {{ background:#4593ff; }}
            QPushButton:pressed{{ background:#1f6feb; }}

            QComboBox {{
                background:{TITLE_BG}; color:{TEXT_COLOR};
                border:1.5px solid {BORDER_COLOR};
                border-radius:8px; padding:0 12px;
                font-size:14px; font-weight:600;
            }}
            QComboBox:hover {{ border-color:#2f81f7; }}
            QComboBox::drop-down {{
                border:none; width:24px;
            }}
            QComboBox QAbstractItemView {{
                background:{BG_COLOR_ALT}; color:{TEXT_COLOR};
                border:1px solid {BORDER_COLOR};
                selection-background-color:#2f81f7;
                selection-color:#ffffff;
                outline:0;
            }}
        """)

    # ---------------- 端口管理 ----------------
    def refresh_ports(self):
        ports = [p.device + " — " + (p.description or "")
                 for p in serial.tools.list_ports.comports()]
        self.cb_openmv.clear()
        if not ports:
            self.cb_openmv.addItem("(无串口)")
        else:
            self.cb_openmv.addItems(ports)
        self._set_status(f"找到 {len(ports)} 个串口")

    def _selected_port(self, combo):
        txt = combo.currentText()
        if "—" in txt:
            return txt.split("—", 1)[0].strip()
        return txt.strip() if txt else ""

    def _set_status(self, msg):
        self.lbl_status.setText(msg)

    # ---------------- 连接切换 ----------------
    def toggle_openmv(self):
        if self.openmv_thread and self.openmv_thread.isRunning():
            self.openmv_thread.stop(); self.openmv_thread.wait(1000)
            self.openmv_thread = None
            self.btn_conn_om.setText("▶  连接 OpenMV")
            self._set_dot(False)
            return
        port = self._selected_port(self.cb_openmv)
        if not port or port.startswith("("):
            self._set_status("请先选择 OpenMV 串口"); return
        self.openmv_thread = OpenMVReader(port)
        self.openmv_thread.frame_received.connect(self.on_openmv_frame)
        self.openmv_thread.status.connect(self._set_status)
        self.openmv_thread.start()
        self.btn_conn_om.setText("■  断开 OpenMV")
        self._set_dot(True)

    def _set_dot(self, connected):
        color = "#3fb950" if connected else "#f85149"
        self.status_dot.setStyleSheet(
            f"color:{color};font-size:18px;background:transparent;")

    # ---------------- 数据回调 ----------------
    def on_openmv_frame(self, f):
        # 1) 显示 JPEG —— 用 FastTransformation 减小缩放开销，FPS 明显提升
        img = QImage.fromData(f["jpeg"], "JPG")
        if not img.isNull():
            pm = QPixmap.fromImage(img).scaled(
                self.lbl_image.width(), self.lbl_image.height(),
                Qt.KeepAspectRatio, Qt.FastTransformation)
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

        # dx / dy 分开显示 + 着色
        def _dxy_color(v):
            if v > 0:  return "#58a6ff"
            if v < 0:  return "#f0883e"
            return TEXT_COLOR
        self.val["dx"].setText(f"{dx:+d}")
        self.val["dy"].setText(f"{dy:+d}")
        self.val["dx"].setStyleSheet(
            f"color:{_dxy_color(dx)};font-size:22px;font-weight:700;")
        self.val["dy"].setStyleSheet(
            f"color:{_dxy_color(dy)};font-size:22px;font-weight:700;")

        now = time.monotonic()

        # 速度（仅在连续找到目标时计算）
        if found and self._last_cx is not None and self._last_ts is not None:
            dt = now - self._last_ts
            if dt > 1e-4:
                dist = math.hypot(cx - self._last_cx, cy - self._last_cy)
                self._speed_px_s = dist / dt
        if found:
            self._last_cx, self._last_cy = cx, cy
        else:
            self._last_cx = self._last_cy = None
            self._speed_px_s = 0.0
        self.val["speed"].setText(f"{self._speed_px_s:7.1f}")

        # 跳变检测 & 捕获耗时
        if self._last_dx is not None:
            ddx = abs(dx - self._last_dx); ddy = abs(dy - self._last_dy)
            if (ddx > JUMP_TH_DIFF or ddy > JUMP_TH_DIFF):
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
# OpenMV 端分辨率 QVGA(320x240)
JUMP_TH_DIFF = 30   # 像素，帧间差分阈值（判定跳变）
LOCK_TH      = 10   # 像素，|dx|,|dy| 同时 <LOCK_TH 视为追上
IMG_W        = 320
IMG_H        = 240
CENTER_X     = IMG_W // 2
CENTER_Y     = IMG_H // 2

# ============================================================
#                    OpenMV USB 串口读取线程（拉模式）
# ============================================================
class OpenMVReader(QThread):
    """
    拉模式 USB VCP 协议：
      PC  → OMV :  b'snap'
      OMV → PC  :  Info(17) + JpegLen(4) + JPEG bytes
                   Info = <B h h h h h h h h>
                   -> found, dx, dy, cx, cy, w, h, img_w, img_h
    """
    frame_received = pyqtSignal(dict)       # {jpeg, found, dx, dy, cx, cy, w, h, img_w, img_h}
    status         = pyqtSignal(str)

    INFO_SIZE = 17

    def __init__(self, port, baud=115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._running = True

    def _read_exact(self, ser, n, deadline):
        """读满 n 字节；超时返回 None。"""
        buf = bytearray()
        while len(buf) < n:
            if not self._running:
                return None
            remain = deadline - time.monotonic()
            if remain <= 0:
                return None
            chunk = ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    def run(self):
        try:
            # 拉模式用较大的 read 超时，让 read_exact 自行按 deadline 控制
            ser = serial.Serial(self.port, self.baud, timeout=0.1,
                                write_timeout=0.5)
            # 显式拉 DTR/RTS，保证 OpenMV 的 usb.isconnected() 为 True
            try:
                ser.dtr = True
                ser.rts = True
            except Exception:
                pass
        except Exception as e:
            self.status.emit(f"[OpenMV] 打开失败: {e}")
            return
        self.status.emit(f"[OpenMV] 已连接 {self.port}（拉模式）")

        # 刚连上时把 OpenMV 可能缓存的老数据扔掉
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass

        timeout_cnt = 0
        while self._running:
            try:
                # 1) 发请求（3 位或 4 位 token）
                ser.write(b'snap')
                try:
                    ser.flush()
                except Exception:
                    pass
                # 2) 读 Info
                deadline = time.monotonic() + 1.0
                info = self._read_exact(ser, self.INFO_SIZE, deadline)
                if info is None:
                    timeout_cnt += 1
                    if timeout_cnt == 1 or timeout_cnt % 10 == 0:
                        self.status.emit(
                            f"[OpenMV] 无响应 {timeout_cnt} 次 "
                            f"(确认 OpenMV IDE 已关闭 / 串口是否正确)")
                    # 把可能残留的半包清掉，避免错位
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    self.msleep(80)
                    continue
                timeout_cnt = 0
                found, dx, dy, cx, cy, bw, bh, img_w, img_h = struct.unpack(
                    "<Bhhhhhhhh", info)
                # 3) 读 JpegLen
                jpeg_len_bytes = self._read_exact(ser, 4, deadline + 0.5)
                if jpeg_len_bytes is None:
                    self.status.emit("[OpenMV] 超时: 未收到 JpegLen，丢弃本帧")
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    continue
                jpeg_len = struct.unpack("<I", jpeg_len_bytes)[0]
                # 容错：非法长度（>1MB）直接丢弃并清空缓冲
                if jpeg_len == 0 or jpeg_len > 1024 * 1024:
                    self.status.emit(
                        f"[OpenMV] 非法 jpeg_len={jpeg_len}，重新同步")
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    continue
                # 4) 读 JPEG
                jpeg = self._read_exact(ser, jpeg_len, deadline + 2.0)
                if jpeg is None:
                    self.status.emit(
                        f"[OpenMV] 超时: 仅读到部分 JPEG ({jpeg_len}B)，丢弃")
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    continue
                # 简单合法性校验：JPEG 起始 FFD8
                if len(jpeg) < 4 or jpeg[:2] != b'\xff\xd8':
                    self.status.emit(
                        f"[OpenMV] JPEG 起始标记不匹配，重新同步")
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    continue

                self.frame_received.emit({
                    "jpeg": jpeg,
                    "found": found,
                    "dx": dx, "dy": dy,
                    "cx": cx, "cy": cy,
                    "w": bw, "h": bh,
                    "img_w": img_w, "img_h": img_h,
                })
            except Exception as e:
                self.status.emit(f"[OpenMV] 读取错误: {e}")
                break

        try:
            ser.close()
        except Exception:
            pass
        self.status.emit("[OpenMV] 已断开")

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
