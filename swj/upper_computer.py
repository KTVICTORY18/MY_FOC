# -*- coding: utf-8 -*-
"""
STM32 目标追踪云台系统数据监控 (上位机)
- 左：OpenMV USB 实时图像
- 右：追踪数据（dx / dy 分开显示）
- 电机控制页：速度控制、绝对/相对位置控制、校准
背面左边上下是绿黄，最下面两个，右边是倒数第四个开始红黑
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
    QFrame, QSizePolicy, QStackedWidget, QDoubleSpinBox,
    QSpinBox, QTextEdit, QScrollArea, QRadioButton, QButtonGroup
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

# ---------------- 颜色常量 ----------------
BG_COLOR     = "#1b1d23"
BG_COLOR_ALT = "#2a2d34"
TITLE_BG     = "#1b1d23"
BORDER_COLOR = "#333a44"
TEXT_COLOR   = "#e6edf3"
SUB_COLOR    = "#9aa5b1"
ACCENT       = "#2f81f7"

# ============================================================
#              电机控制协议（user_protocol.h 一一对应）
# ============================================================
PROTOCOL_HEADER = 0xAA
PROTOCOL_TAIL   = 0xFF

FUNC_CODE_SET_PARAMETER   = 0x01
FUNC_CODE_CALIBRATE       = 0x02
FUNC_CODE_MODIFY_ID       = 0x03
FUNC_CODE_ABS_ANGLE_SPEED = 0x04
FUNC_CODE_REL_ANGLE_SPEED = 0x05

FOC_MODE_LOW_SPEED_LOOP   = 0x00
FOC_MODE_STEP_ANGLE_LOOP  = 0x01
FOC_MODE_SPEED_LOOP       = 0x03
FOC_MODE_CURRENT_LOOP     = 0x04


def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


def build_set_parameter(motor_id: int, mode: int, value: float) -> bytes:
    payload = bytes([motor_id, FUNC_CODE_SET_PARAMETER, mode]) + struct.pack("<f", value)
    crc = crc8(payload)
    return bytes([PROTOCOL_HEADER]) + payload + bytes([crc, PROTOCOL_TAIL])


def build_calibrate(motor_id: int) -> bytes:
    payload = bytes([motor_id, FUNC_CODE_CALIBRATE])
    crc = crc8(payload)
    return bytes([PROTOCOL_HEADER]) + payload + bytes([crc, PROTOCOL_TAIL])


def build_abs_angle_speed(motor_id: int, angle_rad: float, speed_rpm: float) -> bytes:
    payload = bytes([motor_id, FUNC_CODE_ABS_ANGLE_SPEED]) + struct.pack("<ff", angle_rad, speed_rpm)
    crc = crc8(payload)
    return bytes([PROTOCOL_HEADER]) + payload + bytes([crc, PROTOCOL_TAIL])


def build_rel_angle_speed(motor_id: int, step_rad: float, speed_rpm: float) -> bytes:
    payload = bytes([motor_id, FUNC_CODE_REL_ANGLE_SPEED]) + struct.pack("<ff", step_rad, speed_rpm)
    crc = crc8(payload)
    return bytes([PROTOCOL_HEADER]) + payload + bytes([crc, PROTOCOL_TAIL])


# ============================================================
#                    电机串口发送线程
# ============================================================
class MotorSerial(QThread):
    status = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ser = None
        self._port = ""
        self._baud = 115200

    def connect(self, port: str, baud: int = 115200) -> bool:
        self.disconnect()
        try:
            self._ser = serial.Serial(port, baud, timeout=0.5, write_timeout=1.0)
            self._port = port
            self._baud = baud
            self.status.emit(f"[电机] 已连接 {port} @ {baud}bps")
            return True
        except Exception as e:
            self._ser = None
            self.status.emit(f"[电机] 连接失败: {e}")
            return False

    def disconnect(self):
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.status.emit("[电机] 已断开")

    def send(self, frame: bytes) -> bool:
        if not self._ser or not self._ser.is_open:
            self.status.emit("[电机] 未连接，无法发送")
            return False
        try:
            self._ser.write(frame)
            self._ser.flush()
            hex_str = " ".join(f"{b:02X}" for b in frame)
            self.status.emit(f"[电机] 发送: {hex_str}")
            return True
        except Exception as e:
            self.status.emit(f"[电机] 发送失败: {e}")
            return False

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open


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
        self.motor_serial  = MotorSerial(self)
        self.motor_serial.status.connect(self._on_motor_status)

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

        # ── Tab 导航栏 ──
        tab_bar = QFrame()
        tab_bar.setObjectName("TabBar")
        tab_bar.setFixedHeight(48)
        tab_bar.setStyleSheet(
            f"QFrame#TabBar{{background:{TITLE_BG};"
            f"border-bottom:1px solid {BORDER_COLOR};}}"
            f"QFrame#TabBar QLabel{{background:transparent;}}")
        tab_lay = QHBoxLayout(tab_bar)
        tab_lay.setContentsMargins(16, 4, 16, 4)
        tab_lay.setSpacing(6)

        self.btn_tab_monitor = QPushButton("📷  监控页面")
        self.btn_tab_motor   = QPushButton("⚙️  电机控制")
        for b in (self.btn_tab_monitor, self.btn_tab_motor):
            b.setFixedHeight(36)
            b.setMinimumWidth(150)
            b.setCursor(Qt.PointingHandCursor)
            b.setCheckable(True)
            b.setObjectName("TabBtn")
        self.btn_tab_monitor.setChecked(True)
        self.btn_tab_monitor.clicked.connect(lambda: self._switch_tab(0))
        self.btn_tab_motor.clicked.connect(lambda: self._switch_tab(1))
        tab_lay.addWidget(self.btn_tab_monitor)
        tab_lay.addWidget(self.btn_tab_motor)
        tab_lay.addStretch(1)

        # 状态指示（两个 Tab 共用）
        self.status_dot = QLabel("●")
        self.status_dot.setStyleSheet("color:#f85149;font-size:16px;background:transparent;")
        self.lbl_status = QLabel("就绪")
        self.lbl_status.setStyleSheet(
            f"color:{TEXT_COLOR};font-size:14px;font-weight:600;background:transparent;")
        tab_lay.addWidget(self.status_dot)
        tab_lay.addWidget(self.lbl_status)
        outer.addWidget(tab_bar)

        # ── 堆叠页面 ──
        self.stack = QStackedWidget()
        self.stack.addWidget(self._build_monitor_page())   # index 0
        self.stack.addWidget(self._build_motor_page())     # index 1
        outer.addWidget(self.stack, 1)

        self._apply_style()

    # ---------- 监控页面 ----------
    def _build_monitor_page(self):
        page = QWidget()
        page.setStyleSheet(f"background:{BG_COLOR};")
        root = QVBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # 顶部连接栏
        top_card = QFrame()
        top_card.setObjectName("TopBar")
        top_card.setFixedHeight(60)
        top_card.setStyleSheet(
            f"QFrame#TopBar{{background:{TITLE_BG};border:none;"
            f"border-bottom:1px solid {BORDER_COLOR};}}"
            f"QFrame#TopBar QLabel{{background:transparent;}}")
        top = QHBoxLayout(top_card)
        top.setContentsMargins(20, 10, 20, 10)
        top.setSpacing(12)

        lbl_port = QLabel("🔌  OpenMV 端口")
        lbl_port.setStyleSheet(
            f"color:{TEXT_COLOR};font-size:15px;font-weight:700;background:transparent;")
        top.addWidget(lbl_port)

        self.cb_openmv = QComboBox()
        self.cb_openmv.setMinimumWidth(340)
        self.cb_openmv.setFixedHeight(38)
        top.addWidget(self.cb_openmv)

        self.btn_refresh = QPushButton("↻  刷新串口")
        self.btn_conn_om = QPushButton("▶  连接 OpenMV")
        self.btn_refresh.setObjectName("BtnSecondary")
        self.btn_conn_om.setObjectName("BtnPrimary")
        self.btn_refresh.setFixedHeight(38); self.btn_conn_om.setFixedHeight(38)
        self.btn_refresh.setMinimumWidth(130); self.btn_conn_om.setMinimumWidth(170)
        self.btn_refresh.setCursor(Qt.PointingHandCursor)
        self.btn_conn_om.setCursor(Qt.PointingHandCursor)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn_om.clicked.connect(self.toggle_openmv)
        top.addWidget(self.btn_refresh)
        top.addWidget(self.btn_conn_om)
        top.addStretch(1)
        root.addWidget(top_card)

        body_w = QWidget()
        body_w.setStyleSheet(f"background:{BG_COLOR};")
        body = QHBoxLayout(body_w)
        body.setContentsMargins(16, 14, 16, 16)
        body.setSpacing(14)

        # 左：图像
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

        # 右：数据
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
            t = QLabel(label_text); t.setStyleSheet(f"color:{SUB_COLOR};font-size:12px;")
            v = QLabel("—"); v.setStyleSheet(
                f"color:{TEXT_COLOR};font-size:{22 if big else 18}px;font-weight:700;")
            lay.addWidget(t); lay.addWidget(v)
            self.val[key] = v
            return card

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

        root.addWidget(body_w, 1)

        self.val["img_center"].setText(f"({CENTER_X}, {CENTER_Y})")
        self._fps_frames = 0
        self._fps_t0 = time.time()
        self._fps_timer = QTimer(self); self._fps_timer.timeout.connect(self._tick_fps)
        self._fps_timer.start(500)

        return page

    # ---------- 电机控制页面 ----------
    def _build_motor_page(self):
        page = QWidget()
        page.setStyleSheet(f"background:{BG_COLOR};")
        root = QVBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── 顶部串口连接栏 ──
        conn_bar = QFrame()
        conn_bar.setObjectName("TopBar")
        conn_bar.setFixedHeight(60)
        conn_bar.setStyleSheet(
            f"QFrame#TopBar{{background:{TITLE_BG};border:none;"
            f"border-bottom:1px solid {BORDER_COLOR};}}"
            f"QFrame#TopBar QLabel{{background:transparent;}}")
        conn_lay = QHBoxLayout(conn_bar)
        conn_lay.setContentsMargins(20, 10, 20, 10)
        conn_lay.setSpacing(12)

        lbl_mport = QLabel("🔌  电机串口")
        lbl_mport.setStyleSheet(
            f"color:{TEXT_COLOR};font-size:15px;font-weight:700;background:transparent;")
        conn_lay.addWidget(lbl_mport)

        self.cb_motor_port = QComboBox()
        self.cb_motor_port.setMinimumWidth(280)
        self.cb_motor_port.setFixedHeight(38)
        conn_lay.addWidget(self.cb_motor_port)

        lbl_baud = QLabel("波特率")
        lbl_baud.setStyleSheet(f"color:{SUB_COLOR};font-size:14px;background:transparent;")
        self.cb_baud = QComboBox()
        for b in ["9600", "19200", "38400", "57600", "115200", "230400", "500000", "1000000"]:
            self.cb_baud.addItem(b)
        self.cb_baud.setCurrentText("115200")
        self.cb_baud.setFixedHeight(38)
        self.cb_baud.setMinimumWidth(110)
        conn_lay.addWidget(lbl_baud)
        conn_lay.addWidget(self.cb_baud)

        self.btn_motor_refresh = QPushButton("↻  刷新")
        self.btn_motor_conn    = QPushButton("▶  连接电机")
        self.btn_motor_refresh.setObjectName("BtnSecondary")
        self.btn_motor_conn.setObjectName("BtnPrimary")
        self.btn_motor_refresh.setFixedHeight(38); self.btn_motor_conn.setFixedHeight(38)
        self.btn_motor_refresh.setMinimumWidth(100); self.btn_motor_conn.setMinimumWidth(150)
        self.btn_motor_refresh.setCursor(Qt.PointingHandCursor)
        self.btn_motor_conn.setCursor(Qt.PointingHandCursor)
        self.btn_motor_refresh.clicked.connect(self.refresh_ports)
        self.btn_motor_conn.clicked.connect(self.toggle_motor)
        conn_lay.addWidget(self.btn_motor_refresh)
        conn_lay.addWidget(self.btn_motor_conn)
        conn_lay.addStretch(1)
        root.addWidget(conn_bar)

        # ── 主体：左控制面板 + 右日志 ──
        body_w = QWidget()
        body_w.setStyleSheet(f"background:{BG_COLOR};")
        body = QHBoxLayout(body_w)
        body.setContentsMargins(16, 14, 16, 16)
        body.setSpacing(14)

        # ─── 左：控制面板 ───
        ctrl_scroll = QScrollArea()
        ctrl_scroll.setWidgetResizable(True)
        ctrl_scroll.setStyleSheet(
            f"QScrollArea{{background:{BG_COLOR};border:none;}}"
            f"QScrollBar:vertical{{background:{BG_COLOR_ALT};width:8px;border-radius:4px;}}"
            f"QScrollBar::handle:vertical{{background:#3a4a66;border-radius:4px;}}")

        ctrl_inner = QWidget()
        ctrl_inner.setStyleSheet(f"background:{BG_COLOR};")
        ctrl_v = QVBoxLayout(ctrl_inner)
        ctrl_v.setContentsMargins(4, 4, 4, 4)
        ctrl_v.setSpacing(14)

        # ── 公共参数（电机ID）──
        id_box = QGroupBox("公共参数")
        id_lay = QHBoxLayout(id_box)
        id_lay.setSpacing(12)
        id_lay.addWidget(QLabel("电机 ID"))
        self.spin_motor_id = QSpinBox()
        self.spin_motor_id.setRange(0, 255)
        self.spin_motor_id.setValue(1)
        self.spin_motor_id.setFixedHeight(36)
        self.spin_motor_id.setMinimumWidth(80)
        id_lay.addWidget(self.spin_motor_id)
        id_lay.addStretch(1)
        ctrl_v.addWidget(id_box)

        # ── 控制模式选择 ──
        mode_box = QGroupBox("控制模式")
        mode_v = QVBoxLayout(mode_box)
        mode_v.setSpacing(6)
        self._mode_group = QButtonGroup(self)
        mode_items = [
            ("speed_loop",     "速度环控制  (0x01 + Mode=0x03)"),
            ("low_speed_loop", "低速环控制  (0x01 + Mode=0x00)"),
            ("abs_pos",        "绝对位置控制 (0x04)"),
            ("rel_pos",        "相对位置控制 (0x05)"),
            ("calibrate",      "校准触发     (0x02)"),
        ]
        self._mode_radios = {}
        for key, label in mode_items:
            rb = QRadioButton(label)
            rb.setStyleSheet(f"color:{TEXT_COLOR};font-size:14px;")
            self._mode_group.addButton(rb)
            self._mode_radios[key] = rb
            mode_v.addWidget(rb)
        self._mode_radios["speed_loop"].setChecked(True)
        ctrl_v.addWidget(mode_box)

        # ── 速度环参数 ──
        self._grp_speed = QGroupBox("速度环参数")
        sp_lay = QHBoxLayout(self._grp_speed)
        sp_lay.setSpacing(12)
        sp_lay.addWidget(QLabel("目标速度 (rpm)"))
        self.spin_speed_rpm = QDoubleSpinBox()
        self.spin_speed_rpm.setRange(-1000.0, 1000.0)
        self.spin_speed_rpm.setValue(30.0)
        self.spin_speed_rpm.setDecimals(1)
        self.spin_speed_rpm.setSingleStep(5.0)
        self.spin_speed_rpm.setFixedHeight(36)
        self.spin_speed_rpm.setMinimumWidth(120)
        sp_lay.addWidget(self.spin_speed_rpm)
        sp_lay.addStretch(1)
        ctrl_v.addWidget(self._grp_speed)

        # ── 低速环参数 ──
        self._grp_low_speed = QGroupBox("低速环参数")
        ls_lay = QHBoxLayout(self._grp_low_speed)
        ls_lay.setSpacing(12)
        ls_lay.addWidget(QLabel("目标速度 (rpm)"))
        self.spin_low_speed_rpm = QDoubleSpinBox()
        self.spin_low_speed_rpm.setRange(-200.0, 200.0)
        self.spin_low_speed_rpm.setValue(10.0)
        self.spin_low_speed_rpm.setDecimals(1)
        self.spin_low_speed_rpm.setSingleStep(1.0)
        self.spin_low_speed_rpm.setFixedHeight(36)
        self.spin_low_speed_rpm.setMinimumWidth(120)
        ls_lay.addWidget(self.spin_low_speed_rpm)
        ls_lay.addStretch(1)
        ctrl_v.addWidget(self._grp_low_speed)

        # ── 绝对位置参数 ──
        self._grp_abs = QGroupBox("绝对位置参数")
        abs_lay = QHBoxLayout(self._grp_abs)
        abs_lay.setSpacing(12)
        abs_lay.addWidget(QLabel("目标角度 (°)"))
        self.spin_abs_angle_deg = QDoubleSpinBox()
        self.spin_abs_angle_deg.setRange(0.0, 360.0)
        self.spin_abs_angle_deg.setValue(180.0)
        self.spin_abs_angle_deg.setDecimals(1)
        self.spin_abs_angle_deg.setSingleStep(5.0)
        self.spin_abs_angle_deg.setFixedHeight(36)
        self.spin_abs_angle_deg.setMinimumWidth(110)
        abs_lay.addWidget(self.spin_abs_angle_deg)
        abs_lay.addWidget(QLabel("运动速度 (rpm)"))
        self.spin_abs_speed = QDoubleSpinBox()
        self.spin_abs_speed.setRange(0.1, 1000.0)
        self.spin_abs_speed.setValue(20.0)
        self.spin_abs_speed.setDecimals(1)
        self.spin_abs_speed.setSingleStep(5.0)
        self.spin_abs_speed.setFixedHeight(36)
        self.spin_abs_speed.setMinimumWidth(110)
        abs_lay.addWidget(self.spin_abs_speed)
        abs_lay.addStretch(1)
        ctrl_v.addWidget(self._grp_abs)

        # ── 相对位置参数 ──
        self._grp_rel = QGroupBox("相对位置参数")
        rel_lay = QHBoxLayout(self._grp_rel)
        rel_lay.setSpacing(12)
        rel_lay.addWidget(QLabel("步进角度 (°)  正=正转 负=反转"))
        self.spin_rel_angle_deg = QDoubleSpinBox()
        self.spin_rel_angle_deg.setRange(-3600.0, 3600.0)
        self.spin_rel_angle_deg.setValue(90.0)
        self.spin_rel_angle_deg.setDecimals(1)
        self.spin_rel_angle_deg.setSingleStep(5.0)
        self.spin_rel_angle_deg.setFixedHeight(36)
        self.spin_rel_angle_deg.setMinimumWidth(110)
        rel_lay.addWidget(self.spin_rel_angle_deg)
        rel_lay.addWidget(QLabel("运动速度 (rpm)"))
        self.spin_rel_speed = QDoubleSpinBox()
        self.spin_rel_speed.setRange(0.1, 1000.0)
        self.spin_rel_speed.setValue(30.0)
        self.spin_rel_speed.setDecimals(1)
        self.spin_rel_speed.setSingleStep(5.0)
        self.spin_rel_speed.setFixedHeight(36)
        self.spin_rel_speed.setMinimumWidth(110)
        rel_lay.addWidget(self.spin_rel_speed)
        rel_lay.addStretch(1)
        ctrl_v.addWidget(self._grp_rel)

        # ── 校准说明 ──
        self._grp_calib = QGroupBox("校准")
        calib_lay = QVBoxLayout(self._grp_calib)
        calib_hint = QLabel(
            "点击发送后将触发 FOC 校准并保存到 Flash。\n"
            "校准期间电机会旋转，请确保无负载且处于安全状态。")
        calib_hint.setStyleSheet(f"color:{SUB_COLOR};font-size:13px;")
        calib_hint.setWordWrap(True)
        calib_lay.addWidget(calib_hint)
        ctrl_v.addWidget(self._grp_calib)

        # 默认只显示当前模式对应的参数组
        self._update_param_visibility()
        for rb in self._mode_radios.values():
            rb.toggled.connect(self._update_param_visibility)

        # ── 发送按钮 ──
        self.btn_motor_send = QPushButton("▶  发送控制指令")
        self.btn_motor_send.setObjectName("BtnPrimary")
        self.btn_motor_send.setFixedHeight(44)
        self.btn_motor_send.setMinimumWidth(200)
        self.btn_motor_send.setCursor(Qt.PointingHandCursor)
        self.btn_motor_send.clicked.connect(self._on_motor_send)
        ctrl_v.addWidget(self.btn_motor_send)

        ctrl_v.addStretch(1)
        ctrl_scroll.setWidget(ctrl_inner)
        body.addWidget(ctrl_scroll, stretch=3)

        # ─── 右：日志 ───
        log_box = QGroupBox("串口日志")
        log_lay = QVBoxLayout(log_box)
        self.motor_log = QTextEdit()
        self.motor_log.setReadOnly(True)
        self.motor_log.setStyleSheet(
            f"background:{BG_COLOR_ALT};color:{TEXT_COLOR};"
            f"border:1px solid {BORDER_COLOR};border-radius:6px;"
            f"font-family:'Consolas','Courier New';font-size:13px;")
        log_lay.addWidget(self.motor_log)
        btn_clear_log = QPushButton("清空日志")
        btn_clear_log.setObjectName("BtnSecondary")
        btn_clear_log.setFixedHeight(32)
        btn_clear_log.clicked.connect(self.motor_log.clear)
        log_lay.addWidget(btn_clear_log, 0, Qt.AlignRight)
        body.addWidget(log_box, stretch=2)

        root.addWidget(body_w, 1)
        return page

    def _update_param_visibility(self):
        mode = self._current_motor_mode()
        self._grp_speed.setVisible(mode == "speed_loop")
        self._grp_low_speed.setVisible(mode == "low_speed_loop")
        self._grp_abs.setVisible(mode == "abs_pos")
        self._grp_rel.setVisible(mode == "rel_pos")
        self._grp_calib.setVisible(mode == "calibrate")

    def _current_motor_mode(self) -> str:
        for key, rb in self._mode_radios.items():
            if rb.isChecked():
                return key
        return "speed_loop"

    # ---------- Tab 切换 ----------
    def _switch_tab(self, idx: int):
        self.stack.setCurrentIndex(idx)
        self.btn_tab_monitor.setChecked(idx == 0)
        self.btn_tab_motor.setChecked(idx == 1)
        if idx == 1:
            self._sync_motor_port_combo()

    def _sync_motor_port_combo(self):
        """把监控页的串口列表同步到电机页。"""
        items = [self.cb_openmv.itemText(i) for i in range(self.cb_openmv.count())]
        self.cb_motor_port.clear()
        if items:
            self.cb_motor_port.addItems(items)
        else:
            self.cb_motor_port.addItem("(无串口)")

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

            QPushButton#TabBtn {{
                background:transparent; color:{SUB_COLOR};
                border:none; border-radius:8px;
                padding:0 16px; font-weight:700; font-size:14px;
            }}
            QPushButton#TabBtn:hover   {{ background:{BG_COLOR_ALT}; color:{TEXT_COLOR}; }}
            QPushButton#TabBtn:checked {{
                background:{ACCENT}; color:#ffffff;
            }}

            QPushButton#BtnPrimary {{
                background:#2f81f7; color:#ffffff; border:1px solid #2f81f7;
                border-radius:8px; padding:0 20px;
                font-weight:800; font-size:15px; letter-spacing:1px;
            }}
            QPushButton#BtnPrimary:hover   {{ background:#4593ff; border-color:#4593ff; }}
            QPushButton#BtnPrimary:pressed {{ background:#1f6feb; border-color:#1f6feb; }}

            QPushButton#BtnSecondary {{
                background:{BG_COLOR}; color:{TEXT_COLOR};
                border:1.5px solid #3a4a66; border-radius:8px;
                padding:0 18px; font-weight:700; font-size:14px;
            }}
            QPushButton#BtnSecondary:hover   {{ background:#2f81f7; color:#ffffff; border-color:#2f81f7; }}
            QPushButton#BtnSecondary:pressed {{ background:#1f6feb; border-color:#1f6feb; }}

            QPushButton {{
                background:#2f81f7; color:white; border:none; border-radius:6px;
                padding:6px 18px; font-weight:700; font-size:15px;
            }}
            QPushButton:hover   {{ background:#4593ff; }}
            QPushButton:pressed {{ background:#1f6feb; }}

            QComboBox {{
                background:{TITLE_BG}; color:{TEXT_COLOR};
                border:1.5px solid {BORDER_COLOR};
                border-radius:8px; padding:0 12px;
                font-size:14px; font-weight:600;
            }}
            QComboBox:hover {{ border-color:#2f81f7; }}
            QComboBox::drop-down {{ border:none; width:24px; }}
            QComboBox QAbstractItemView {{
                background:{BG_COLOR_ALT}; color:{TEXT_COLOR};
                border:1px solid {BORDER_COLOR};
                selection-background-color:#2f81f7;
                selection-color:#ffffff;
                outline:0;
            }}

            QDoubleSpinBox, QSpinBox {{
                background:{BG_COLOR_ALT}; color:{TEXT_COLOR};
                border:1.5px solid {BORDER_COLOR}; border-radius:6px;
                padding:0 8px; font-size:14px;
            }}
            QDoubleSpinBox:focus, QSpinBox:focus {{ border-color:#2f81f7; }}

            QRadioButton::indicator {{
                width:16px; height:16px;
                border:2px solid {BORDER_COLOR}; border-radius:8px;
                background:{BG_COLOR_ALT};
            }}
            QRadioButton::indicator:checked {{
                background:{ACCENT}; border-color:{ACCENT};
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
        self._sync_motor_port_combo()
        self._set_status(f"找到 {len(ports)} 个串口")

    def _selected_port(self, combo):
        txt = combo.currentText()
        if "—" in txt:
            return txt.split("—", 1)[0].strip()
        return txt.strip() if txt else ""

    def _set_status(self, msg):
        self.lbl_status.setText(msg)

    # ---------------- OpenMV 连接切换 ----------------
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

    # ---------------- 电机串口连接切换 ----------------
    def toggle_motor(self):
        if self.motor_serial.is_connected:
            self.motor_serial.disconnect()
            self.btn_motor_conn.setText("▶  连接电机")
            self._set_dot(False)
            return
        port = self._selected_port(self.cb_motor_port)
        if not port or port.startswith("("):
            self._on_motor_status("请先选择电机串口"); return
        baud = int(self.cb_baud.currentText())
        ok = self.motor_serial.connect(port, baud)
        if ok:
            self.btn_motor_conn.setText("■  断开电机")
            self._set_dot(True)

    def _on_motor_status(self, msg: str):
        self._set_status(msg)
        self.motor_log.append(msg)

    # ---------------- 电机指令发送 ----------------
    def _on_motor_send(self):
        motor_id = self.spin_motor_id.value()
        mode = self._current_motor_mode()

        if mode == "speed_loop":
            rpm = self.spin_speed_rpm.value()
            frame = build_set_parameter(motor_id, FOC_MODE_SPEED_LOOP, rpm)
        elif mode == "low_speed_loop":
            rpm = self.spin_low_speed_rpm.value()
            frame = build_set_parameter(motor_id, FOC_MODE_LOW_SPEED_LOOP, rpm)
        elif mode == "abs_pos":
            angle_rad = math.radians(self.spin_abs_angle_deg.value())
            speed_rpm = self.spin_abs_speed.value()
            frame = build_abs_angle_speed(motor_id, angle_rad, speed_rpm)
        elif mode == "rel_pos":
            step_rad  = math.radians(self.spin_rel_angle_deg.value())
            speed_rpm = self.spin_rel_speed.value()
            frame = build_rel_angle_speed(motor_id, step_rad, speed_rpm)
        elif mode == "calibrate":
            frame = build_calibrate(motor_id)
        else:
            return

        self.motor_serial.send(frame)

    # ---------------- 数据回调 ----------------
    def on_openmv_frame(self, f):
        img = QImage.fromData(f["jpeg"], "JPG")
        if not img.isNull():
            pm = QPixmap.fromImage(img).scaled(
                self.lbl_image.width(), self.lbl_image.height(),
                Qt.KeepAspectRatio, Qt.FastTransformation)
            self.lbl_image.setPixmap(pm)
            self._fps_frames += 1

        found = f["found"]; dx = f["dx"]; dy = f["dy"]
        cx = f["cx"]; cy = f["cy"]

        self.val["obj_center"].setText(f"({cx}, {cy})" if found else "—")
        self.val["found"].setText("✅ FOUND" if found else "❌ LOST")
        self.val["found"].setStyleSheet(
            "color:#3fb950;font-size:18px;font-weight:700;" if found
            else "color:#f85149;font-size:18px;font-weight:700;")

        def _dxy_color(v):
            if v > 0: return "#58a6ff"
            if v < 0: return "#f0883e"
            return TEXT_COLOR
        self.val["dx"].setText(f"{dx:+d}")
        self.val["dy"].setText(f"{dy:+d}")
        self.val["dx"].setStyleSheet(f"color:{_dxy_color(dx)};font-size:22px;font-weight:700;")
        self.val["dy"].setStyleSheet(f"color:{_dxy_color(dy)};font-size:22px;font-weight:700;")

        now = time.monotonic()
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

        if self._last_dx is not None:
            ddx = abs(dx - self._last_dx); ddy = abs(dy - self._last_dy)
            if ddx > JUMP_TH_DIFF or ddy > JUMP_TH_DIFF:
                self._jump_start_ts = self._prev_ts if self._prev_ts is not None else self._last_ts
                self._in_tracking = True

        if self._in_tracking and self._jump_start_ts is not None:
            if abs(dx) < LOCK_TH and abs(dy) < LOCK_TH and found:
                self._last_capture_ms = (now - self._jump_start_ts) * 1000.0
                self._in_tracking = False
                self._jump_start_ts = None

        status_txt = "追踪中…" if self._in_tracking else "已锁定"
        self.val["capture"].setText(f"{self._last_capture_ms:7.1f}   ({status_txt})")

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
        if self.motor_serial.is_connected:
            self.motor_serial.disconnect()
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
