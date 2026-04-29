"""
Control LoRa + Camara serial (ESP32 gateway)
--------------------------------------------
- Control Xbox 360 para base movil
- Telemetria RPM y estado de enlace LoRa
- Stop de emergencia y comandos manuales
- Ventana de video (FRAM + len + JPEG) desde gateway serial

Requiere:
    pip install pyserial pygame PyQt5 opencv-python numpy
"""

import sys
import time
import queue
import struct
import threading

import cv2
import numpy as np
import pygame
import serial

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QLineEdit,
    QGroupBox,
)

# Silencia mensajes de libjpeg/OpenCV tipo:
# "Corrupt JPEG data: bad Huffman code"
try:
    cv2.setLogLevel(0)  # LOG_LEVEL_SILENT
except (AttributeError, TypeError):
    try:
        cv2.utils.logging.setLogLevel(0)  # type: ignore[attr-defined]
    except (AttributeError, TypeError):
        pass


# ------------------------- Config -------------------------
SERIAL_PORT = "COM3"        # ESP32 unica (LoRa control + video gateway)
SERIAL_BAUD = 921600

MAX_RPM = 120.0
DPAD_RPM = 80.0
DEAD_JOY = 0.08
DEAD_TRIG = 0.03
RESEND_EVERY = 8
DPAD_SWAP_LR = True

AX_JOY_IZQ_X = 0
AX_LT = 4
AX_RT = 5

MAGIC = b"FRAM"
MAX_FRAME_SIZE = 50000


# ------------------- Serial control (LoRa) -------------------
_ctrl_ser = None
_ctrl_ok = False
_lora_ok = False
_serial_tx_lock = threading.Lock()
_tx_queue = queue.Queue(maxsize=64)
_urgent_tx = queue.Queue(maxsize=32)

last_esp_echo = ""
lora_timeout = False
lora_fails = 0

tl_live = 0.0
tr_live = 0.0
rpm1_live = 0.0
rpm2_live = 0.0
rpm3_live = 0.0
rpm4_live = 0.0


def connect_ctrl_serial(port: str, baud: int, retries: int = 5) -> None:
    global _ctrl_ser, _ctrl_ok
    for i in range(retries):
        try:
            _ctrl_ser = serial.Serial(
                port,
                baud,
                timeout=0.08,
                write_timeout=2,
                dsrdtr=False,
                rtscts=False,
            )
            time.sleep(2.0)
            _ctrl_ok = True
            print(f"[CTRL] OK {port} @ {baud}")
            return
        except serial.SerialException as exc:
            print(f"[CTRL] intento {i+1}/{retries}: {exc}")
            time.sleep(2.0)
    print("[CTRL] no se pudo conectar (modo simulacion)")


def drain_tx_queue() -> None:
    while True:
        try:
            _tx_queue.get_nowait()
        except queue.Empty:
            break


def serial_send(cmd: str) -> None:
    if _ctrl_ok and _ctrl_ser:
        try:
            _tx_queue.put_nowait(cmd)
        except queue.Full:
            try:
                _tx_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                _tx_queue.put_nowait(cmd)
            except queue.Full:
                pass
    else:
        print(f"[SIM] {cmd}")


def serial_send_immediate(cmd: str) -> None:
    if not _ctrl_ok or _ctrl_ser is None:
        print(f"[SIM] {cmd}")
        return
    try:
        _urgent_tx.put_nowait(cmd)
    except queue.Full:
        try:
            _urgent_tx.get_nowait()
        except queue.Empty:
            pass
        try:
            _urgent_tx.put_nowait(cmd)
        except queue.Full:
            pass


def writer_thread() -> None:
    global _ctrl_ok, last_esp_echo
    while True:
        if not _ctrl_ok or _ctrl_ser is None:
            time.sleep(0.05)
            continue

        cmd = None
        urgent = False
        try:
            while True:
                cmd = _urgent_tx.get_nowait()
                urgent = True
        except queue.Empty:
            pass

        if not urgent:
            try:
                cmd = _tx_queue.get(timeout=0.08)
            except queue.Empty:
                continue

        try:
            if urgent:
                drain_tx_queue()
            with _serial_tx_lock:
                _ctrl_ser.write((cmd + "\n").encode("utf-8", errors="replace"))
                _ctrl_ser.flush()
            if urgent:
                last_esp_echo = f"-> {cmd}"[:180]
        except Exception as exc:
            print(f"[CTRL] TX error: {exc}")
            _ctrl_ok = False
            last_esp_echo = f"ERR serial TX: {exc}"


def reader_thread() -> None:
    global _lora_ok, lora_timeout, lora_fails, last_esp_echo
    global tl_live, tr_live, rpm1_live, rpm2_live, rpm3_live, rpm4_live

    while True:
        if not _ctrl_ok or _ctrl_ser is None:
            time.sleep(0.2)
            continue
        try:
            raw = _ctrl_ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            if "LORA_READY" in raw or "ANTENA" in raw:
                _lora_ok = True
            elif "ERR:LORA_TIMEOUT" in raw:
                lora_timeout = True
                lora_fails += 1
                last_esp_echo = raw[:180]
            elif raw.startswith("OK:"):
                lora_timeout = False
                last_esp_echo = raw[:180]
            elif "RPM1:" in raw or "TL:" in raw:
                lora_timeout = False
                vals = {}
                for tok in raw.split("\t"):
                    if ":" in tok:
                        k, v = tok.split(":", 1)
                        try:
                            vals[k.strip()] = float(v.strip())
                        except ValueError:
                            pass
                tl_live = vals.get("TL", tl_live)
                tr_live = vals.get("TR", tr_live)
                rpm1_live = vals.get("RPM1", rpm1_live)
                rpm2_live = vals.get("RPM2", rpm2_live)
                rpm3_live = vals.get("RPM3", rpm3_live)
                rpm4_live = vals.get("RPM4", rpm4_live)
            elif raw.startswith("FWD:"):
                last_esp_echo = raw[:180]
        except Exception:
            pass


# ---------------------- Video serial ----------------------
_cam_status = "Sin stream"
_last_frame = None
_last_cam_ts = 0.0
_cam_lock = threading.Lock()


def _parse_text_line(raw: str) -> None:
    global _lora_ok, lora_timeout, lora_fails, last_esp_echo
    global tl_live, tr_live, rpm1_live, rpm2_live, rpm3_live, rpm4_live
    if not raw:
        return
    if "LORA_READY" in raw or "ANTENA" in raw:
        _lora_ok = True
    elif "ERR:LORA_TIMEOUT" in raw:
        lora_timeout = True
        lora_fails += 1
        last_esp_echo = raw[:180]
    elif raw.startswith("OK:"):
        lora_timeout = False
        last_esp_echo = raw[:180]
    elif "RPM1:" in raw or "TL:" in raw:
        lora_timeout = False
        vals = {}
        for tok in raw.split("\t"):
            if ":" in tok:
                k, v = tok.split(":", 1)
                try:
                    vals[k.strip()] = float(v.strip())
                except ValueError:
                    pass
        tl_live = vals.get("TL", tl_live)
        tr_live = vals.get("TR", tr_live)
        rpm1_live = vals.get("RPM1", rpm1_live)
        rpm2_live = vals.get("RPM2", rpm2_live)
        rpm3_live = vals.get("RPM3", rpm3_live)
        rpm4_live = vals.get("RPM4", rpm4_live)
    elif raw.startswith("FWD:"):
        last_esp_echo = raw[:180]


def combined_reader_thread() -> None:
    global _cam_status, _last_frame, _last_cam_ts
    while True:
        if not _ctrl_ok or _ctrl_ser is None:
            time.sleep(0.2)
            continue
        try:
            chunk = _ctrl_ser.read(2048)
            if not chunk:
                _cam_status = "Esperando stream FRAM..."
                continue

            if not hasattr(combined_reader_thread, "buf"):
                combined_reader_thread.buf = b""
            combined_reader_thread.buf += chunk
            buf = combined_reader_thread.buf

            while buf:
                magic_idx = buf.find(MAGIC)
                nl_idx = buf.find(b"\n")

                # Prioridad: si hay una linea de texto antes de un frame, parsearla primero.
                if nl_idx != -1 and (magic_idx == -1 or nl_idx < magic_idx):
                    line = buf[:nl_idx].decode(errors="ignore").strip()
                    _parse_text_line(line)
                    buf = buf[nl_idx + 1 :]
                    continue

                if magic_idx == -1:
                    # Sin frame aun; evita crecer infinito y conserva cola para lineas futuras.
                    if len(buf) > 8192:
                        buf = buf[-8192:]
                    break

                # Desecha ruido/texto previo al magic.
                if magic_idx > 0:
                    prefix = buf[:magic_idx]
                    if b"\n" in prefix:
                        for ln in prefix.split(b"\n"):
                            _parse_text_line(ln.decode(errors="ignore").strip())
                    buf = buf[magic_idx:]
                    if len(buf) < 8:
                        break

                if len(buf) < 8:
                    break

                (frame_len,) = struct.unpack("<I", buf[4:8])
                if frame_len <= 0 or frame_len > MAX_FRAME_SIZE:
                    # Resincroniza corriendo un byte.
                    buf = buf[1:]
                    continue

                total = 8 + frame_len
                if len(buf) < total:
                    break

                jpeg = buf[8:total]
                buf = buf[total:]
                if len(jpeg) < 4:
                    continue
                if not (jpeg[0] == 0xFF and jpeg[1] == 0xD8 and jpeg[-2] == 0xFF and jpeg[-1] == 0xD9):
                    continue

                # Decodifica aqui: solo propagamos frames validos a la UI.
                arr = np.frombuffer(jpeg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None or frame.size == 0:
                    continue

                with _cam_lock:
                    _last_frame = frame
                    _last_cam_ts = time.time()
                    _cam_status = "Streaming"

            combined_reader_thread.buf = buf
        except Exception as exc:
            _cam_status = f"Cam error: {exc}"
            time.sleep(0.2)


# ---------------------- Xbox logic ----------------------
_last_l = None
_last_r = None
_last_dpad_cmd = None


def read_trigger(joy, axis: int) -> float:
    raw = max(0.0, min(1.0, joy.get_axis(axis)))
    if raw < DEAD_TRIG:
        return 0.0
    return (raw - DEAD_TRIG) / (1.0 - DEAD_TRIG) * MAX_RPM


def dead(val: float, zone: float) -> float:
    if abs(val) < zone:
        return 0.0
    sign = 1.0 if val > 0 else -1.0
    return sign * (abs(val) - zone) / (1.0 - zone)


def apply_steering(base_rpm: float, joy_x: float):
    joy_x = dead(joy_x, DEAD_JOY)
    if abs(base_rpm) < 1.0:
        spin = joy_x * MAX_RPM
        return -spin, spin
    if joy_x > 0:
        return base_rpm * (1.0 - 2.0 * joy_x), base_rpm
    return base_rpm, base_rpm * (1.0 - 2.0 * abs(joy_x))


def dpad_clear_if_active() -> None:
    global _last_dpad_cmd, _last_l, _last_r
    if _last_dpad_cmd is None:
        return
    serial_send_immediate("stop")
    _last_dpad_cmd = None
    _last_l = 0.0
    _last_r = 0.0


def dpad_send(name: str, force: bool = False) -> None:
    global _last_dpad_cmd, _last_l, _last_r
    if name != _last_dpad_cmd or force:
        serial_send_immediate(name)
        _last_dpad_cmd = name
        if name == "forward":
            _last_l = _last_r = float(DPAD_RPM)
        elif name == "back":
            _last_l = _last_r = -float(DPAD_RPM)
        elif name == "right":
            _last_l, _last_r = -float(DPAD_RPM), float(DPAD_RPM)
        elif name == "left":
            _last_l, _last_r = float(DPAD_RPM), -float(DPAD_RPM)


def set_tracks(l: float, r: float, force: bool = False) -> None:
    global _last_l, _last_r
    l = round(max(-MAX_RPM, min(MAX_RPM, l)), 1)
    r = round(max(-MAX_RPM, min(MAX_RPM, r)), 1)
    changed_l = force or _last_l is None or abs(l - (_last_l or 0.0)) >= 1.0
    changed_r = force or _last_r is None or abs(r - (_last_r or 0.0)) >= 1.0
    if changed_l or changed_r:
        if l == r:
            serial_send(f"lr={l}")
        else:
            if changed_l:
                serial_send(f"l={l}")
            if changed_r:
                serial_send(f"r={r}")
        _last_l = l
        _last_r = r


# -------------------------- UI --------------------------
def sbtn(text: str, bg: str, hv: str, mh: int = 36) -> QPushButton:
    b = QPushButton(text)
    b.setFont(QFont("Courier New", 10, QFont.Bold))
    b.setMinimumHeight(mh)
    b.setCursor(Qt.PointingHandCursor)
    b.setStyleSheet(
        f"QPushButton{{background:{bg};color:#f0f6fc;border:1px solid {hv};"
        f"border-radius:7px;padding:5px 14px;}}"
        f"QPushButton:hover{{background:{hv};}}"
        f"QPushButton:disabled{{background:#161b22;color:#484f58;}}"
    )
    return b


STYLE = (
    "QWidget{background:#0d1117;color:#c9d1d9;}"
    "QGroupBox{background:#161b22;border:1px solid #30363d;border-radius:8px;"
    "margin-top:10px;padding:10px 8px 8px 8px;color:#8b949e;"
    "font-family:'Courier New';font-size:9pt;}"
    "QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;}"
)


class Dashboard(QWidget):
    def __init__(self, joy):
        super().__init__()
        self.joy = joy
        self._cycle = 0
        self._mode = "DETENIDO"
        self._l_cmd = 0.0
        self._r_cmd = 0.0
        self._fps = 0.0
        self._fps_count = 0
        self._fps_t0 = time.time()

        self.setWindowTitle(f"Control LoRa + Camara | COM:{SERIAL_PORT}")
        self.setMinimumSize(1280, 760)
        self.setStyleSheet(STYLE)
        self._build_ui()

        self.t_ctrl = QTimer()
        self.t_ctrl.timeout.connect(self._ctrl_tick)
        self.t_ctrl.start(20)

        self.t_ui = QTimer()
        self.t_ui.timeout.connect(self._refresh_ui)
        self.t_ui.start(80)

        self.t_cam = QTimer()
        self.t_cam.timeout.connect(self._refresh_cam)
        self.t_cam.start(30)

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        top = QHBoxLayout()
        title = QLabel("BASE MOVIL 4 ORUGAS - CONTROL + VIDEO")
        title.setFont(QFont("Courier New", 13, QFont.Bold))
        title.setStyleSheet("color:#58a6ff;")
        top.addWidget(title)
        top.addStretch()
        self.conn_lbl = QLabel("CTRL: --")
        self.lora_lbl = QLabel("LoRa: --")
        self.cam_lbl = QLabel("Cam: --")
        for lbl in (self.conn_lbl, self.lora_lbl, self.cam_lbl):
            lbl.setFont(QFont("Courier New", 9, QFont.Bold))
            lbl.setStyleSheet("color:#8b949e;padding:3px 8px;border:1px solid #30363d;border-radius:5px;")
            top.addWidget(lbl)
        root.addLayout(top)

        body = QHBoxLayout()
        body.setSpacing(10)

        left = QVBoxLayout()
        left.setSpacing(8)

        g_ctrl = QGroupBox("Control / Seguridad")
        v1 = QVBoxLayout(g_ctrl)
        self.mode_lbl = QLabel("DETENIDO")
        self.mode_lbl.setFont(QFont("Courier New", 11, QFont.Bold))
        self.mode_lbl.setStyleSheet("color:#8b949e;background:#0d1117;border:1px solid #30363d;border-radius:8px;padding:8px;")
        v1.addWidget(self.mode_lbl)

        self.echo_lbl = QLabel("ESP32: --")
        self.echo_lbl.setWordWrap(True)
        self.echo_lbl.setFont(QFont("Courier New", 9))
        self.echo_lbl.setStyleSheet("color:#6b7fa3;")
        v1.addWidget(self.echo_lbl)

        row_cmd = QHBoxLayout()
        self.cmd_inp = QLineEdit()
        self.cmd_inp.setPlaceholderText("Comando manual: status, diag, enc, stop...")
        self.cmd_inp.returnPressed.connect(self._send_manual)
        send_btn = sbtn("Enviar", "#161b22", "#30363d", 30)
        send_btn.clicked.connect(self._send_manual)
        row_cmd.addWidget(self.cmd_inp)
        row_cmd.addWidget(send_btn)
        v1.addLayout(row_cmd)

        row_btn = QHBoxLayout()
        ping_btn = sbtn("Verificar LoRa (ping)", "#0e2a47", "#1f6feb")
        ping_btn.clicked.connect(lambda: serial_send_immediate("ping"))
        status_btn = sbtn("Pedir estado", "#0e2a47", "#1f6feb")
        status_btn.clicked.connect(lambda: serial_send_immediate("status"))
        stop_btn = sbtn("STOP EMERGENCIA", "#3d0c0c", "#f85149")
        stop_btn.clicked.connect(self._emergency_stop)
        row_btn.addWidget(ping_btn)
        row_btn.addWidget(status_btn)
        v1.addWidget(stop_btn)
        v1.addLayout(row_btn)

        left.addWidget(g_ctrl)

        g_rpm = QGroupBox("RPM motores")
        v2 = QVBoxLayout(g_rpm)
        self.rpm1_lbl = QLabel("M1: 0.0")
        self.rpm2_lbl = QLabel("M2: 0.0")
        self.rpm3_lbl = QLabel("M3: 0.0")
        self.rpm4_lbl = QLabel("M4: 0.0")
        for lbl in (self.rpm1_lbl, self.rpm2_lbl, self.rpm3_lbl, self.rpm4_lbl):
            lbl.setFont(QFont("Courier New", 11, QFont.Bold))
            lbl.setStyleSheet("color:#c9d1d9;")
            v2.addWidget(lbl)
        self.tgt_lbl = QLabel("Target L/R: 0.0 / 0.0")
        self.tgt_lbl.setFont(QFont("Courier New", 10))
        self.tgt_lbl.setStyleSheet("color:#8b949e;")
        v2.addWidget(self.tgt_lbl)
        left.addWidget(g_rpm)
        left.addStretch()

        body.addLayout(left, 36)

        g_cam = QGroupBox("Camara ESP32-S3 (gateway serial)")
        vcam = QVBoxLayout(g_cam)
        self.cam_view = QLabel("Sin video")
        self.cam_view.setAlignment(Qt.AlignCenter)
        self.cam_view.setMinimumSize(800, 540)
        self.cam_view.setStyleSheet("background:#000000;border:1px solid #30363d;border-radius:6px;")
        self.cam_info = QLabel("FPS: 0.0")
        self.cam_info.setFont(QFont("Courier New", 10, QFont.Bold))
        self.cam_info.setStyleSheet("color:#3fb950;")
        vcam.addWidget(self.cam_view)
        vcam.addWidget(self.cam_info)
        body.addWidget(g_cam, 64)

        root.addLayout(body)

    def _ctrl_tick(self):
        global _last_l, _last_r
        pygame.event.pump()
        self._cycle += 1
        force = self._cycle % RESEND_EVERY == 0

        rt = read_trigger(self.joy, AX_RT)
        lt = read_trigger(self.joy, AX_LT)
        jx = self.joy.get_axis(AX_JOY_IZQ_X)
        dp = self.joy.get_hat(0)

        use_steer = True
        base = 0.0

        if rt > 0.5 and lt > 0.5:
            dpad_clear_if_active()
            base = rt if rt >= lt else -lt
            self._mode = f"RT+LT {base:+.0f} RPM"
        elif rt > 0.5:
            dpad_clear_if_active()
            base = rt
            self._mode = f"RT +{rt:.0f} RPM"
        elif lt > 0.5:
            dpad_clear_if_active()
            base = -lt
            self._mode = f"LT {base:.0f} RPM"
        elif dp[1] == 1:
            dpad_send("forward", force=force)
            self._l_cmd, self._r_cmd = float(DPAD_RPM), float(DPAD_RPM)
            self._mode = "D-PAD FORWARD"
            use_steer = False
        elif dp[1] == -1:
            dpad_send("back", force=force)
            self._l_cmd, self._r_cmd = -float(DPAD_RPM), -float(DPAD_RPM)
            self._mode = "D-PAD BACK"
            use_steer = False
        elif dp[0] == -1:
            if DPAD_SWAP_LR:
                dpad_send("right", force=force)
                self._l_cmd, self._r_cmd = float(DPAD_RPM), -float(DPAD_RPM)
            else:
                dpad_send("left", force=force)
                self._l_cmd, self._r_cmd = -float(DPAD_RPM), float(DPAD_RPM)
            self._mode = "D-PAD LEFT"
            use_steer = False
        elif dp[0] == 1:
            if DPAD_SWAP_LR:
                dpad_send("left", force=force)
                self._l_cmd, self._r_cmd = -float(DPAD_RPM), float(DPAD_RPM)
            else:
                dpad_send("right", force=force)
                self._l_cmd, self._r_cmd = float(DPAD_RPM), -float(DPAD_RPM)
            self._mode = "D-PAD RIGHT"
            use_steer = False
        else:
            dpad_clear_if_active()
            if _last_l != 0.0 or _last_r != 0.0:
                la, ra = _last_l or 0.0, _last_r or 0.0
                if abs(la - ra) < 0.5:
                    serial_send_immediate("lr=0")
                else:
                    serial_send_immediate("l=0")
                    serial_send_immediate("r=0")
                _last_l = 0.0
                _last_r = 0.0
            self._l_cmd, self._r_cmd = 0.0, 0.0
            self._mode = "DETENIDO"
            use_steer = False

        if use_steer:
            lo, ro = apply_steering(base, jx)
            set_tracks(lo, ro, force=force)
            self._l_cmd, self._r_cmd = lo, ro

    def _send_manual(self):
        cmd = self.cmd_inp.text().strip()
        if cmd:
            serial_send_immediate(cmd)
            self.cmd_inp.clear()

    def _emergency_stop(self):
        global _last_dpad_cmd, _last_l, _last_r
        _last_dpad_cmd = None
        _last_l = 0.0
        _last_r = 0.0
        serial_send_immediate("stop")
        self._mode = "STOP EMERGENCIA"

    def _refresh_ui(self):
        self.echo_lbl.setText(f"ESP32: {last_esp_echo or '--'}")
        self.mode_lbl.setText(f"{self._mode}\nL={self._l_cmd:+.0f}  R={self._r_cmd:+.0f}")

        self.rpm1_lbl.setText(f"M1: {rpm1_live:+.1f} RPM")
        self.rpm2_lbl.setText(f"M2: {rpm2_live:+.1f} RPM")
        self.rpm3_lbl.setText(f"M3: {rpm3_live:+.1f} RPM")
        self.rpm4_lbl.setText(f"M4: {rpm4_live:+.1f} RPM")
        self.tgt_lbl.setText(f"Target L/R: {tl_live:+.1f} / {tr_live:+.1f}")

        if _ctrl_ok:
            self.conn_lbl.setText(f"CTRL OK {SERIAL_PORT}")
            self.conn_lbl.setStyleSheet("color:#3fb950;padding:3px 8px;border:1px solid #30363d;border-radius:5px;")
        else:
            self.conn_lbl.setText("CTRL sin serial")
            self.conn_lbl.setStyleSheet("color:#f85149;padding:3px 8px;border:1px solid #30363d;border-radius:5px;")

        if lora_timeout:
            self.lora_lbl.setText(f"LoRa TIMEOUT #{lora_fails}")
            self.lora_lbl.setStyleSheet("color:#f85149;padding:3px 8px;border:1px solid #30363d;border-radius:5px;")
        elif _lora_ok:
            self.lora_lbl.setText("LoRa OK")
            self.lora_lbl.setStyleSheet("color:#3fb950;padding:3px 8px;border:1px solid #30363d;border-radius:5px;")
        else:
            self.lora_lbl.setText("LoRa --")
            self.lora_lbl.setStyleSheet("color:#d29922;padding:3px 8px;border:1px solid #30363d;border-radius:5px;")

        self.cam_lbl.setText(f"Cam: {_cam_status}")
        self.cam_lbl.setStyleSheet("color:#58a6ff;padding:3px 8px;border:1px solid #30363d;border-radius:5px;")

    def _refresh_cam(self):
        global _last_frame
        with _cam_lock:
            frame = _last_frame
            ts = _last_cam_ts
        if frame is None:
            return

        self._fps_count += 1
        now = time.time()
        if now - self._fps_t0 >= 1.0:
            self._fps = self._fps_count / (now - self._fps_t0)
            self._fps_count = 0
            self._fps_t0 = now
        self.cam_info.setText(f"FPS: {self._fps:.2f} | Ultimo frame: {now - ts:.2f}s")

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(
            self.cam_view.width(),
            self.cam_view.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.cam_view.setPixmap(pix)

    def closeEvent(self, event):
        serial_send_immediate("stop")
        pygame.quit()
        super().closeEvent(event)


def main() -> None:
    connect_ctrl_serial(SERIAL_PORT, SERIAL_BAUD)

    threading.Thread(target=writer_thread, daemon=True).start()
    threading.Thread(target=combined_reader_thread, daemon=True).start()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("ERROR: No se detecto ningun control Xbox.")
        sys.exit(1)
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[XBOX] {joy.get_name()}")

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = Dashboard(joy)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
