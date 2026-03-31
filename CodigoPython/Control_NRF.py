"""
Dashboard Python — Robot 4 Motores  ·  Control inalámbrico
===========================================================
PC  →  Serial USB  →  ESP32  →  nRF24L01 RF  →  Arduino Mega

Cambios respecto a la versión con cable:
  - Se conecta al puerto COM de la ESP32 (no del Mega)
  - La ESP32 reenvía cada comando por RF al Mega
  - El ACK del nRF confirma que el Mega recibió el paquete

Requisitos:
  pip install pyserial pygame PyQt5 matplotlib

Pasos:
  1. Sube esp32_transmisor_nrf24.ino a la ESP32
  2. Sube mega_receptor_nrf24.ino al Arduino Mega
  3. Conecta la ESP32 al PC por USB
  4. Cambia SERIAL_PORT al COM de la ESP32
  5. Conecta el control Xbox 360
  6. Ejecuta este script
"""

import sys, serial, threading, time
from collections import deque

import pygame
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QCheckBox, QGroupBox, QTabWidget)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.gridspec as gridspec

# ═══════════════════════════════════════════════════════
#  CONFIGURACIÓN
# ═══════════════════════════════════════════════════════
SERIAL_PORT  = "COM3"      # Puerto COM de la ESP32
SERIAL_BAUD  = 115200

MAX_RPM      = 150.0
DPAD_RPM     = 100.0
DEAD_JOY     = 0.08
DEAD_TRIG    = 0.03

# Cada cuántos ciclos (50 Hz) se reenvía aunque no haya cambio
# Mantiene vivo el watchdog del ESP32 y del Mega
RESEND_EVERY = 8           # cada ~160 ms

AX_JOY_IZQ_X = 0
AX_LT        = 4
AX_RT        = 5

# ═══════════════════════════════════════════════════════
#  SERIAL  (conectado a la ESP32)
# ═══════════════════════════════════════════════════════
_ser       = None
_serial_ok = False
_rf_ok     = False         # True si la ESP32 reporta nRF24 listo
_ack_ok    = 0             # contador de ACKs recibidos
_ack_fail  = 0             # contador de fallos RF

def connect_serial(port, baud, retries=5):
    global _ser, _serial_ok
    for i in range(retries):
        try:
            _ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
            _serial_ok = True
            print(f"✓ Serial conectado a ESP32  {port}")
            return
        except serial.SerialException as e:
            print(f"  Intento {i+1}/{retries}: {e}")
            time.sleep(2)
    print("  No se pudo conectar — modo simulación")

def serial_send(cmd):
    if _serial_ok and _ser:
        try:
            _ser.write((cmd + "\n").encode())
        except Exception as e:
            print(f"  Serial error: {e}")
    else:
        print(f"[SIM] {cmd}")

# ═══════════════════════════════════════════════════════
#  LECTOR SERIAL — respuestas de la ESP32
#  La ESP32 responde:
#    "OK lr=80.0"   → comando enviado y ACK recibido
#    "FAIL lr=80.0" → comando enviado pero sin ACK (RF fail)
#    "WD: ..."      → watchdog disparado
#    "OK: ..."      → mensajes de inicio
# ═══════════════════════════════════════════════════════
_last_esp_msg  = ""
_rf_link_ok    = True

def reader_thread():
    global _rf_ok, _last_esp_msg, _ack_ok, _ack_fail, _rf_link_ok
    while True:
        if not _serial_ok or _ser is None:
            time.sleep(0.2); continue
        try:
            raw = _ser.readline().decode(errors="ignore").strip()
            if not raw: continue
            _last_esp_msg = raw

            if raw.startswith("OK:") and "nRF24" in raw:
                _rf_ok = True
            elif raw.startswith("OK "):
                _ack_ok += 1
                _rf_link_ok = True
            elif raw.startswith("FAIL "):
                _ack_fail += 1
                _rf_link_ok = False
                print(f"  RF FAIL: {raw}")
            elif raw.startswith("WD:"):
                print(f"  Watchdog ESP32: {raw}")
            elif raw.startswith("ERR"):
                print(f"  ESP32 error: {raw}")
        except Exception:
            pass

# ═══════════════════════════════════════════════════════
#  LÓGICA DE CONTROL
# ═══════════════════════════════════════════════════════
_last_L = None
_last_R = None

def set_orugas(left: float, right: float, force: bool = False):
    global _last_L, _last_R
    left  = round(max(-MAX_RPM, min(MAX_RPM, left)),  1)
    right = round(max(-MAX_RPM, min(MAX_RPM, right)), 1)
    active = abs(left) > 0.5 or abs(right) > 0.5

    # Si ambas orugas tienen el mismo valor → usar comando lr= (un solo paquete RF)
    if abs(left - right) < 0.5 and (force or active or _last_L is None):
        serial_send(f"lr={left}")
        _last_L = left; _last_R = right
        return

    if force or active or _last_L is None or abs(left  - _last_L) >= 1.0:
        serial_send(f"l={left}");  _last_L = left
    if force or active or _last_R is None or abs(right - _last_R) >= 1.0:
        serial_send(f"r={right}"); _last_R = right

def read_trigger(joy, axis):
    raw = max(0.0, min(1.0, joy.get_axis(axis)))
    if raw < DEAD_TRIG: return 0.0
    return (raw - DEAD_TRIG) / (1.0 - DEAD_TRIG) * MAX_RPM

def dead(val, zone):
    if abs(val) < zone: return 0.0
    sign = 1.0 if val > 0 else -1.0
    return sign * (abs(val) - zone) / (1.0 - zone)

def apply_steering(base_rpm: float, joy_x: float):
    joy_x = dead(joy_x, DEAD_JOY)
    if abs(base_rpm) < 1.0:
        spin = joy_x * MAX_RPM
        return -spin, spin
    if joy_x > 0:
        return base_rpm * (1.0 - 2.0 * joy_x), base_rpm
    else:
        return base_rpm, base_rpm * (1.0 - 2.0 * abs(joy_x))

# ═══════════════════════════════════════════════════════
#  HELPERS UI
# ═══════════════════════════════════════════════════════
STYLE = (
    "QWidget{background:#0d1117;color:#c9d1d9;}"
    "QGroupBox{background:#161b22;border:1px solid #30363d;border-radius:8px;"
    "margin-top:10px;padding:10px 8px 8px 8px;color:#8b949e;"
    "font-family:'Courier New';font-size:9pt;}"
    "QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;}"
    "QTabWidget::pane{border:1px solid #30363d;background:#161b22;"
    "border-radius:0 6px 6px 6px;}"
    "QTabBar::tab{background:#0d1117;color:#8b949e;padding:8px 14px;"
    "font-family:'Courier New';font-size:9pt;border:1px solid #30363d;"
    "border-bottom:none;border-radius:6px 6px 0 0;margin-right:2px;}"
    "QTabBar::tab:selected{background:#161b22;color:#f0f6fc;border-color:#58a6ff;}"
    "QCheckBox{font-family:'Courier New';font-size:9pt;color:#8b949e;padding:3px;}"
    "QCheckBox::indicator{width:14px;height:14px;border-radius:3px;"
    "border:1px solid #30363d;background:#0d1117;}"
    "QCheckBox::indicator:checked{background:#1f6feb;border-color:#58a6ff;}"
)

def sbtn(text, bg, hv, mh=40):
    b = QPushButton(text)
    b.setFont(QFont("Courier New", 10, QFont.Bold))
    b.setMinimumHeight(mh); b.setCursor(Qt.PointingHandCursor)
    b.setStyleSheet(
        f"QPushButton{{background:{bg};color:#f0f6fc;border:1px solid {hv};"
        f"border-radius:7px;padding:5px 14px;}}"
        f"QPushButton:hover{{background:{hv};}}"
        f"QPushButton:disabled{{background:#161b22;color:#484f58;}}")
    return b

def inp_field(val="0", w=72):
    f = QLineEdit(str(val))
    f.setFont(QFont("Courier New", 11)); f.setFixedWidth(w)
    f.setAlignment(Qt.AlignCenter)
    f.setStyleSheet("background:#0d1117;color:#c9d1d9;"
                    "border:1px solid #30363d;border-radius:5px;padding:3px 5px;")
    return f

def pid_row_4(label, color, kp, ki, kd):
    row = QWidget()
    hl  = QHBoxLayout(row); hl.setContentsMargins(4,2,4,2); hl.setSpacing(6)
    lbl = QLabel(label)
    lbl.setFont(QFont("Courier New", 9, QFont.Bold))
    lbl.setStyleSheet(f"color:{color};"); lbl.setFixedWidth(55)
    hl.addWidget(lbl)
    inputs = {}
    for tag, val in [("Kp", kp), ("Ki", ki), ("Kd", kd)]:
        tl = QLabel(tag); tl.setFont(QFont("Courier New", 9))
        tl.setStyleSheet("color:#6b7fa3;")
        f = inp_field(str(val), 68)
        hl.addWidget(tl); hl.addWidget(f); inputs[tag] = f
    hl.addStretch()
    return row, inputs["Kp"], inputs["Ki"], inputs["Kd"]

# ═══════════════════════════════════════════════════════
#  VENTANA PRINCIPAL
# ═══════════════════════════════════════════════════════
class Dashboard(QWidget):
    def __init__(self, joy):
        super().__init__()
        self.joy = joy
        self.setWindowTitle(f"Robot 4M  —  RF inalámbrico  |  ESP32 {SERIAL_PORT}")
        self.setMinimumSize(1300, 900)
        self.setStyleSheet(STYLE)

        self._cycle      = 0
        self._mode_str   = "DETENIDO"
        self._mode_color = "#484f58"
        self._l_cmd      = 0.0
        self._r_cmd      = 0.0

        self._build_ui()

        self.t_ctrl = QTimer(); self.t_ctrl.timeout.connect(self._ctrl_tick); self.t_ctrl.start(20)
        self.t_disp = QTimer(); self.t_disp.timeout.connect(self._upd_disp); self.t_disp.start(80)
        self.t_conn = QTimer(); self.t_conn.timeout.connect(self._upd_conn); self.t_conn.start(500)

    def _ctrl_tick(self):
        pygame.event.pump()
        self._cycle += 1
        force = (self._cycle % RESEND_EVERY == 0)

        rt = read_trigger(self.joy, AX_RT)
        lt = read_trigger(self.joy, AX_LT)
        jx = self.joy.get_axis(AX_JOY_IZQ_X)
        dp = self.joy.get_hat(0)

        use_steering = True
        base_rpm     = 0.0

        if rt > 0.5 and lt > 0.5:
            base_rpm = rt if rt >= lt else -lt
            self._mode_str   = f"RT+LT  {base_rpm:+.0f} RPM"
            self._mode_color = "#3fb950" if base_rpm > 0 else "#f85149"
        elif rt > 0.5:
            base_rpm = rt
            self._mode_str   = f"RT  +{rt:.0f} RPM"
            self._mode_color = "#3fb950"
        elif lt > 0.5:
            base_rpm = -lt
            self._mode_str   = f"LT  -{lt:.0f} RPM"
            self._mode_color = "#f85149"
        elif dp[1] == 1:
            base_rpm = DPAD_RPM
            self._mode_str   = f"D-PAD ↑  +{DPAD_RPM:.0f} RPM"
            self._mode_color = "#58a6ff"
        elif dp[1] == -1:
            base_rpm = -DPAD_RPM
            self._mode_str   = f"D-PAD ↓  -{DPAD_RPM:.0f} RPM"
            self._mode_color = "#ffa657"
        elif dp[0] == -1:
            lo, ro = +DPAD_RPM, -DPAD_RPM
            set_orugas(lo, ro, force=force)
            self._l_cmd, self._r_cmd = lo, ro
            self._mode_str   = "D-PAD ←  giro izq"
            self._mode_color = "#d29922"
            use_steering     = False
        elif dp[0] == 1:
            lo, ro = -DPAD_RPM, +DPAD_RPM
            set_orugas(lo, ro, force=force)
            self._l_cmd, self._r_cmd = lo, ro
            self._mode_str   = "D-PAD →  giro der"
            self._mode_color = "#d29922"
            use_steering     = False
        else:
            if _last_L != 0.0 or _last_R != 0.0:
                set_orugas(0.0, 0.0, force=True)
                serial_send("stop")
            self._l_cmd, self._r_cmd = 0.0, 0.0
            self._mode_str   = "DETENIDO"
            self._mode_color = "#484f58"
            use_steering     = False

        if use_steering:
            lo, ro = apply_steering(base_rpm, jx)
            set_orugas(lo, ro, force=force)
            self._l_cmd, self._r_cmd = lo, ro

    def _build_ui(self):
        root = QVBoxLayout(self); root.setSpacing(8); root.setContentsMargins(14,10,14,10)

        # Header
        hdr = QHBoxLayout()
        title = QLabel("ROBOT 4 MOTORES  —  nRF24L01 INALÁMBRICO  |  ESP32 + XBOX 360")
        title.setFont(QFont("Courier New", 12, QFont.Bold))
        title.setStyleSheet("color:#58a6ff;letter-spacing:1px;")
        sub = QLabel(f"ESP32 → {SERIAL_PORT}  |  RF 2.4GHz 250kbps ACK  |  RT=avance  LT=retroceso  Joy IZQ=dir")
        sub.setFont(QFont("Courier New", 9)); sub.setStyleSheet("color:#484f58;")
        vt = QVBoxLayout(); vt.addWidget(title); vt.addWidget(sub)
        hdr.addLayout(vt); hdr.addStretch()
        self.conn_lbl = QLabel("CONECTANDO...")
        self.conn_lbl.setFont(QFont("Courier New", 9, QFont.Bold))
        self.conn_lbl.setStyleSheet("color:#d29922;padding:4px 10px;"
                                    "border:1px solid #30363d;border-radius:5px;background:#161b22;")
        hdr.addWidget(self.conn_lbl)
        self.rf_lbl = QLabel("RF ···")
        self.rf_lbl.setFont(QFont("Courier New", 9, QFont.Bold))
        self.rf_lbl.setStyleSheet("color:#484f58;padding:4px 10px;"
                                  "border:1px solid #30363d;border-radius:5px;background:#161b22;margin-left:6px;")
        hdr.addWidget(self.rf_lbl)
        root.addLayout(hdr)

        # Fila estados + modo
        dr = QHBoxLayout(); dr.setSpacing(8)
        self.lbl_l = QLabel("Oruga IZQ\n0 RPM")
        self.lbl_r = QLabel("Oruga DER\n0 RPM")
        for lbl, color in [(self.lbl_l,"#58a6ff"),(self.lbl_r,"#f78166")]:
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFont(QFont("Courier New", 11, QFont.Bold))
            lbl.setStyleSheet(f"color:{color};background:#0d1117;"
                              f"border:1px solid #30363d;border-radius:8px;padding:10px;")
            lbl.setMinimumHeight(64); dr.addWidget(lbl, 3)

        self.mode_lbl = QLabel("DETENIDO")
        self.mode_lbl.setAlignment(Qt.AlignCenter)
        self.mode_lbl.setFont(QFont("Courier New", 10, QFont.Bold))
        self.mode_lbl.setStyleSheet("color:#484f58;background:#0d1117;"
                                    "border:1px solid #30363d;border-radius:8px;"
                                    "padding:10px;min-width:140px;")
        self.mode_lbl.setMinimumHeight(64); dr.addWidget(self.mode_lbl, 2)

        # Estadísticas RF
        self.rf_stats = QLabel("ACK OK: 0\nFAIL: 0")
        self.rf_stats.setAlignment(Qt.AlignCenter)
        self.rf_stats.setFont(QFont("Courier New", 9))
        self.rf_stats.setStyleSheet("color:#8b949e;background:#0d1117;"
                                    "border:1px solid #30363d;border-radius:8px;padding:10px;")
        self.rf_stats.setMinimumHeight(64); dr.addWidget(self.rf_stats, 2)
        root.addLayout(dr)

        # Controles Xbox + STOP
        ctrl_box = QGroupBox("CONTROL  —  Xbox 360  →  ESP32  →  nRF24  →  Mega")
        vl = QVBoxLayout(ctrl_box); vl.setSpacing(6)
        ctrl_map = [
            ("RT","Avance proporcional  0–150 RPM","#3fb950"),
            ("LT","Retroceso proporcional  0–150 RPM","#f85149"),
            ("D-Pad ↑↓",f"Avance/retroceso fijo  ±{DPAD_RPM:.0f} RPM","#58a6ff"),
            ("D-Pad ←→","Giro en sitio","#d29922"),
            ("Joy IZQ X","Dirección (mezcla orugas)","#c9d1d9"),
        ]
        for ctrl, desc, color in ctrl_map:
            row = QHBoxLayout()
            cl = QLabel(ctrl); cl.setFont(QFont("Courier New",9,QFont.Bold))
            cl.setStyleSheet(f"color:{color};"); cl.setFixedWidth(90)
            dl = QLabel(desc); dl.setFont(QFont("Courier New",9))
            dl.setStyleSheet("color:#6b7fa3;")
            row.addWidget(cl); row.addWidget(dl); row.addStretch()
            vl.addLayout(row)

        rr = QHBoxLayout()
        rl = QLabel("RPM máx:"); rl.setFont(QFont("Courier New",10))
        rl.setStyleSheet("color:#8b949e;")
        self.rpm_inp = inp_field(str(int(MAX_RPM)), 70)
        self.rpm_inp.editingFinished.connect(self._upd_max_rpm)
        rr.addWidget(rl); rr.addWidget(self.rpm_inp)
        rr.addWidget(QLabel("(10–150)")); rr.addStretch()
        vl.addLayout(rr)

        stop_btn = sbtn("STOP EMERGENCIA", "#3d0c0c", "#f85149", 40)
        stop_btn.clicked.connect(lambda: (set_orugas(0,0,force=True), serial_send("stop")))
        vl.addWidget(stop_btn)
        root.addWidget(ctrl_box)

    def _upd_max_rpm(self):
        global MAX_RPM
        try:
            v = max(10., min(150., abs(float(self.rpm_inp.text()))))
            MAX_RPM = v; self.rpm_inp.setText(str(int(v)))
        except ValueError: pass

    def _upd_conn(self):
        if _serial_ok:
            self.conn_lbl.setText(f"ESP32 OK  {SERIAL_PORT}")
            self.conn_lbl.setStyleSheet("color:#3fb950;padding:4px 10px;"
                                        "border:1px solid #30363d;border-radius:5px;background:#161b22;")
        else:
            self.conn_lbl.setText("SIN SERIAL — verifica el puerto")
            self.conn_lbl.setStyleSheet("color:#f85149;padding:4px 10px;"
                                        "border:1px solid #30363d;border-radius:5px;background:#161b22;")
        if _rf_link_ok and _rf_ok:
            self.rf_lbl.setText("RF OK")
            self.rf_lbl.setStyleSheet("color:#3fb950;padding:4px 10px;"
                                      "border:1px solid #30363d;border-radius:5px;background:#161b22;margin-left:6px;")
        elif not _rf_ok:
            self.rf_lbl.setText("RF sin init")
            self.rf_lbl.setStyleSheet("color:#d29922;padding:4px 10px;"
                                      "border:1px solid #30363d;border-radius:5px;background:#161b22;margin-left:6px;")
        else:
            self.rf_lbl.setText("RF FAIL")
            self.rf_lbl.setStyleSheet("color:#f85149;padding:4px 10px;"
                                      "border:1px solid #30363d;border-radius:5px;background:#161b22;margin-left:6px;")
        self.rf_stats.setText(f"ACK OK: {_ack_ok}\nFAIL: {_ack_fail}")

    def _upd_disp(self):
        lc = "#58a6ff" if self._l_cmd >= 0 else "#ff7b72"
        rc = "#f78166" if self._r_cmd >= 0 else "#ffa657"
        self.lbl_l.setText(f"Oruga IZQ\n{self._l_cmd:+.0f} RPM")
        self.lbl_l.setStyleSheet(f"color:{lc};background:#0d1117;"
                                 f"border:1px solid #30363d;border-radius:8px;padding:10px;")
        self.lbl_r.setText(f"Oruga DER\n{self._r_cmd:+.0f} RPM")
        self.lbl_r.setStyleSheet(f"color:{rc};background:#0d1117;"
                                 f"border:1px solid #30363d;border-radius:8px;padding:10px;")
        self.mode_lbl.setText(f"{self._mode_str}\nL={self._l_cmd:+.0f}  R={self._r_cmd:+.0f}")
        self.mode_lbl.setStyleSheet(
            f"color:{self._mode_color};background:#0d1117;"
            f"border:1px solid #30363d;border-radius:8px;"
            f"padding:10px;min-width:140px;font-family:'Courier New';"
            f"font-size:10pt;font-weight:bold;")

    def closeEvent(self, e):
        set_orugas(0, 0, force=True)
        serial_send("stop")
        pygame.quit()
        super().closeEvent(e)

# ═══════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════
def main():
    connect_serial(SERIAL_PORT, SERIAL_BAUD)
    threading.Thread(target=reader_thread, daemon=True).start()

    pygame.init(); pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("ERROR: No se detectó control Xbox. Conéctalo y reintenta.")
        sys.exit(1)
    joy = pygame.joystick.Joystick(0); joy.init()
    print(f"✓ Control: {joy.get_name()}")

    app = QApplication(sys.argv); app.setStyle("Fusion")
    w = Dashboard(joy); w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()