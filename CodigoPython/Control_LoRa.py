"""
Robot PID — 4 Orugas — Control Xbox 360
PC → Serial USB → ESP32 antena (LoRa TX)
                      ~~LoRa~~
               ESP32 receptora (LoRa RX)
                      │ UART
               Arduino Mega → 4 motores + PID x4
================================================================================
Requiere:
    pip install pyserial pygame PyQt5 matplotlib
"""

import sys, serial, threading, time, queue
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
ESP32_PORT  = "COM3"      # Puerto COM de la ESP32 ANTENA
SERIAL_BAUD = 115200

MAX_RPM   = 120.0
DPAD_RPM  = 80.0
DEAD_JOY  = 0.08
DEAD_TRIG = 0.03

AX_JOY_IZQ_X = 0
AX_LT        = 4
AX_RT        = 5

RESEND_EVERY = 8   # cada ~160ms — mantiene watchdog Mega (5s)
# Cruceta: comandos fijos (forward/back/left/right). En firmware antena/Mega usan ±80 RPM;
# mantén DPAD_RPM coherente con eso si cambias uno u otro.
# Si físicamente izquierda/derecha van al revés respecto a la cruceta, cambia este flag.
DPAD_SWAP_LR = True

# ═══════════════════════════════════════════════════════
#  CONEXIÓN SERIAL
# ═══════════════════════════════════════════════════════
_ser       = None
_serial_ok = False
_lora_ok   = False
# Solo TX: la lectura NO debe compartir candado con write/readline (congela la UI).
_serial_tx_lock = threading.Lock()
_TX_MAX = 64
_tx_queue = queue.Queue(maxsize=_TX_MAX)
_urgent_tx = queue.Queue(maxsize=32)
last_esp_echo = ""

def drain_tx_queue():
    """Descarta comandos pendientes (evita que un stop llegue después de lr= viejos)."""
    while True:
        try:
            _tx_queue.get_nowait()
        except queue.Empty:
            break

def serial_send_immediate(cmd: str):
    """
    Encola envío prioritario (no bloquea el hilo de la UI).
    El writer vacía la cola normal antes de escribir cada comando urgente.
    """
    if not _serial_ok or _ser is None:
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

def connect_serial(port, baud, retries=5):
    global _ser, _serial_ok
    for i in range(retries):
        try:
            # timeout corto: el hilo lector no bloquea el envío más de ~100 ms
            _ser = serial.Serial(
                port, baud,
                timeout=0.08,
                write_timeout=2,
                dsrdtr=False,
                rtscts=False,
            )
            time.sleep(2)
            _serial_ok = True
            print(f"✓ Serial conectado  {port}")
            return
        except serial.SerialException as e:
            print(f"  Intento {i+1}/{retries}: {e}")
            time.sleep(2)
    print("  No se pudo conectar — modo simulación")

def serial_send(cmd: str):
    if _serial_ok and _ser:
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

def serial_send_many(cmds):
    for c in cmds:
        serial_send(c)

def writer_thread():
    global _serial_ok, last_esp_echo
    while True:
        if not _serial_ok or _ser is None:
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
                _ser.write((cmd + "\n").encode("utf-8", errors="replace"))
                _ser.flush()
            if urgent:
                last_esp_echo = f"→ {cmd}"[:160]
        except Exception as e:
            print(f"  Serial TX error: {e}")
            last_esp_echo = f"ERR serial TX: {e}"
            _serial_ok = False

# ═══════════════════════════════════════════════════════
#  HILO LECTOR — telemetría 4 motores
#  Formato del Mega: TL:XX\tTR:XX\tRPM1:XX\tRPM2:XX\t...
# ═══════════════════════════════════════════════════════
tl_live  = 0.0; tr_live  = 0.0
rpm1_live= 0.0; rpm2_live= 0.0
rpm3_live= 0.0; rpm4_live= 0.0
err1_live= 0.0; err2_live= 0.0
err3_live= 0.0; err4_live= 0.0
lora_timeout = False
lora_fails   = 0

_hist = 300
d_rpm1 = deque(maxlen=_hist); d_rpm2 = deque(maxlen=_hist)
d_rpm3 = deque(maxlen=_hist); d_rpm4 = deque(maxlen=_hist)
d_tl   = deque(maxlen=_hist); d_tr   = deque(maxlen=_hist)

def reader_thread():
    global tl_live, tr_live
    global rpm1_live, rpm2_live, rpm3_live, rpm4_live
    global err1_live, err2_live, err3_live, err4_live
    global _lora_ok, lora_timeout, lora_fails, last_esp_echo
    while True:
        if not _serial_ok or _ser is None:
            time.sleep(0.2); continue
        try:
            raw = _ser.readline().decode(errors="ignore").strip()
            if not raw: continue

            if "LORA_READY" in raw or "ANTENA" in raw:
                _lora_ok = True; print(f"✓ {raw}")
            elif "ERR:LORA_TIMEOUT" in raw:
                lora_timeout = True; lora_fails += 1
                last_esp_echo = raw[:160]
            elif raw.startswith("OK:"):
                lora_timeout = False
                last_esp_echo = raw[:160]

            # Telemetría del Mega: TL:XX\tTR:XX\tRPM1:XX\t...
            elif "RPM1:" in raw or "TL:" in raw:
                lora_timeout = False
                try:
                    p = {}
                    for tok in raw.split("\t"):
                        if ":" in tok:
                            k, v = tok.split(":", 1)
                            p[k.strip()] = float(v.strip())
                    tl_live   = p.get("TL",   tl_live)
                    tr_live   = p.get("TR",   tr_live)
                    rpm1_live = p.get("RPM1", rpm1_live)
                    rpm2_live = p.get("RPM2", rpm2_live)
                    rpm3_live = p.get("RPM3", rpm3_live)
                    rpm4_live = p.get("RPM4", rpm4_live)
                    err1_live = p.get("Err1", err1_live)
                    err2_live = p.get("Err2", err2_live)
                    err3_live = p.get("Err3", err3_live)
                    err4_live = p.get("Err4", err4_live)
                    d_rpm1.append(rpm1_live); d_rpm2.append(rpm2_live)
                    d_rpm3.append(rpm3_live); d_rpm4.append(rpm4_live)
                    d_tl.append(tl_live);     d_tr.append(tr_live)
                except Exception: pass
            elif raw.startswith("FWD:"):
                last_esp_echo = raw[:160]
        except Exception: pass

# ═══════════════════════════════════════════════════════
#  LÓGICA XBOX
#  l = oruga izquierda (M1+M3)
#  r = oruga derecha   (M2+M4)
# ═══════════════════════════════════════════════════════
_last_l = None
_last_r = None
_last_dpad_cmd = None  # "forward"|"back"|"left"|"right" mientras cruceta activa

def dpad_clear_if_active():
    """Si la cruceta enviaba comandos fijos, envía stop y limpia estado."""
    global _last_dpad_cmd, _last_l, _last_r
    if _last_dpad_cmd is None:
        return
    serial_send_immediate("stop")
    _last_dpad_cmd = None
    _last_l = 0.0
    _last_r = 0.0

def dpad_send(name: str, force: bool = False):
    """Envía forward/back/left/right; reenvía en force (watchdog)."""
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

def set_lr_symmetric(v: float, force: bool = False):
    """RT/LT proporcional: un solo lr= (positivo o negativo) cuando ambas orugas van igual."""
    global _last_l, _last_r
    v = round(max(-MAX_RPM, min(MAX_RPM, v)), 1)
    changed = (
        force
        or _last_l is None
        or abs(v - (_last_l or 0)) >= 1.0
        or abs(v - (_last_r or 0)) >= 1.0
    )
    if changed:
        serial_send(f"lr={v}")
        _last_l = _last_r = v

def set_tracks(l: float, r: float, force: bool = False):
    global _last_l, _last_r
    l = round(max(-MAX_RPM, min(MAX_RPM, l)), 1)
    r = round(max(-MAX_RPM, min(MAX_RPM, r)), 1)

    changed_l = force or _last_l is None or abs(l - (_last_l or 0)) >= 1.0
    changed_r = force or _last_r is None or abs(r - (_last_r or 0)) >= 1.0

    if changed_l or changed_r:
        if l == r:
            serial_send(f"lr={l}")
        else:
            if changed_l: serial_send(f"l={l}")
            if changed_r: serial_send(f"r={r}")
        _last_l = l
        _last_r = r

def set_steering_tracks(lo: float, ro: float, force: bool = False):
    """RT/LT + joystick: lr= si va recto; l=/r= si hay giro diferencial."""
    lo = round(max(-MAX_RPM, min(MAX_RPM, lo)), 1)
    ro = round(max(-MAX_RPM, min(MAX_RPM, ro)), 1)
    if abs(lo - ro) < 0.5:
        set_lr_symmetric(lo, force=force)
    else:
        set_tracks(lo, ro, force=force)

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
def pid_row(label, color, kp, ki, kd):
    row = QWidget()
    hl  = QHBoxLayout(row); hl.setContentsMargins(4,2,4,2); hl.setSpacing(8)
    lbl = QLabel(label)
    lbl.setFont(QFont("Courier New",10,QFont.Bold))
    lbl.setStyleSheet(f"color:{color};"); lbl.setFixedWidth(50); hl.addWidget(lbl)
    inputs = {}
    for tag, val in [("Kp",kp),("Ki",ki),("Kd",kd)]:
        tl  = QLabel(tag); tl.setFont(QFont("Courier New",9))
        tl.setStyleSheet("color:#6b7fa3;")
        inp = QLineEdit(str(val))
        inp.setFont(QFont("Courier New",10)); inp.setFixedWidth(70)
        inp.setAlignment(Qt.AlignCenter)
        inp.setStyleSheet("background:#0d1117;color:#c9d1d9;"
                          "border:1px solid #30363d;border-radius:5px;padding:3px 5px;")
        hl.addWidget(tl); hl.addWidget(inp); inputs[tag] = inp
    hl.addStretch()
    return row, inputs["Kp"], inputs["Ki"], inputs["Kd"]

def sbtn(text, bg, hv, mh=40):
    b = QPushButton(text)
    b.setFont(QFont("Courier New",10,QFont.Bold))
    b.setMinimumHeight(mh); b.setCursor(Qt.PointingHandCursor)
    b.setStyleSheet(
        f"QPushButton{{background:{bg};color:#f0f6fc;border:1px solid {hv};"
        f"border-radius:7px;padding:5px 14px;}}"
        f"QPushButton:hover{{background:{hv};}}"
        f"QPushButton:disabled{{background:#161b22;color:#484f58;}}")
    return b

STYLE = (
    "QWidget{background:#0d1117;color:#c9d1d9;}"
    "QGroupBox{background:#161b22;border:1px solid #30363d;border-radius:8px;"
    "margin-top:10px;padding:10px 8px 8px 8px;color:#8b949e;"
    "font-family:'Courier New';font-size:9pt;}"
    "QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;}"
    "QTabWidget::pane{border:1px solid #30363d;background:#161b22;"
    "border-radius:0 6px 6px 6px;}"
    "QTabBar::tab{background:#0d1117;color:#8b949e;padding:6px 12px;"
    "font-family:'Courier New';font-size:9pt;border:1px solid #30363d;"
    "border-bottom:none;border-radius:6px 6px 0 0;margin-right:2px;}"
    "QTabBar::tab:selected{background:#161b22;color:#f0f6fc;border-color:#58a6ff;}"
    "QCheckBox{font-family:'Courier New';font-size:9pt;color:#8b949e;padding:3px;}"
    "QCheckBox::indicator{width:14px;height:14px;border-radius:3px;"
    "border:1px solid #30363d;background:#0d1117;}"
    "QCheckBox::indicator:checked{background:#1f6feb;border-color:#58a6ff;}"
)

# ═══════════════════════════════════════════════════════
#  DASHBOARD
# ═══════════════════════════════════════════════════════
class Dashboard(QWidget):
    def __init__(self, joy):
        super().__init__()
        self.joy = joy
        self.setWindowTitle(
            f"Robot 4 Orugas  —  Xbox 360  |  LoRa  |  {ESP32_PORT}")
        self.setMinimumSize(1280, 920)
        self.setStyleSheet(STYLE)

        self._cycle      = 0
        self._mode_str   = "DETENIDO"
        self._mode_color = "#484f58"
        self._l_cmd      = 0.0
        self._r_cmd      = 0.0

        self._build_ui()

        self.t_ctrl = QTimer(); self.t_ctrl.timeout.connect(self._ctrl_tick); self.t_ctrl.start(20)
        self.t_disp = QTimer(); self.t_disp.timeout.connect(self._upd_disp); self.t_disp.start(60)
        self.t_plot = QTimer(); self.t_plot.timeout.connect(self._upd_plot); self.t_plot.start(250)
        self.t_conn = QTimer(); self.t_conn.timeout.connect(self._upd_conn); self.t_conn.start(400)

    # ── Control Xbox ──────────────────────────────────
    def _ctrl_tick(self):
        global _last_l, _last_r
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
            dpad_clear_if_active()
            base_rpm = rt if rt >= lt else -lt
            self._mode_str   = f"RT+LT  {base_rpm:+.0f} RPM"
            self._mode_color = "#3fb950" if base_rpm > 0 else "#f85149"
        elif rt > 0.5:
            dpad_clear_if_active()
            base_rpm = rt
            self._mode_str   = f"RT  +{rt:.0f} RPM"
            self._mode_color = "#3fb950"
        elif lt > 0.5:
            dpad_clear_if_active()
            base_rpm = -lt
            self._mode_str   = f"LT  -{lt:.0f} RPM"
            self._mode_color = "#f85149"
        elif dp[1] == 1:
            dpad_send("forward", force=force)
            self._l_cmd, self._r_cmd = float(DPAD_RPM), float(DPAD_RPM)
            self._mode_str   = f"D-PAD ↑  forward  (+{DPAD_RPM:.0f})"
            self._mode_color = "#58a6ff"
            use_steering     = False
        elif dp[1] == -1:
            dpad_send("back", force=force)
            self._l_cmd, self._r_cmd = -float(DPAD_RPM), -float(DPAD_RPM)
            self._mode_str   = f"D-PAD ↓  back  (-{DPAD_RPM:.0f})"
            self._mode_color = "#ffa657"
            use_steering     = False
        elif dp[0] == -1:
            if DPAD_SWAP_LR:
                dpad_send("right", force=force)
                self._l_cmd, self._r_cmd = float(DPAD_RPM), -float(DPAD_RPM)
            else:
                dpad_send("left", force=force)
                self._l_cmd, self._r_cmd = -float(DPAD_RPM), float(DPAD_RPM)
            self._mode_str   = "D-PAD ←  giro izq."
            self._mode_color = "#d29922"
            use_steering     = False
        elif dp[0] == 1:
            if DPAD_SWAP_LR:
                dpad_send("left", force=force)
                self._l_cmd, self._r_cmd = -float(DPAD_RPM), float(DPAD_RPM)
            else:
                dpad_send("right", force=force)
                self._l_cmd, self._r_cmd = float(DPAD_RPM), -float(DPAD_RPM)
            self._mode_str   = "D-PAD →  giro der."
            self._mode_color = "#d29922"
            use_steering     = False
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
            self._mode_str   = "DETENIDO"
            self._mode_color = "#484f58"
            use_steering     = False

        if use_steering:
            lo, ro = apply_steering(base_rpm, jx)
            set_steering_tracks(lo, ro, force=force)
            self._l_cmd, self._r_cmd = lo, ro

    # ── UI ───────────────────────────────────────────
    def _build_ui(self):
        root = QVBoxLayout(self); root.setSpacing(8); root.setContentsMargins(14,10,14,10)

        # Header
        hdr = QHBoxLayout()
        title = QLabel("ROBOT 4 ORUGAS  —  Xbox 360  +  LoRa  →  Arduino Mega")
        title.setFont(QFont("Courier New",13,QFont.Bold))
        title.setStyleSheet("color:#58a6ff;letter-spacing:1px;")
        sub = QLabel(
            f"Antena: {ESP32_PORT}  |  RT/LT → lr=±RPM  |  D-Pad → forward/back/left/right  |  Joy=girar")
        sub.setFont(QFont("Courier New",9)); sub.setStyleSheet("color:#484f58;")
        vt = QVBoxLayout(); vt.addWidget(title); vt.addWidget(sub)
        hdr.addLayout(vt); hdr.addStretch()

        conn_box = QVBoxLayout()
        self.conn_lbl = QLabel("CONECTANDO...")
        self.conn_lbl.setFont(QFont("Courier New",9,QFont.Bold))
        self.conn_lbl.setStyleSheet("color:#d29922;padding:3px 8px;"
                                    "border:1px solid #30363d;border-radius:5px;background:#161b22;")
        self.lora_lbl = QLabel("LoRa: —")
        self.lora_lbl.setFont(QFont("Courier New",9,QFont.Bold))
        self.lora_lbl.setStyleSheet("color:#484f58;padding:3px 8px;"
                                    "border:1px solid #30363d;border-radius:5px;background:#161b22;")
        self.echo_lbl = QLabel("ESP32: —")
        self.echo_lbl.setFont(QFont("Courier New",8))
        self.echo_lbl.setStyleSheet("color:#6b7fa3;padding:2px 6px;"
                                    "border:1px solid #21262d;border-radius:4px;background:#0d1117;")
        self.echo_lbl.setWordWrap(True)
        self.echo_lbl.setMaximumWidth(420)
        conn_box.addWidget(self.conn_lbl); conn_box.addWidget(self.lora_lbl)
        conn_box.addWidget(self.echo_lbl)
        hdr.addLayout(conn_box)
        root.addLayout(hdr)

        # Displays: 4 RPM individuales + modo
        dr = QHBoxLayout(); dr.setSpacing(6)
        self.d1 = self._mkd("M1 IZQ-F", "#58a6ff")
        self.d2 = self._mkd("M2 DER-F", "#f78166")
        self.d3 = self._mkd("M3 IZQ-T", "#85b7eb")
        self.d4 = self._mkd("M4 DER-T", "#ffa657")
        self.mode_lbl = QLabel("DETENIDO")
        self.mode_lbl.setAlignment(Qt.AlignCenter)
        self.mode_lbl.setFont(QFont("Courier New",10,QFont.Bold))
        self.mode_lbl.setStyleSheet("color:#484f58;background:#0d1117;"
                                    "border:1px solid #30363d;border-radius:10px;"
                                    "padding:8px;min-width:130px;")
        self.mode_lbl.setMinimumHeight(68)
        for d in [self.d1,self.d2,self.d3,self.d4]:
            dr.addWidget(d, 2)
        dr.addWidget(self.mode_lbl, 2)
        root.addLayout(dr)

        # Centro
        ctr = QHBoxLayout(); ctr.setSpacing(10)
        lft = QVBoxLayout(); lft.setSpacing(8)
        lft.addWidget(self._ctrl_panel())
        lft.addWidget(self._pid_tabs())
        lft.addStretch()
        self._build_plot()
        ctr.addLayout(lft,38); ctr.addWidget(self.canvas,62)
        root.addLayout(ctr)

    def _ctrl_panel(self):
        box = QGroupBox("CONTROL  —  Xbox 360  +  Comandos")
        vl  = QVBoxLayout(box); vl.setSpacing(7)

        ctrl_map = [
            ("RT",        "Avance proporcional  0–120 RPM", "#3fb950"),
            ("LT",        "Retroceso proporcional  0–120 RPM", "#f85149"),
            ("D-Pad ↑↓",  f"forward / back  (fijo ±{DPAD_RPM:.0f} RPM en firmware)", "#58a6ff"),
            ("D-Pad ←→",  "left / right  (giro en sitio, comando fijo)", "#d29922"),
            ("Joy IZQ X", "Dirección mezclada con RT/LT", "#c9d1d9"),
        ]
        for ctrl, desc, color in ctrl_map:
            row = QHBoxLayout()
            cl = QLabel(ctrl); cl.setFont(QFont("Courier New",9,QFont.Bold))
            cl.setStyleSheet(f"color:{color};"); cl.setFixedWidth(90)
            dl = QLabel(desc); dl.setFont(QFont("Courier New",9))
            dl.setStyleSheet("color:#6b7fa3;")
            row.addWidget(cl); row.addWidget(dl); row.addStretch()
            vl.addLayout(row)

        # Comando manual
        cr = QHBoxLayout()
        cl2 = QLabel("Cmd:"); cl2.setFont(QFont("Courier New",9))
        cl2.setStyleSheet("color:#8b949e;")
        self.cmd_inp = QLineEdit()
        self.cmd_inp.setPlaceholderText("kpl=1.4  kpr=1.4  diag  pid ...")
        self.cmd_inp.setFont(QFont("Courier New",10)); self.cmd_inp.setMinimumWidth(200)
        self.cmd_inp.setStyleSheet("background:#0d1117;color:#c9d1d9;"
                                   "border:1px solid #30363d;border-radius:5px;padding:4px;")
        self.cmd_inp.returnPressed.connect(self._send_manual)
        send_btn = sbtn("Enviar", "#161b22", "#30363d", 30)
        send_btn.setFixedWidth(65)
        send_btn.clicked.connect(self._send_manual)
        cr.addWidget(cl2); cr.addWidget(self.cmd_inp); cr.addWidget(send_btn)
        vl.addLayout(cr)

        # RPM máx + fix sign
        rr = QHBoxLayout()
        rl = QLabel("RPM máx:"); rl.setFont(QFont("Courier New",9))
        rl.setStyleSheet("color:#8b949e;")
        self.rpm_inp = QLineEdit(str(int(MAX_RPM)))
        self.rpm_inp.setFont(QFont("Courier New",10)); self.rpm_inp.setFixedWidth(60)
        self.rpm_inp.setAlignment(Qt.AlignCenter)
        self.rpm_inp.setStyleSheet("background:#0d1117;color:#c9d1d9;"
                                   "border:1px solid #30363d;border-radius:5px;padding:3px;")
        self.rpm_inp.editingFinished.connect(self._upd_max_rpm)
        self.fix_cb = QCheckBox("Fix sign")
        self.fix_cb.setChecked(True)
        self.fix_cb.stateChanged.connect(
            lambda s: serial_send("fixsign=1" if s else "fixsign=0"))
        rr.addWidget(rl); rr.addWidget(self.rpm_inp); rr.addSpacing(12)
        rr.addWidget(self.fix_cb); rr.addStretch()
        vl.addLayout(rr)

        stop_btn = sbtn("STOP EMERGENCIA", "#3d0c0c", "#f85149", 40)
        stop_btn.clicked.connect(self._emergency_stop)
        vl.addWidget(stop_btn)
        return box

    def _pid_tabs(self):
        tabs = QTabWidget()

        # Tab oruga IZQ (M1+M3)
        tl = QWidget(); tl.setStyleSheet("background:#161b22;")
        vl = QVBoxLayout(tl); vl.setSpacing(5)
        vl.addWidget(self._note("Oruga IZQ — M1 (frontal) + M3 (trasero)", "#58a6ff"))
        r1f,self.kp1f,self.ki1f,self.kd1f = pid_row("M1 fwd","#58a6ff",15.35,1.500,0.038)
        r3f,self.kp3f,self.ki3f,self.kd3f = pid_row("M3 fwd","#85b7eb",15.35,1.500,0.038)
        vl.addWidget(r1f); vl.addWidget(r3f)
        bl = sbtn("Aplicar IZQ avance","#0e2a47","#1f6feb",30)
        bl.clicked.connect(self._apply_izq_fwd); vl.addWidget(bl); vl.addStretch()

        # Tab oruga DER (M2+M4)
        tr = QWidget(); tr.setStyleSheet("background:#161b22;")
        vr = QVBoxLayout(tr); vr.setSpacing(5)
        vr.addWidget(self._note("Oruga DER — M2 (frontal) + M4 (trasero)", "#f78166"))
        r2f,self.kp2f,self.ki2f,self.kd2f = pid_row("M2 fwd","#f78166",15.35,1.545,0.028)
        r4f,self.kp4f,self.ki4f,self.kd4f = pid_row("M4 fwd","#ffa657",15.35,1.545,0.028)
        vr.addWidget(r2f); vr.addWidget(r4f)
        br = sbtn("Aplicar DER avance","#0e2a47","#1f6feb",30)
        br.clicked.connect(self._apply_der_fwd); vr.addWidget(br); vr.addStretch()

        # Tab ajuste rápido por oruga
        tq = QWidget(); tq.setStyleSheet("background:#161b22;")
        vq = QVBoxLayout(tq); vq.setSpacing(5)
        vq.addWidget(self._note("Ajuste simétrico — aplica a M1+M3 o M2+M4", "#3fb950"))
        rl_row,self.kpl,self.kil,self.kdl = pid_row("IZQ","#58a6ff",15.35,1.500,0.038)
        rr_row,self.kpr,self.kir,self.kdr = pid_row("DER","#f78166",15.35,1.545,0.028)
        vq.addWidget(rl_row); vq.addWidget(rr_row)
        bq = sbtn("Aplicar ambas orugas","#0e2a47","#1f6feb",30)
        bq.clicked.connect(self._apply_both); vq.addWidget(bq); vq.addStretch()

        tabs.addTab(tl,"ORUGA IZQ"); tabs.addTab(tr,"ORUGA DER"); tabs.addTab(tq,"AJUSTE RÁPIDO")
        return tabs

    def _note(self, text, color):
        l = QLabel(text); l.setFont(QFont("Courier New",8))
        l.setStyleSheet(f"color:{color};padding:2px 4px;")
        return l

    def _build_plot(self):
        self.figure = Figure(facecolor="#0d1117")
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setStyleSheet("background:#0d1117;border-radius:8px;")
        self.canvas.setFocusPolicy(Qt.NoFocus)
        gs = gridspec.GridSpec(2,1,figure=self.figure,
                               hspace=0.45,top=0.93,bottom=0.08,left=0.09,right=0.97)
        self.ax_rpm = self.figure.add_subplot(gs[0])
        self.ax_dif = self.figure.add_subplot(gs[1])
        for ax in (self.ax_rpm, self.ax_dif):
            ax.set_facecolor("#161b22"); ax.tick_params(colors="#484f58",labelsize=8)
            ax.grid(True,alpha=0.15,color="#30363d")
            ax.axhline(0,color="#30363d",linewidth=0.8)
            for sp in ax.spines.values(): sp.set_edgecolor("#30363d")
        self.ax_rpm.set_title("RPM vs Target — 4 motores",color="#8b949e",fontsize=9,pad=4)
        self.ax_dif.set_title("Diferencia IZQ − DER  (M1−M2 y M3−M4)",color="#8b949e",fontsize=9,pad=4)

    # ── Acciones ──────────────────────────────────────
    def _emergency_stop(self):
        global _last_dpad_cmd, _last_l, _last_r
        _last_dpad_cmd = None
        _last_l = 0.0
        _last_r = 0.0
        serial_send_immediate("stop")

    def _send_manual(self):
        cmd = self.cmd_inp.text().strip()
        if cmd: serial_send_immediate(cmd); self.cmd_inp.clear()

    def _upd_max_rpm(self):
        global MAX_RPM
        try:
            v = max(10., min(150., abs(float(self.rpm_inp.text()))))
            MAX_RPM = v; self.rpm_inp.setText(str(int(v)))
        except ValueError: pass

    def _apply_izq_fwd(self):
        try:
            serial_send_many([
                f"kp1f={float(self.kp1f.text())}", f"ki1f={float(self.ki1f.text())}",
                f"kd1f={float(self.kd1f.text())}", f"kp3f={float(self.kp3f.text())}",
                f"ki3f={float(self.ki3f.text())}", f"kd3f={float(self.kd3f.text())}"
            ])
        except ValueError: pass

    def _apply_der_fwd(self):
        try:
            serial_send_many([
                f"kp2f={float(self.kp2f.text())}", f"ki2f={float(self.ki2f.text())}",
                f"kd2f={float(self.kd2f.text())}", f"kp4f={float(self.kp4f.text())}",
                f"ki4f={float(self.ki4f.text())}", f"kd4f={float(self.kd4f.text())}"
            ])
        except ValueError: pass

    def _apply_both(self):
        try:
            serial_send_many([
                f"kpl={float(self.kpl.text())}",
                f"kil={float(self.kil.text())}",
                f"kdl={float(self.kdl.text())}",
                f"kpr={float(self.kpr.text())}",
                f"kir={float(self.kir.text())}",
                f"kdr={float(self.kdr.text())}",
            ])
        except ValueError: pass

    # ── Refresh UI ────────────────────────────────────
    def _upd_conn(self):
        if last_esp_echo:
            self.echo_lbl.setText(f"ESP32: {last_esp_echo}")
        if _serial_ok and _lora_ok:
            self.conn_lbl.setText(f"Antena OK  {ESP32_PORT}")
            self.conn_lbl.setStyleSheet("color:#3fb950;padding:3px 8px;"
                                        "border:1px solid #30363d;border-radius:5px;background:#161b22;")
        elif _serial_ok:
            self.conn_lbl.setText("Serial OK — esperando LoRa...")
            self.conn_lbl.setStyleSheet("color:#d29922;padding:3px 8px;"
                                        "border:1px solid #30363d;border-radius:5px;background:#161b22;")
        else:
            self.conn_lbl.setText("SIN SERIAL — verifica el puerto")
            self.conn_lbl.setStyleSheet("color:#f85149;padding:3px 8px;"
                                        "border:1px solid #30363d;border-radius:5px;background:#161b22;")

        if lora_timeout:
            self.lora_lbl.setText(f"LoRa: TIMEOUT #{lora_fails}")
            self.lora_lbl.setStyleSheet("color:#f85149;padding:3px 8px;"
                                        "border:1px solid #30363d;border-radius:5px;background:#161b22;")
        elif _lora_ok:
            self.lora_lbl.setText("LoRa: OK")
            self.lora_lbl.setStyleSheet("color:#3fb950;padding:3px 8px;"
                                        "border:1px solid #30363d;border-radius:5px;background:#161b22;")

    def _upd_disp(self):
        def disp(lbl, val, name):
            d = "FWD" if val >= 0 else "REV"
            c = "#58a6ff" if "IZQ" in name else "#f78166"
            if val < 0: c = "#ff7b72" if "IZQ" in name else "#ffa657"
            lbl.setText(f"{name}\n[{d}] {val:+.1f}")
            lbl.setStyleSheet(f"color:{c};background:#0d1117;"
                              f"border:1px solid #30363d;border-radius:10px;padding:8px;")

        disp(self.d1, rpm1_live, "M1 IZQ-F")
        disp(self.d2, rpm2_live, "M2 DER-F")
        disp(self.d3, rpm3_live, "M3 IZQ-T")
        disp(self.d4, rpm4_live, "M4 DER-T")

        self.mode_lbl.setText(
            f"{self._mode_str}\nL={self._l_cmd:+.0f}  R={self._r_cmd:+.0f}")
        self.mode_lbl.setStyleSheet(
            f"color:{self._mode_color};background:#0d1117;"
            f"border:1px solid #30363d;border-radius:10px;"
            f"padding:8px;font-family:'Courier New';font-size:9pt;font-weight:bold;")

    def _upd_plot(self):
        if not d_rpm1: return
        r1=list(d_rpm1); r2=list(d_rpm2)
        r3=list(d_rpm3); r4=list(d_rpm4)
        tl=list(d_tl);   tr=list(d_tr)
        xs=range(len(r1))

        self.ax_rpm.clear()
        self.ax_rpm.set_facecolor("#161b22")
        self.ax_rpm.grid(True,alpha=0.15,color="#30363d")
        self.ax_rpm.axhline(0,color="#30363d",lw=0.8)
        self.ax_rpm.plot(xs,tl,color="#1f6feb",lw=1.0,ls="--",alpha=0.6,label="Tgt IZQ")
        self.ax_rpm.plot(xs,tr,color="#da3633",lw=1.0,ls="--",alpha=0.6,label="Tgt DER")
        self.ax_rpm.plot(xs,r1,color="#58a6ff",lw=1.8,label="M1 IZQ-F")
        self.ax_rpm.plot(xs,r2,color="#f78166",lw=1.8,label="M2 DER-F")
        self.ax_rpm.plot(xs,r3,color="#85b7eb",lw=1.2,ls="-.",label="M3 IZQ-T")
        self.ax_rpm.plot(xs,r4,color="#ffa657",lw=1.2,ls="-.",label="M4 DER-T")
        self.ax_rpm.set_title("RPM vs Target — 4 motores",color="#8b949e",fontsize=9,pad=4)
        self.ax_rpm.set_ylabel("RPM",color="#8b949e",fontsize=8)
        self.ax_rpm.tick_params(colors="#484f58",labelsize=7)
        for sp in self.ax_rpm.spines.values(): sp.set_edgecolor("#30363d")
        self.ax_rpm.legend(loc="upper left",fontsize=7,framealpha=0.7,
                           facecolor="#161b22",edgecolor="#30363d",labelcolor="#c9d1d9",
                           ncol=2)

        diff_f = [a-b for a,b in zip(r1,r2)]
        diff_t = [a-b for a,b in zip(r3,r4)]
        self.ax_dif.clear()
        self.ax_dif.set_facecolor("#161b22")
        self.ax_dif.grid(True,alpha=0.15,color="#30363d")
        self.ax_dif.axhline(0,color="#3fb950",lw=0.8,alpha=0.5)
        self.ax_dif.fill_between(xs,diff_f,alpha=0.15,color="#58a6ff")
        self.ax_dif.fill_between(xs,diff_t,alpha=0.15,color="#ffa657")
        self.ax_dif.plot(xs,diff_f,color="#58a6ff",lw=1.5,label="M1−M2 (frontales)")
        self.ax_dif.plot(xs,diff_t,color="#ffa657",lw=1.5,label="M3−M4 (traseros)")
        self.ax_dif.set_title("Diferencia IZQ−DER  (0 = recto)",
                              color="#8b949e",fontsize=9,pad=4)
        self.ax_dif.set_ylabel("ΔRPM",color="#8b949e",fontsize=8)
        self.ax_dif.set_xlabel("muestras",color="#8b949e",fontsize=8)
        self.ax_dif.tick_params(colors="#484f58",labelsize=7)
        for sp in self.ax_dif.spines.values(): sp.set_edgecolor("#30363d")
        self.ax_dif.legend(loc="upper left",fontsize=7,framealpha=0.7,
                           facecolor="#161b22",edgecolor="#30363d",labelcolor="#c9d1d9")
        self.figure.canvas.draw_idle()

    def _mkd(self, name, color):
        l = QLabel(f"{name}\n0.0 RPM")
        l.setAlignment(Qt.AlignCenter)
        l.setFont(QFont("Courier New",10,QFont.Bold))
        l.setStyleSheet(f"color:{color};background:#0d1117;"
                        f"border:1px solid #30363d;border-radius:10px;padding:8px;")
        l.setMinimumHeight(68)
        return l

    def closeEvent(self, e):
        global _last_dpad_cmd, _last_l, _last_r
        _last_dpad_cmd = None
        _last_l = 0.0
        _last_r = 0.0
        serial_send_immediate("stop")
        pygame.quit()
        super().closeEvent(e)

# ═══════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════
def main():
    try:    import serial
    except: import subprocess; subprocess.run([sys.executable,"-m","pip","install","pyserial"],check=True)

    connect_serial(ESP32_PORT, SERIAL_BAUD)
    threading.Thread(target=reader_thread, daemon=True).start()
    threading.Thread(target=writer_thread, daemon=True).start()

    pygame.init(); pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("ERROR: No se detectó ningún control Xbox."); sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"✓ Mando: {joy.get_name()}")

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = Dashboard(joy)
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()