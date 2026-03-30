"""
Arduino Mega — PID Robot 4 Motores / 2 Orugas
Control Xbox 360 via Serial USB
================================================
Requiere:
  pip install pyserial pygame PyQt5 matplotlib

Pasos:
  1. Sube mega_pid_4motores.ino al Arduino Mega
  2. Conecta el mando Xbox 360
  3. Cambia SERIAL_PORT con el COM de tu Arduino
  4. Ejecuta este script
"""

import sys, serial, threading, time
from collections import deque

import pygame
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QCheckBox, QGroupBox, QTabWidget, QSplitter)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.gridspec as gridspec

# ═══════════════════════════════════════════════════════
#  CONFIGURACIÓN
# ═══════════════════════════════════════════════════════
SERIAL_PORT  = "COM4"       # Windows: "COM3"  /  Linux: "/dev/ttyUSB0"
SERIAL_BAUD  = 115200

MAX_RPM      = 150.0
DPAD_RPM     = 100.0
DEAD_JOY     = 0.08
DEAD_TRIG    = 0.03
RESEND_EVERY = 10           # cada 200 ms — mantiene vivo el watchdog

# Índices de ejes Xbox 360 estándar Windows
AX_JOY_IZQ_X = 0
AX_LT        = 4
AX_RT        = 5

# ═══════════════════════════════════════════════════════
#  SERIAL
# ═══════════════════════════════════════════════════════
_ser       = None
_serial_ok = False

def connect_serial(port, baud, retries=5):
    global _ser, _serial_ok
    for i in range(retries):
        try:
            _ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
            _serial_ok = True
            print(f"✓ Serial conectado  {port}")
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
#  TELEMETRÍA EN TIEMPO REAL
#  El Arduino envía: TL  TR  RPM1..4  Err1..4  Out1..4
# ═══════════════════════════════════════════════════════
tl_live = 0.0; tr_live = 0.0
rpm_live = [0.0]*4
err_live = [0.0]*4
out_live = [0.0]*4

_hist  = 300
d_rpm  = [deque(maxlen=_hist) for _ in range(4)]
d_tgt  = [deque(maxlen=_hist) for _ in range(4)]   # tgt[0,2]=TL  tgt[1,3]=TR
d_err  = [deque(maxlen=_hist) for _ in range(4)]

def reader_thread():
    global tl_live, tr_live
    while True:
        if not _serial_ok or _ser is None:
            time.sleep(0.2); continue
        try:
            raw = _ser.readline().decode(errors="ignore").strip()
            if "RPM1:" in raw and "TL:" in raw:
                p = {}
                for tok in raw.split("\t"):
                    if ":" in tok:
                        k, v = tok.split(":", 1)
                        p[k.strip()] = v.strip()
                tl_live = float(p.get("TL", 0))
                tr_live = float(p.get("TR", 0))
                for i in range(4):
                    rpm_live[i] = float(p.get(f"RPM{i+1}", 0))
                    err_live[i] = float(p.get(f"Err{i+1}", 0))
                    out_live[i] = float(p.get(f"Out{i+1}", 0))
                    d_rpm[i].append(rpm_live[i])
                    d_err[i].append(err_live[i])
                # Targets: M1,M3 siguen TL;  M2,M4 siguen TR
                d_tgt[0].append(tl_live); d_tgt[2].append(tl_live)
                d_tgt[1].append(tr_live); d_tgt[3].append(tr_live)
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
    if force or _last_L is None or abs(left  - _last_L) >= 1.0:
        serial_send(f"l={left}");  _last_L = left
    if force or _last_R is None or abs(right - _last_R) >= 1.0:
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
    """Mezcla velocidad base + giro con joystick izquierdo."""
    joy_x = dead(joy_x, DEAD_JOY)
    if abs(base_rpm) < 1.0:
        spin = joy_x * MAX_RPM
        return -spin, spin          # giro en sitio
    if joy_x > 0:                   # girar a la derecha
        return base_rpm * (1.0 - 2.0 * joy_x), base_rpm
    else:                           # girar a la izquierda
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
        hl.addWidget(tl); hl.addWidget(f)
        inputs[tag] = f
    hl.addStretch()
    return row, inputs["Kp"], inputs["Ki"], inputs["Kd"]

# ═══════════════════════════════════════════════════════
#  VENTANA PRINCIPAL
# ═══════════════════════════════════════════════════════
class Dashboard(QWidget):
    def __init__(self, joy):
        super().__init__()
        self.joy = joy
        self.setWindowTitle(f"Mega PID 4 Motores — Xbox 360 | {SERIAL_PORT}")
        self.setMinimumSize(1300, 900)
        self.setStyleSheet(STYLE)

        self._cycle      = 0
        self._mode_str   = "DETENIDO"
        self._mode_color = "#484f58"
        self._l_cmd      = 0.0
        self._r_cmd      = 0.0

        self._build_ui()

        self.t_ctrl = QTimer(); self.t_ctrl.timeout.connect(self._ctrl_tick); self.t_ctrl.start(20)
        self.t_disp = QTimer(); self.t_disp.timeout.connect(self._upd_disp); self.t_disp.start(60)
        self.t_plot = QTimer(); self.t_plot.timeout.connect(self._upd_plot); self.t_plot.start(200)
        self.t_conn = QTimer(); self.t_conn.timeout.connect(self._upd_conn); self.t_conn.start(500)

    # ── Tick de control ──────────────────────────────
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
            base_rpm         = rt
            self._mode_str   = f"RT  +{rt:.0f} RPM"
            self._mode_color = "#3fb950"

        elif lt > 0.5:
            base_rpm         = -lt
            self._mode_str   = f"LT  -{lt:.0f} RPM"
            self._mode_color = "#f85149"

        elif dp[1] == 1:
            base_rpm         = DPAD_RPM
            self._mode_str   = f"D-PAD ↑  +{DPAD_RPM:.0f} RPM"
            self._mode_color = "#58a6ff"

        elif dp[1] == -1:
            base_rpm         = -DPAD_RPM
            self._mode_str   = f"D-PAD ↓  -{DPAD_RPM:.0f} RPM"
            self._mode_color = "#ffa657"

        elif dp[0] == -1:
            lo, ro = +DPAD_RPM, -DPAD_RPM
            set_orugas(lo, ro, force=force)
            self._l_cmd, self._r_cmd = lo, ro
            self._mode_str   = f"D-PAD ←  giro izq"
            self._mode_color = "#d29922"
            use_steering     = False

        elif dp[0] == 1:
            lo, ro = -DPAD_RPM, +DPAD_RPM
            set_orugas(lo, ro, force=force)
            self._l_cmd, self._r_cmd = lo, ro
            self._mode_str   = f"D-PAD →  giro der"
            self._mode_color = "#d29922"
            use_steering     = False

        else:
            if _last_L != 0.0 or _last_R != 0.0:
                set_orugas(0.0, 0.0, force=True)
            self._l_cmd, self._r_cmd = 0.0, 0.0
            self._mode_str   = "DETENIDO"
            self._mode_color = "#484f58"
            use_steering     = False

        if use_steering:
            lo, ro = apply_steering(base_rpm, jx)
            set_orugas(lo, ro, force=force)
            self._l_cmd, self._r_cmd = lo, ro

    # ── Build UI ─────────────────────────────────────
    def _build_ui(self):
        root = QVBoxLayout(self); root.setSpacing(8); root.setContentsMargins(14,10,14,10)

        # Header
        hdr = QHBoxLayout()
        title = QLabel("ARDUINO MEGA  —  PID 4 MOTORES  2×TB6612FNG  XBOX 360")
        title.setFont(QFont("Courier New", 13, QFont.Bold))
        title.setStyleSheet("color:#58a6ff;letter-spacing:1px;")
        sub = QLabel("RT=avance · LT=retroceso · D-Pad=mov fijo · Joy IZQ=dirección")
        sub.setFont(QFont("Courier New", 9)); sub.setStyleSheet("color:#484f58;")
        vt = QVBoxLayout(); vt.addWidget(title); vt.addWidget(sub)
        hdr.addLayout(vt); hdr.addStretch()
        self.conn_lbl = QLabel("CONECTANDO...")
        self.conn_lbl.setFont(QFont("Courier New", 9, QFont.Bold))
        self.conn_lbl.setStyleSheet("color:#d29922;padding:4px 10px;"
                                    "border:1px solid #30363d;border-radius:5px;background:#161b22;")
        hdr.addWidget(self.conn_lbl); root.addLayout(hdr)

        # Displays RPM — 4 motores + modo
        dr = QHBoxLayout(); dr.setSpacing(8)
        COLORS = ["#58a6ff","#3fb950","#ffa657","#f78166"]
        NAMES  = ["M1  IZQ FWD","M2  DER FWD","M3  IZQ ATR","M4  DER ATR"]
        self.disps = []
        for i in range(4):
            l = QLabel(f"{NAMES[i]}\n0.0 RPM")
            l.setAlignment(Qt.AlignCenter)
            l.setFont(QFont("Courier New", 10, QFont.Bold))
            l.setStyleSheet(f"color:{COLORS[i]};background:#0d1117;"
                            f"border:1px solid #30363d;border-radius:8px;padding:8px;")
            l.setMinimumHeight(60)
            self.disps.append(l); dr.addWidget(l, 2)

        self.mode_lbl = QLabel("DETENIDO")
        self.mode_lbl.setAlignment(Qt.AlignCenter)
        self.mode_lbl.setFont(QFont("Courier New", 10, QFont.Bold))
        self.mode_lbl.setStyleSheet("color:#484f58;background:#0d1117;"
                                    "border:1px solid #30363d;border-radius:8px;"
                                    "padding:8px;min-width:150px;")
        self.mode_lbl.setMinimumHeight(60)
        dr.addWidget(self.mode_lbl, 2)
        root.addLayout(dr)

        # Centro — panel izq + gráfica
        ctr = QHBoxLayout(); ctr.setSpacing(10)
        lft = QVBoxLayout(); lft.setSpacing(8)
        lft.addWidget(self._xbox_panel())
        lft.addWidget(self._pid_tabs())
        lft.addStretch()
        self._build_plot()
        ctr.addLayout(lft, 38); ctr.addWidget(self.canvas, 62)
        root.addLayout(ctr)

    # ── Panel Xbox ───────────────────────────────────
    def _xbox_panel(self):
        box = QGroupBox("CONTROL  —  Xbox 360")
        vl  = QVBoxLayout(box); vl.setSpacing(6)

        ctrl_map = [
            ("RT  (der)",   "Avance proporcional  0–150 RPM",         "#3fb950"),
            ("LT  (izq)",   "Retroceso proporcional  0–150 RPM",      "#f85149"),
            ("D-Pad ↑ ↓",  f"Avance/retroceso fijo  ±{DPAD_RPM:.0f} RPM","#58a6ff"),
            ("D-Pad ← →",  "Giro en sitio",                           "#d29922"),
            ("Joy IZQ X",   "Dirección (mezcla orugas)",               "#c9d1d9"),
        ]
        for ctrl, desc, color in ctrl_map:
            row = QHBoxLayout()
            cl = QLabel(ctrl); cl.setFont(QFont("Courier New", 9, QFont.Bold))
            cl.setStyleSheet(f"color:{color};"); cl.setFixedWidth(110)
            dl = QLabel(desc); dl.setFont(QFont("Courier New", 9))
            dl.setStyleSheet("color:#6b7fa3;")
            row.addWidget(cl); row.addWidget(dl); row.addStretch()
            vl.addLayout(row)

        # RPM máxima
        rr = QHBoxLayout()
        rl = QLabel("RPM máx:"); rl.setFont(QFont("Courier New", 10))
        rl.setStyleSheet("color:#8b949e;")
        self.rpm_inp = inp_field(str(int(MAX_RPM)), 70)
        self.rpm_inp.editingFinished.connect(self._upd_max_rpm)
        rr.addWidget(rl); rr.addWidget(self.rpm_inp)
        rr.addWidget(QLabel("(10–150)")); rr.addStretch()
        vl.addLayout(rr)

        # Fix sign + diag + STOP
        dg = QHBoxLayout()
        self.fix_cb = QCheckBox("Fix sign encoder")
        self.fix_cb.setChecked(True)
        self.fix_cb.stateChanged.connect(
            lambda s: serial_send("fixsign=1" if s else "fixsign=0"))
        diag_btn = sbtn("diag", "#161b22", "#30363d", 26)
        diag_btn.setFixedWidth(55)
        diag_btn.clicked.connect(lambda: serial_send("diag"))
        dg.addWidget(self.fix_cb); dg.addWidget(diag_btn); dg.addStretch()
        vl.addLayout(dg)

        stop_btn = sbtn("STOP EMERGENCIA", "#3d0c0c", "#f85149", 40)
        stop_btn.clicked.connect(lambda: (set_orugas(0,0,force=True),
                                          serial_send("stop")))
        vl.addWidget(stop_btn)
        return box

    # ── Tabs PID ─────────────────────────────────────
    def _pid_tabs(self):
        tabs = QTabWidget()

        # ── Avance ──
        tf = QWidget(); tf.setStyleSheet("background:#161b22;")
        vf = QVBoxLayout(tf); vf.setSpacing(4)
        note = QLabel("Activo cuando setpoint > 0  (avance)")
        note.setFont(QFont("Courier New", 8)); note.setStyleSheet("color:#3fb950;padding:2px 4px;")
        vf.addWidget(note)

        lbl_oruga = QLabel("— Oruga IZQ (M1 + M3) —")
        lbl_oruga.setFont(QFont("Courier New", 8, QFont.Bold))
        lbl_oruga.setStyleSheet("color:#58a6ff;padding:2px 6px;")
        vf.addWidget(lbl_oruga)

        r1f,self.kp1f,self.ki1f,self.kd1f = pid_row_4("M1 fwd","#58a6ff",1.35,0.500,0.038)
        r3f,self.kp3f,self.ki3f,self.kd3f = pid_row_4("M3 fwd","#4ec9b0",1.35,0.500,0.038)
        vf.addWidget(r1f); vf.addWidget(r3f)

        lbl_oruga2 = QLabel("— Oruga DER (M2 + M4) —")
        lbl_oruga2.setFont(QFont("Courier New", 8, QFont.Bold))
        lbl_oruga2.setStyleSheet("color:#f78166;padding:2px 6px;")
        vf.addWidget(lbl_oruga2)

        r2f,self.kp2f,self.ki2f,self.kd2f = pid_row_4("M2 fwd","#f78166",1.35,0.545,0.028)
        r4f,self.kp4f,self.ki4f,self.kd4f = pid_row_4("M4 fwd","#ffa657",1.35,0.545,0.028)
        vf.addWidget(r2f); vf.addWidget(r4f)

        bf = sbtn("Aplicar PID Avance", "#0e2a47", "#1f6feb", 30)
        bf.clicked.connect(self._send_pid_fwd)
        vf.addWidget(bf); vf.addStretch()

        # ── Retroceso ──
        tr = QWidget(); tr.setStyleSheet("background:#161b22;")
        vr = QVBoxLayout(tr); vr.setSpacing(4)
        note2 = QLabel("Activo cuando setpoint < 0  (retroceso)")
        note2.setFont(QFont("Courier New", 8)); note2.setStyleSheet("color:#f85149;padding:2px 4px;")
        vr.addWidget(note2)

        lbl_or = QLabel("— Oruga IZQ (M1 + M3) —")
        lbl_or.setFont(QFont("Courier New", 8, QFont.Bold))
        lbl_or.setStyleSheet("color:#58a6ff;padding:2px 6px;")
        vr.addWidget(lbl_or)

        r1r,self.kp1r,self.ki1r,self.kd1r = pid_row_4("M1 rev","#ff7b72",1.35,0.500,0.038)
        r3r,self.kp3r,self.ki3r,self.kd3r = pid_row_4("M3 rev","#f0a0a0",1.35,0.500,0.038)
        vr.addWidget(r1r); vr.addWidget(r3r)

        lbl_or2 = QLabel("— Oruga DER (M2 + M4) —")
        lbl_or2.setFont(QFont("Courier New", 8, QFont.Bold))
        lbl_or2.setStyleSheet("color:#f78166;padding:2px 6px;")
        vr.addWidget(lbl_or2)

        r2r,self.kp2r,self.ki2r,self.kd2r = pid_row_4("M2 rev","#ffa657",1.35,0.545,0.028)
        r4r,self.kp4r,self.ki4r,self.kd4r = pid_row_4("M4 rev","#e8c000",1.35,0.545,0.028)
        vr.addWidget(r2r); vr.addWidget(r4r)

        br = sbtn("Aplicar PID Retroceso", "#3d0c0c", "#da3633", 30)
        br.clicked.connect(self._send_pid_rev)
        vr.addWidget(br); vr.addStretch()

        tabs.addTab(tf, "AVANCE"); tabs.addTab(tr, "RETROCESO")
        return tabs

    # ── Gráfica ──────────────────────────────────────
    def _build_plot(self):
        self.figure = Figure(facecolor="#0d1117")
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setStyleSheet("background:#0d1117;border-radius:8px;")
        self.canvas.setFocusPolicy(Qt.NoFocus)
        gs = gridspec.GridSpec(2, 2, figure=self.figure,
                               hspace=0.50, wspace=0.35,
                               top=0.93, bottom=0.07, left=0.08, right=0.97)
        self.axes = [self.figure.add_subplot(gs[r,c])
                     for r in range(2) for c in range(2)]
        titles = ["RPM — Oruga IZQ (M1, M3)", "RPM — Oruga DER (M2, M4)",
                  "Error PID — Oruga IZQ",     "Error PID — Oruga DER"]
        for ax, t in zip(self.axes, titles):
            ax.set_facecolor("#161b22")
            ax.tick_params(colors="#484f58", labelsize=7)
            ax.grid(True, alpha=0.15, color="#30363d")
            ax.axhline(0, color="#30363d", lw=0.8)
            ax.set_title(t, color="#8b949e", fontsize=8, pad=3)
            for sp in ax.spines.values(): sp.set_edgecolor("#30363d")

    # ── Acciones ─────────────────────────────────────
    def _upd_max_rpm(self):
        global MAX_RPM
        try:
            v = max(10., min(150., abs(float(self.rpm_inp.text()))))
            MAX_RPM = v; self.rpm_inp.setText(str(int(v)))
        except ValueError:
            pass

    def _send_pid_fwd(self):
        try:
            cmds = [
                f"kp1f={float(self.kp1f.text())}", f"ki1f={float(self.ki1f.text())}", f"kd1f={float(self.kd1f.text())}",
                f"kp2f={float(self.kp2f.text())}", f"ki2f={float(self.ki2f.text())}", f"kd2f={float(self.kd2f.text())}",
                f"kp3f={float(self.kp3f.text())}", f"ki3f={float(self.ki3f.text())}", f"kd3f={float(self.kd3f.text())}",
                f"kp4f={float(self.kp4f.text())}", f"ki4f={float(self.ki4f.text())}", f"kd4f={float(self.kd4f.text())}",
            ]
            for c in cmds: serial_send(c); time.sleep(0.03)
            print("PID Avance enviado OK")
        except ValueError:
            print("Valor PID inválido")

    def _send_pid_rev(self):
        try:
            cmds = [
                f"kp1r={float(self.kp1r.text())}", f"ki1r={float(self.ki1r.text())}", f"kd1r={float(self.kd1r.text())}",
                f"kp2r={float(self.kp2r.text())}", f"ki2r={float(self.ki2r.text())}", f"kd2r={float(self.kd2r.text())}",
                f"kp3r={float(self.kp3r.text())}", f"ki3r={float(self.ki3r.text())}", f"kd3r={float(self.kd3r.text())}",
                f"kp4r={float(self.kp4r.text())}", f"ki4r={float(self.ki4r.text())}", f"kd4r={float(self.kd4r.text())}",
            ]
            for c in cmds: serial_send(c); time.sleep(0.03)
            print("PID Retroceso enviado OK")
        except ValueError:
            print("Valor PID inválido")

    # ── Refresh UI ────────────────────────────────────
    def _upd_conn(self):
        if _serial_ok:
            self.conn_lbl.setText(f"Serial OK  {SERIAL_PORT}")
            self.conn_lbl.setStyleSheet("color:#3fb950;padding:4px 10px;"
                                        "border:1px solid #30363d;border-radius:5px;background:#161b22;")
        else:
            self.conn_lbl.setText("SIN SERIAL — verifica el puerto")
            self.conn_lbl.setStyleSheet("color:#f85149;padding:4px 10px;"
                                        "border:1px solid #30363d;border-radius:5px;background:#161b22;")

    def _upd_disp(self):
        COLORS_FWD = ["#58a6ff","#3fb950","#ffa657","#f78166"]
        COLORS_REV = ["#ff7b72","#f85149","#d29922","#e05252"]
        NAMES      = ["M1  IZQ FWD","M2  DER FWD","M3  IZQ ATR","M4  DER ATR"]
        for i, (disp, name) in enumerate(zip(self.disps, NAMES)):
            rpm = rpm_live[i]
            color = COLORS_FWD[i] if rpm >= 0 else COLORS_REV[i]
            tag   = "FWD" if rpm >= 0 else "REV"
            disp.setText(f"{name.replace('FWD',tag).replace('ATR',tag)}\n{rpm:+.1f} RPM")
            disp.setStyleSheet(f"color:{color};background:#0d1117;"
                               f"border:1px solid #30363d;border-radius:8px;padding:8px;")
        self.mode_lbl.setText(
            f"{self._mode_str}\nL={self._l_cmd:+.0f}  R={self._r_cmd:+.0f}")
        self.mode_lbl.setStyleSheet(
            f"color:{self._mode_color};background:#0d1117;"
            f"border:1px solid #30363d;border-radius:8px;"
            f"padding:8px;min-width:150px;font-family:'Courier New';"
            f"font-size:10pt;font-weight:bold;")

    def _upd_plot(self):
        if not d_rpm[0]: return
        # Colores: M1=azul M2=verde M3=naranja M4=rojo
        COLORS = ["#58a6ff","#3fb950","#ffa657","#f78166"]
        TGT_C  = ["#1f6feb","#238636","#b06000","#b81c1c"]

        # Pares: [0,2] = oruga izq  [1,3] = oruga der
        for plot_idx, (m1, m2) in enumerate([(0,2),(1,3)]):
            ax = self.axes[plot_idx]
            ax.clear(); ax.set_facecolor("#161b22")
            ax.grid(True, alpha=0.15, color="#30363d")
            ax.axhline(0, color="#30363d", lw=0.8)
            r1 = list(d_rpm[m1]); r2 = list(d_rpm[m2])
            t1 = list(d_tgt[m1])
            xs = range(len(r1))
            ax.plot(xs, t1,  color=TGT_C[m1], lw=1.0, ls="--", alpha=0.6, label="Target")
            ax.plot(xs, r1,  color=COLORS[m1], lw=1.8, label=f"M{m1+1}")
            ax.plot(xs, r2,  color=COLORS[m2], lw=1.8, label=f"M{m2+1}")
            titles = ["RPM — Oruga IZQ (M1, M3)", "RPM — Oruga DER (M2, M4)"]
            ax.set_title(titles[plot_idx], color="#8b949e", fontsize=8, pad=3)
            ax.set_ylabel("RPM", color="#8b949e", fontsize=7)
            ax.tick_params(colors="#484f58", labelsize=7)
            for sp in ax.spines.values(): sp.set_edgecolor("#30363d")
            ax.legend(loc="upper left", fontsize=7, framealpha=0.7,
                      facecolor="#161b22", edgecolor="#30363d", labelcolor="#c9d1d9")

        for plot_idx, (m1, m2) in enumerate([(0,2),(1,3)]):
            ax = self.axes[plot_idx + 2]
            ax.clear(); ax.set_facecolor("#161b22")
            ax.grid(True, alpha=0.15, color="#30363d")
            ax.axhline(0, color="#3fb950", lw=0.8, alpha=0.4)
            e1 = list(d_err[m1]); e2 = list(d_err[m2])
            xs = range(len(e1))
            ax.fill_between(xs, e1, alpha=0.12, color=COLORS[m1])
            ax.fill_between(xs, e2, alpha=0.12, color=COLORS[m2])
            ax.plot(xs, e1, color=COLORS[m1], lw=1.6, label=f"Err M{m1+1}")
            ax.plot(xs, e2, color=COLORS[m2], lw=1.6, label=f"Err M{m2+1}")
            titles2 = ["Error PID — Oruga IZQ", "Error PID — Oruga DER"]
            ax.set_title(titles2[plot_idx], color="#8b949e", fontsize=8, pad=3)
            ax.set_ylabel("error", color="#8b949e", fontsize=7)
            ax.set_xlabel("muestras", color="#8b949e", fontsize=7)
            ax.tick_params(colors="#484f58", labelsize=7)
            for sp in ax.spines.values(): sp.set_edgecolor("#30363d")
            ax.legend(loc="upper left", fontsize=7, framealpha=0.7,
                      facecolor="#161b22", edgecolor="#30363d", labelcolor="#c9d1d9")

        self.figure.canvas.draw_idle()

    def closeEvent(self, e):
        set_orugas(0, 0, force=True)
        serial_send("stop")
        pygame.quit()
        super().closeEvent(e)

# ═══════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════
def main():
    for pkg in ["serial", "pygame"]:
        try:
            __import__(pkg)
        except ImportError:
            import subprocess
            name = "pyserial" if pkg == "serial" else pkg
            subprocess.run([sys.executable, "-m", "pip", "install", name], check=True)

    connect_serial(SERIAL_PORT, SERIAL_BAUD)
    threading.Thread(target=reader_thread, daemon=True).start()

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("ERROR: No se detectó ningún control Xbox.")
        print("Conecta el mando y vuelve a ejecutar.")
        sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"✓ Mando detectado: {joy.get_name()}")
    print(f"  Ejes: {joy.get_numaxes()}  |  Hats: {joy.get_numhats()}")

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = Dashboard(joy)
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()