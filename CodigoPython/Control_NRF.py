"""
Robot PID — Control Xbox 360
PC → Serial USB → ESP32 (NRF24 TX) ~~RF~~ Arduino Mega (NRF24 RX) → Motores
================================================================================
Requiere:
    pip install pyserial pygame PyQt5 matplotlib

Pasos:
    1. Sube esp32_nrf24_tx.ino al ESP32
    2. Sube mega_nrf24_rx.ino + uno_pid_tb6612.ino al Arduino Mega
       (ambos archivos en la misma carpeta del sketch)
    3. Conecta el mando Xbox 360 al PC (USB o receptor)
    4. Conecta el ESP32 al PC por USB
    5. Cambia ESP32_PORT al COM del ESP32
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
#  CONFIGURACIÓNn
# ═══════════════════════════════════════════════════════
ESP32_PORT  = "COM3"      # Puerto COM del ESP32 transmisor
SERIAL_BAUD = 115200

MAX_RPM    = 120.0        # RPM máxima enviada
DPAD_RPM   = 100.0         # RPM fija D-Pad
DEAD_JOY   = 0.08         # Zona muerta joystick
DEAD_TRIG  = 0.03         # Zona muerta gatillos

# Índices de ejes Xbox 360 estándar (Windows/Linux)
AX_JOY_IZQ_X = 0
AX_LT        = 4
AX_RT        = 5

# Cada cuántos ciclos (50 Hz) se reenvía el setpoint
# Necesario para mantener vivo el watchdog del Mega (2s)
RESEND_EVERY = 8    # cada ~160 ms — bien por debajo del watchdog de 2s

# ═══════════════════════════════════════════════════════
#  CONEXIÓN SERIAL (al ESP32)
# ═══════════════════════════════════════════════════════
_ser       = None
_serial_ok = False
_esp_ready = False     # True cuando el ESP32 responde "OK:ESP32_TX_READY"

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
    print("  No se pudo conectar al ESP32 — modo simulación")

def serial_send(cmd: str):
    """Envía un comando al ESP32 por Serial."""
    if _serial_ok and _ser:
        try:
            _ser.write((cmd + "\n").encode())
        except Exception as e:
            print(f"  Serial error: {e}")
    else:
        print(f"[SIM] {cmd}")

# ═══════════════════════════════════════════════════════
#  HILO LECTOR — respuestas del ESP32 + telemetría del Mega
# ═══════════════════════════════════════════════════════
rpm1_live = 0.0; rpm2_live = 0.0
tgt1_live = 0.0; tgt2_live = 0.0
rf_ok     = False     # último paquete RF llegó con OK
rf_fails  = 0         # contador de fallos RF consecutivos

_hist  = 300
d_rpm1 = deque(maxlen=_hist); d_rpm2 = deque(maxlen=_hist)
d_tgt1 = deque(maxlen=_hist); d_tgt2 = deque(maxlen=_hist)

def reader_thread():
    global rpm1_live, rpm2_live, tgt1_live, tgt2_live
    global rf_ok, rf_fails, _esp_ready
    while True:
        if not _serial_ok or _ser is None:
            time.sleep(0.2); continue
        try:
            raw = _ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            # Confirmación de envío RF
            if raw.startswith("OK:"):
                rf_ok = True; rf_fails = 0
                payload = raw[3:]
                if payload == "ESP32_TX_READY":
                    _esp_ready = True
                    print("✓ ESP32 transmisor listo")

            elif raw.startswith("ERR:RF_FAIL"):
                rf_ok = False; rf_fails += 1
                print(f"  RF fallo #{rf_fails}: {raw}")

            elif raw.startswith("ERR:NRF24"):
                print(f"  ⚠ {raw}")

            # Telemetría del Mega via ACK payload
            # Formato: "R1:XX T1:XX R2:XX T2:XX"
            elif raw.startswith("R1:"):
                try:
                    parts = {}
                    for tok in raw.split():
                        if ":" in tok:
                            k, v = tok.split(":", 1)
                            parts[k] = float(v)
                    rpm1_live = parts.get("R1", rpm1_live)
                    tgt1_live = parts.get("T1", tgt1_live)
                    rpm2_live = parts.get("R2", rpm2_live)
                    tgt2_live = parts.get("T2", tgt2_live)
                    d_rpm1.append(rpm1_live); d_rpm2.append(rpm2_live)
                    d_tgt1.append(tgt1_live); d_tgt2.append(tgt2_live)
                except Exception:
                    pass

            # Telemetría completa si el Mega también está conectado por Serial
            elif "RPM1:" in raw and "Target1:" in raw:
                try:
                    p = {}
                    for tok in raw.split("\t"):
                        if ":" in tok:
                            k, v = tok.split(":", 1)
                            p[k.strip()] = float(v.strip())
                    rpm1_live = p.get("RPM1",    rpm1_live)
                    rpm2_live = p.get("RPM2",    rpm2_live)
                    tgt1_live = p.get("Target1", tgt1_live)
                    tgt2_live = p.get("Target2", tgt2_live)
                    d_rpm1.append(rpm1_live); d_rpm2.append(rpm2_live)
                    d_tgt1.append(tgt1_live); d_tgt2.append(tgt2_live)
                except Exception:
                    pass

        except Exception:
            pass

# ═══════════════════════════════════════════════════════
#  LÓGICA DE CONTROL XBOX
# ═══════════════════════════════════════════════════════
_last_m1 = None
_last_m2 = None

def set_motors(m1: float, m2: float, force: bool = False):
    global _last_m1, _last_m2
    m1 = round(max(-MAX_RPM, min(MAX_RPM, m1)), 1)
    m2 = round(max(-MAX_RPM, min(MAX_RPM, m2)), 1)
    if force or _last_m1 is None or abs(m1 - (_last_m1 or 0)) >= 1.0:
        serial_send(f"m1={m1}"); _last_m1 = m1
    if force or _last_m2 is None or abs(m2 - (_last_m2 or 0)) >= 1.0:
        serial_send(f"m2={m2}"); _last_m2 = m2

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
    lbl.setStyleSheet(f"color:{color};"); lbl.setFixedWidth(60); hl.addWidget(lbl)
    inputs = {}
    for tag, val in [("Kp",kp),("Ki",ki),("Kd",kd)]:
        tl  = QLabel(tag); tl.setFont(QFont("Courier New",9))
        tl.setStyleSheet("color:#6b7fa3;")
        inp = QLineEdit(str(val))
        inp.setFont(QFont("Courier New",11)); inp.setFixedWidth(76)
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
    "QTabBar::tab{background:#0d1117;color:#8b949e;padding:8px 14px;"
    "font-family:'Courier New';font-size:9pt;border:1px solid #30363d;"
    "border-bottom:none;border-radius:6px 6px 0 0;margin-right:2px;}"
    "QTabBar::tab:selected{background:#161b22;color:#f0f6fc;border-color:#58a6ff;}"
    "QCheckBox{font-family:'Courier New';font-size:9pt;color:#8b949e;padding:3px;}"
    "QCheckBox::indicator{width:14px;height:14px;border-radius:3px;"
    "border:1px solid #30363d;background:#0d1117;}"
    "QCheckBox::indicator:checked{background:#1f6feb;border-color:#58a6ff;}"
)

# ═══════════════════════════════════════════════════════
#  VENTANA PRINCIPAL
# ═══════════════════════════════════════════════════════
class Dashboard(QWidget):
    def __init__(self, joy):
        super().__init__()
        self.joy = joy
        self.setWindowTitle(
            f"PID Robot  —  Xbox 360  |  ESP32→NRF24→Mega  |  {ESP32_PORT}")
        self.setMinimumSize(1220, 880)
        self.setStyleSheet(STYLE)

        self._cycle      = 0
        self._mode_str   = "DETENIDO"
        self._mode_color = "#484f58"
        self._m1_cmd     = 0.0
        self._m2_cmd     = 0.0

        self._build_ui()

        self.t_ctrl = QTimer(); self.t_ctrl.timeout.connect(self._ctrl_tick); self.t_ctrl.start(20)
        self.t_disp = QTimer(); self.t_disp.timeout.connect(self._upd_disp); self.t_disp.start(60)
        self.t_plot = QTimer(); self.t_plot.timeout.connect(self._upd_plot); self.t_plot.start(250)
        self.t_conn = QTimer(); self.t_conn.timeout.connect(self._upd_conn); self.t_conn.start(400)

    # ── Tick de control Xbox (50 Hz) ──
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
            m1o, m2o = +DPAD_RPM, -DPAD_RPM
            set_motors(m1o, m2o, force=force)
            self._m1_cmd, self._m2_cmd = m1o, m2o
            self._mode_str   = f"D-PAD ←  giro IZQ"
            self._mode_color = "#d29922"
            use_steering     = False
        elif dp[0] == 1:
            m1o, m2o = -DPAD_RPM, +DPAD_RPM
            set_motors(m1o, m2o, force=force)
            self._m1_cmd, self._m2_cmd = m1o, m2o
            self._mode_str   = f"D-PAD →  giro DER"
            self._mode_color = "#d29922"
            use_steering     = False
        else:
            if _last_m1 != 0.0 or _last_m2 != 0.0:
                set_motors(0.0, 0.0, force=True)
            self._m1_cmd, self._m2_cmd = 0.0, 0.0
            self._mode_str   = "DETENIDO"
            self._mode_color = "#484f58"
            use_steering     = False

        if use_steering:
            m1o, m2o = apply_steering(base_rpm, jx)
            set_motors(m1o, m2o, force=force)
            self._m1_cmd, self._m2_cmd = m1o, m2o

    # ── Construcción UI ──
    def _build_ui(self):
        root = QVBoxLayout(self); root.setSpacing(8); root.setContentsMargins(14,10,14,10)

        # Header
        hdr = QHBoxLayout()
        title = QLabel("PID ROBOT  —  Xbox 360  +  ESP32 NRF24  →  Arduino Mega")
        title.setFont(QFont("Courier New",13,QFont.Bold))
        title.setStyleSheet("color:#58a6ff;letter-spacing:1px;")
        sub = QLabel(
            f"ESP32: {ESP32_PORT}  |  RT=avance  LT=retroceso  "
            "D-Pad=dirección  Joy IZQ=giro  |  Comandos → RF → Mega")
        sub.setFont(QFont("Courier New",9)); sub.setStyleSheet("color:#484f58;")
        vt = QVBoxLayout(); vt.addWidget(title); vt.addWidget(sub)
        hdr.addLayout(vt); hdr.addStretch()

        # Estado de conexión + RF
        conn_box = QVBoxLayout()
        self.conn_lbl = QLabel("CONECTANDO...")
        self.conn_lbl.setFont(QFont("Courier New",9,QFont.Bold))
        self.conn_lbl.setStyleSheet(
            "color:#d29922;padding:3px 8px;border:1px solid #30363d;"
            "border-radius:5px;background:#161b22;")
        self.rf_lbl = QLabel("RF: —")
        self.rf_lbl.setFont(QFont("Courier New",9,QFont.Bold))
        self.rf_lbl.setStyleSheet(
            "color:#484f58;padding:3px 8px;border:1px solid #30363d;"
            "border-radius:5px;background:#161b22;")
        conn_box.addWidget(self.conn_lbl); conn_box.addWidget(self.rf_lbl)
        hdr.addLayout(conn_box)
        root.addLayout(hdr)

        # Displays RPM
        dr = QHBoxLayout(); dr.setSpacing(10)
        self.disp1 = self._mkd("M1  DERECHO",   "#58a6ff")
        self.disp2 = self._mkd("M2  IZQUIERDO", "#f78166")
        self.mode_lbl = QLabel("DETENIDO")
        self.mode_lbl.setAlignment(Qt.AlignCenter)
        self.mode_lbl.setFont(QFont("Courier New",11,QFont.Bold))
        self.mode_lbl.setStyleSheet(
            "color:#484f58;background:#0d1117;border:1px solid #30363d;"
            "border-radius:10px;padding:10px;min-width:160px;")
        self.mode_lbl.setMinimumHeight(68)
        dr.addWidget(self.disp1,3); dr.addWidget(self.disp2,3); dr.addWidget(self.mode_lbl,2)
        root.addLayout(dr)

        # Centro
        ctr = QHBoxLayout(); ctr.setSpacing(10)
        lft = QVBoxLayout(); lft.setSpacing(8)
        lft.addWidget(self._ctrl_panel())
        lft.addWidget(self._pid_tabs())
        lft.addStretch()
        self._build_plot()
        ctr.addLayout(lft,40); ctr.addWidget(self.canvas,60)
        root.addLayout(ctr)

    def _ctrl_panel(self):
        box = QGroupBox("CONTROL  —  Xbox 360  +  Comandos directos")
        vl  = QVBoxLayout(box); vl.setSpacing(7)

        # Mapa controles
        ctrl_map = [
            ("RT", "Avance proporcional  0–120 RPM", "#3fb950"),
            ("LT", "Retroceso proporcional  0–120 RPM", "#f85149"),
            ("D-Pad ↑↓", f"Avance/retroceso fijo ±{DPAD_RPM:.0f} RPM", "#58a6ff"),
            ("D-Pad ←→", "Giro en sitio", "#d29922"),
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

        # Comando manual directo al Mega via RF
        cr = QHBoxLayout()
        cl2 = QLabel("Comando RF:"); cl2.setFont(QFont("Courier New",9))
        cl2.setStyleSheet("color:#8b949e;")
        self.cmd_inp = QLineEdit()
        self.cmd_inp.setPlaceholderText("kp1r=1.6   sync=1   diag ...")
        self.cmd_inp.setFont(QFont("Courier New",10)); self.cmd_inp.setMinimumWidth(180)
        self.cmd_inp.setStyleSheet(
            "background:#0d1117;color:#c9d1d9;border:1px solid #30363d;"
            "border-radius:5px;padding:4px;")
        self.cmd_inp.returnPressed.connect(self._send_manual)
        send_btn = sbtn("Enviar", "#161b22", "#30363d", 30)
        send_btn.setFixedWidth(65)
        send_btn.clicked.connect(self._send_manual)
        cr.addWidget(cl2); cr.addWidget(self.cmd_inp); cr.addWidget(send_btn)
        vl.addLayout(cr)

        # RPM máxima + Fix sign
        rr = QHBoxLayout()
        rl = QLabel("RPM máx:"); rl.setFont(QFont("Courier New",9))
        rl.setStyleSheet("color:#8b949e;")
        self.rpm_inp = QLineEdit(str(int(MAX_RPM)))
        self.rpm_inp.setFont(QFont("Courier New",10)); self.rpm_inp.setFixedWidth(60)
        self.rpm_inp.setAlignment(Qt.AlignCenter)
        self.rpm_inp.setStyleSheet(
            "background:#0d1117;color:#c9d1d9;border:1px solid #30363d;"
            "border-radius:5px;padding:3px;")
        self.rpm_inp.editingFinished.connect(self._upd_max_rpm)
        self.fix_cb = QCheckBox("Fix sign")
        self.fix_cb.setChecked(True)
        self.fix_cb.stateChanged.connect(
            lambda s: serial_send("fixsign=1" if s else "fixsign=0"))
        rr.addWidget(rl); rr.addWidget(self.rpm_inp); rr.addSpacing(12)
        rr.addWidget(self.fix_cb); rr.addStretch()
        vl.addLayout(rr)

        # Sync
        sb = QGroupBox("SINCRONIZACIÓN")
        sb.setStyleSheet(
            "QGroupBox{background:#0d1117;border:1px solid #1f6feb;border-radius:6px;"
            "margin-top:6px;padding:8px;color:#58a6ff;font-family:'Courier New';"
            "font-size:9pt;}QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;}")
        sl = QVBoxLayout(sb)
        kr = QHBoxLayout()
        kl = QLabel("Kp sync:"); kl.setFont(QFont("Courier New",9))
        kl.setStyleSheet("color:#8b949e;")
        self.kpsync = QLineEdit("0.5")
        self.kpsync.setFont(QFont("Courier New",10)); self.kpsync.setFixedWidth(60)
        self.kpsync.setAlignment(Qt.AlignCenter)
        self.kpsync.setStyleSheet(
            "background:#0d1117;color:#c9d1d9;border:1px solid #30363d;"
            "border-radius:5px;padding:3px;")
        kr.addWidget(kl); kr.addWidget(self.kpsync); kr.addStretch()
        sl.addLayout(kr)
        scb = QCheckBox("Activar sync lineal")
        scb.setStyleSheet("color:#58a6ff;font-family:'Courier New';font-size:9pt;")
        scb.stateChanged.connect(self._tog_sync)
        sl.addWidget(scb)
        vl.addWidget(sb)

        # STOP
        stop_btn = sbtn("STOP EMERGENCIA", "#3d0c0c", "#f85149", 40)
        stop_btn.clicked.connect(self._emergency_stop)
        vl.addWidget(stop_btn)
        return box

    def _pid_tabs(self):
        tabs = QTabWidget()
        tf = QWidget(); tf.setStyleSheet("background:#161b22;")
        vf = QVBoxLayout(tf); vf.setSpacing(6)
        vf.addWidget(self._note("Activo cuando setpoint > 0  (avance)", "#3fb950"))
        r1f,self.kp1f,self.ki1f,self.kd1f = pid_row("M1 DER","#58a6ff",1.35,0.500,0.038)
        r2f,self.kp2f,self.ki2f,self.kd2f = pid_row("M2 IZQ","#f78166",1.35,0.545,0.028)
        vf.addWidget(r1f); vf.addWidget(r2f)
        bf = sbtn("Aplicar PID Avance","#0e2a47","#1f6feb",32)
        bf.clicked.connect(self._spf); vf.addWidget(bf); vf.addStretch()

        tr = QWidget(); tr.setStyleSheet("background:#161b22;")
        vr = QVBoxLayout(tr); vr.setSpacing(6)
        vr.addWidget(self._note("Activo cuando setpoint < 0  (retroceso)", "#f85149"))
        r1r,self.kp1r,self.ki1r,self.kd1r = pid_row("M1 DER","#ff7b72",1.35,0.500,0.038)
        r2r,self.kp2r,self.ki2r,self.kd2r = pid_row("M2 IZQ","#ffa657",1.35,0.545,0.028)
        vr.addWidget(r1r); vr.addWidget(r2r)
        br = sbtn("Aplicar PID Retroceso","#3d0c0c","#da3633",32)
        br.clicked.connect(self._spr); vr.addWidget(br); vr.addStretch()

        tabs.addTab(tf,"AVANCE"); tabs.addTab(tr,"RETROCESO")
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
                               hspace=0.48,top=0.93,bottom=0.08,left=0.09,right=0.97)
        self.ax_rpm = self.figure.add_subplot(gs[0])
        self.ax_err = self.figure.add_subplot(gs[1])
        for ax in (self.ax_rpm, self.ax_err):
            ax.set_facecolor("#161b22"); ax.tick_params(colors="#484f58",labelsize=8)
            ax.grid(True,alpha=0.15,color="#30363d")
            ax.axhline(0,color="#30363d",linewidth=0.8)
            for sp in ax.spines.values(): sp.set_edgecolor("#30363d")
        self.ax_rpm.set_title("RPM vs Target (via ACK payload)",
                              color="#8b949e",fontsize=9,pad=4)
        self.ax_err.set_title("Diferencia M1 - M2",
                              color="#8b949e",fontsize=9,pad=4)

    # ── Acciones ──
    def _emergency_stop(self):
        set_motors(0, 0, force=True)
        serial_send("stop")

    def _send_manual(self):
        cmd = self.cmd_inp.text().strip()
        if cmd:
            serial_send(cmd)
            self.cmd_inp.clear()

    def _upd_max_rpm(self):
        global MAX_RPM
        try:
            v = max(10., min(150., abs(float(self.rpm_inp.text()))))
            MAX_RPM = v; self.rpm_inp.setText(str(int(v)))
        except ValueError: pass

    def _tog_sync(self, s):
        if s: serial_send(f"kpsync={self.kpsync.text()}"); serial_send("sync=1")
        else: serial_send("sync=0")

    def _spf(self):
        try:
            for c in [f"kp1f={float(self.kp1f.text())}",f"ki1f={float(self.ki1f.text())}",
                      f"kd1f={float(self.kd1f.text())}",f"kp2f={float(self.kp2f.text())}",
                      f"ki2f={float(self.ki2f.text())}",f"kd2f={float(self.kd2f.text())}"]:
                serial_send(c); time.sleep(0.06)
            print("PID Avance enviado")
        except ValueError: print("Valor PID inválido")

    def _spr(self):
        try:
            for c in [f"kp1r={float(self.kp1r.text())}",f"ki1r={float(self.ki1r.text())}",
                      f"kd1r={float(self.kd1r.text())}",f"kp2r={float(self.kp2r.text())}",
                      f"ki2r={float(self.ki2r.text())}",f"kd2r={float(self.kd2r.text())}"]:
                serial_send(c); time.sleep(0.06)
            print("PID Retroceso enviado")
        except ValueError: print("Valor PID inválido")

    # ── Refresh UI ──
    def _upd_conn(self):
        if _serial_ok and _esp_ready:
            self.conn_lbl.setText(f"ESP32 OK  {ESP32_PORT}")
            self.conn_lbl.setStyleSheet(
                "color:#3fb950;padding:3px 8px;border:1px solid #30363d;"
                "border-radius:5px;background:#161b22;")
        elif _serial_ok:
            self.conn_lbl.setText(f"Serial OK — esperando ESP32...")
            self.conn_lbl.setStyleSheet(
                "color:#d29922;padding:3px 8px;border:1px solid #30363d;"
                "border-radius:5px;background:#161b22;")
        else:
            self.conn_lbl.setText("SIN SERIAL — verifica el puerto")
            self.conn_lbl.setStyleSheet(
                "color:#f85149;padding:3px 8px;border:1px solid #30363d;"
                "border-radius:5px;background:#161b22;")

        if rf_fails == 0 and _esp_ready:
            self.rf_lbl.setText("RF: OK")
            self.rf_lbl.setStyleSheet(
                "color:#3fb950;padding:3px 8px;border:1px solid #30363d;"
                "border-radius:5px;background:#161b22;")
        elif rf_fails > 0:
            self.rf_lbl.setText(f"RF: {rf_fails} fallos")
            col = "#f85149" if rf_fails > 3 else "#d29922"
            self.rf_lbl.setStyleSheet(
                f"color:{col};padding:3px 8px;border:1px solid #30363d;"
                f"border-radius:5px;background:#161b22;")

    def _upd_disp(self):
        d1 = "FWD" if rpm1_live>=0 else "REV"
        d2 = "FWD" if rpm2_live>=0 else "REV"
        c1 = "#58a6ff" if rpm1_live>=0 else "#ff7b72"
        c2 = "#f78166" if rpm2_live>=0 else "#ffa657"
        self.disp1.setText(f"M1 DER [{d1}]\n{rpm1_live:+.1f} RPM")
        self.disp1.setStyleSheet(
            f"color:{c1};background:#0d1117;border:1px solid #30363d;"
            f"border-radius:10px;padding:10px;")
        self.disp2.setText(f"M2 IZQ [{d2}]\n{rpm2_live:+.1f} RPM")
        self.disp2.setStyleSheet(
            f"color:{c2};background:#0d1117;border:1px solid #30363d;"
            f"border-radius:10px;padding:10px;")
        self.mode_lbl.setText(
            f"{self._mode_str}\nM1={self._m1_cmd:+.0f}  M2={self._m2_cmd:+.0f}")
        self.mode_lbl.setStyleSheet(
            f"color:{self._mode_color};background:#0d1117;"
            f"border:1px solid #30363d;border-radius:10px;"
            f"padding:10px;min-width:160px;font-family:'Courier New';"
            f"font-size:10pt;font-weight:bold;")

    def _upd_plot(self):
        if not d_rpm1: return
        r1=list(d_rpm1); r2=list(d_rpm2)
        t1=list(d_tgt1); t2=list(d_tgt2)
        xs=range(len(r1))
        diff=[a-b for a,b in zip(r1,r2)]

        self.ax_rpm.clear()
        self.ax_rpm.set_facecolor("#161b22")
        self.ax_rpm.grid(True,alpha=0.15,color="#30363d")
        self.ax_rpm.axhline(0,color="#30363d",lw=0.8)
        self.ax_rpm.plot(xs,t1,color="#1f6feb",lw=1.2,ls="--",alpha=0.7,label="Tgt M1")
        self.ax_rpm.plot(xs,t2,color="#da3633",lw=1.2,ls="--",alpha=0.7,label="Tgt M2")
        self.ax_rpm.plot(xs,r1,color="#58a6ff",lw=2.0,label="M1 DER")
        self.ax_rpm.plot(xs,r2,color="#f78166",lw=2.0,label="M2 IZQ")
        self.ax_rpm.set_title("RPM vs Target",color="#8b949e",fontsize=9,pad=4)
        self.ax_rpm.set_ylabel("RPM",color="#8b949e",fontsize=8)
        self.ax_rpm.tick_params(colors="#484f58",labelsize=7)
        for sp in self.ax_rpm.spines.values(): sp.set_edgecolor("#30363d")
        self.ax_rpm.legend(loc="upper left",fontsize=7,framealpha=0.7,
                           facecolor="#161b22",edgecolor="#30363d",labelcolor="#c9d1d9")

        self.ax_err.clear()
        self.ax_err.set_facecolor("#161b22")
        self.ax_err.grid(True,alpha=0.15,color="#30363d")
        self.ax_err.axhline(0,color="#3fb950",lw=0.8,alpha=0.5)
        self.ax_err.fill_between(xs,diff,alpha=0.2,color="#d29922")
        self.ax_err.plot(xs,diff,color="#d29922",lw=1.8,label="M1−M2")
        self.ax_err.set_title("Diferencia RPM  M1 − M2  (0 = sincronizado)",
                              color="#8b949e",fontsize=9,pad=4)
        self.ax_err.set_ylabel("ΔRPM",color="#8b949e",fontsize=8)
        self.ax_err.set_xlabel("muestras",color="#8b949e",fontsize=8)
        self.ax_err.tick_params(colors="#484f58",labelsize=7)
        for sp in self.ax_err.spines.values(): sp.set_edgecolor("#30363d")
        self.ax_err.legend(loc="upper left",fontsize=7,framealpha=0.7,
                           facecolor="#161b22",edgecolor="#30363d",labelcolor="#c9d1d9")
        self.figure.canvas.draw_idle()

    def _mkd(self, name, color):
        l = QLabel(f"{name}\n0.0 RPM")
        l.setAlignment(Qt.AlignCenter)
        l.setFont(QFont("Courier New",12,QFont.Bold))
        l.setStyleSheet(
            f"color:{color};background:#0d1117;border:1px solid #30363d;"
            f"border-radius:10px;padding:10px;")
        l.setMinimumHeight(68)
        return l

    def closeEvent(self, e):
        set_motors(0, 0, force=True)
        serial_send("stop")
        pygame.quit()
        super().closeEvent(e)


# ═══════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════
def main():
    for pkg in ["serial","pygame"]:
        try: __import__(pkg)
        except ImportError:
            import subprocess
            subprocess.run([sys.executable,"-m","pip","install",
                            "pyserial" if pkg=="serial" else pkg],check=True)

    connect_serial(ESP32_PORT, SERIAL_BAUD)
    threading.Thread(target=reader_thread, daemon=True).start()

    pygame.init(); pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("ERROR: No se detectó ningún control Xbox.")
        sys.exit(1)

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