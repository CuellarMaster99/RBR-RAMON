import sys, serial, threading, time
from collections import deque
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QCheckBox, QGroupBox, QTabWidget)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QKeyEvent
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.gridspec as gridspec

PORT = "COM3"
BAUD = 115200
try:
    ser = serial.Serial(PORT, BAUD, timeout=1); time.sleep(2); SERIAL_OK = True
except Exception as e:
    print(f"Serial no disponible: {e}"); SERIAL_OK = False

BASE_RPM = 50.0
MOVES = {"forward":(+50.,+50.),"backward":(-50.,-50.),"left":(+50.,-50.),"right":(-50.,+50.),"stop":(0.,0.)}

def pid_row(label, color, kp, ki, kd):
    row=QWidget(); hl=QHBoxLayout(row); hl.setContentsMargins(4,2,4,2); hl.setSpacing(8)
    lbl=QLabel(label); lbl.setFont(QFont("Courier New",10,QFont.Bold)); lbl.setStyleSheet(f"color:{color};"); lbl.setFixedWidth(60); hl.addWidget(lbl)
    inputs={}
    for tag,val in [("Kp",kp),("Ki",ki),("Kd",kd)]:
        tl=QLabel(tag); tl.setFont(QFont("Courier New",9)); tl.setStyleSheet("color:#6b7fa3;")
        inp=QLineEdit(str(val)); inp.setFont(QFont("Courier New",11)); inp.setFixedWidth(76); inp.setAlignment(Qt.AlignCenter)
        inp.setStyleSheet("background:#0d1117;color:#c9d1d9;border:1px solid #30363d;border-radius:5px;padding:3px 5px;")
        hl.addWidget(tl); hl.addWidget(inp); inputs[tag]=inp
    hl.addStretch()
    return row, inputs["Kp"], inputs["Ki"], inputs["Kd"]

def sbtn(text,bg,hv,mh=40):
    b=QPushButton(text); b.setFont(QFont("Courier New",10,QFont.Bold)); b.setMinimumHeight(mh); b.setCursor(Qt.PointingHandCursor)
    b.setStyleSheet(f"QPushButton{{background:{bg};color:#f0f6fc;border:1px solid {hv};border-radius:7px;padding:5px 14px;}}QPushButton:hover{{background:{hv};}}QPushButton:disabled{{background:#161b22;color:#484f58;}}")
    return b

def abtn(sym, sz=52):
    l=QLabel(sym); l.setAlignment(Qt.AlignCenter); l.setFixedSize(sz,sz)
    l.setFont(QFont("Courier New",18,QFont.Bold)); l.setStyleSheet("color:#484f58;background:#161b22;border:1px solid #30363d;border-radius:8px;")
    return l

IDLE  = "color:#484f58;background:#161b22;border:1px solid #30363d;border-radius:8px;"
ACT   = "color:#f0f6fc;background:#1f6feb;border:1px solid #58a6ff;border-radius:8px;"
STOPC = "color:#f85149;background:#3d0c0c;border:1px solid #f85149;border-radius:8px;"

class PIDDashboard(QWidget):
    HIST=300
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 PID Robot  —  Flechas para mover | Espacio = STOP")
        self.setMinimumSize(1150,860)
        self.setStyleSheet("QWidget{background:#0d1117;color:#c9d1d9;}QGroupBox{background:#161b22;border:1px solid #30363d;border-radius:8px;margin-top:10px;padding:10px 8px 8px 8px;color:#8b949e;font-family:'Courier New';font-size:9pt;}QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;}QTabWidget::pane{border:1px solid #30363d;background:#161b22;border-radius:0 6px 6px 6px;}QTabBar::tab{background:#0d1117;color:#8b949e;padding:8px 18px;font-family:'Courier New';font-size:10pt;border:1px solid #30363d;border-bottom:none;border-radius:6px 6px 0 0;margin-right:2px;}QTabBar::tab:selected{background:#161b22;color:#f0f6fc;border-color:#58a6ff;}QCheckBox{font-family:'Courier New';font-size:10pt;color:#8b949e;padding:4px;}QCheckBox::indicator{width:16px;height:16px;border-radius:4px;border:1px solid #30363d;background:#0d1117;}QCheckBox::indicator:checked{background:#1f6feb;border-color:#58a6ff;}")
        self.rpm1=self.rpm2=self.tgt1=self.tgt2=self.err1=self.err2=self.out1=self.out2=0.0
        self.sync_active=False; self.current_move="stop"; self._kp=set()
        self.d_rpm1=deque(maxlen=self.HIST); self.d_rpm2=deque(maxlen=self.HIST)
        self.d_tgt1=deque(maxlen=self.HIST); self.d_tgt2=deque(maxlen=self.HIST)
        self.d_err1=deque(maxlen=self.HIST); self.d_err2=deque(maxlen=self.HIST)
        self._build_ui(); self._start_serial()
        self.timer=QTimer(); self.timer.timeout.connect(self._refresh); self.timer.start(150)
        self.setFocusPolicy(Qt.StrongFocus); self.setFocus()

    # ── teclado ──
    def keyPressEvent(self,e):
        if e.isAutoRepeat(): return
        k=e.key(); self._kp.add(k)
        if k==Qt.Key_Up:    self._move("forward")
        elif k==Qt.Key_Down:  self._move("backward")
        elif k==Qt.Key_Left:  self._move("left")
        elif k==Qt.Key_Right: self._move("right")
        elif k==Qt.Key_Space: self._move("stop")
    def keyReleaseEvent(self,e):
        if e.isAutoRepeat(): return
        k=e.key(); self._kp.discard(k)
        nav={Qt.Key_Up,Qt.Key_Down,Qt.Key_Left,Qt.Key_Right}
        if k in nav and not (self._kp & nav): self._move("stop")
    def focusOutEvent(self,e): self._move("stop"); super().focusOutEvent(e)

    def _move(self,mv):
        if mv==self.current_move and mv!="stop": return
        self.current_move=mv; m1,m2=MOVES[mv]
        self._send(f"m1={m1}"); time.sleep(0.02); self._send(f"m2={m2}")
        self._hi(mv); print(f">>> {mv.upper():12s}  M1={m1:+.0f}  M2={m2:+.0f}")

    # ── UI ──
    def _build_ui(self):
        root=QVBoxLayout(self); root.setSpacing(8); root.setContentsMargins(14,10,14,10)
        # header
        hdr=QHBoxLayout()
        title=QLabel("ESP32  PID ROBOT"); title.setFont(QFont("Courier New",16,QFont.Bold)); title.setStyleSheet("color:#58a6ff;letter-spacing:2px;")
        sub=QLabel("Flechas del teclado para mover  |  Espacio = STOP"); sub.setFont(QFont("Courier New",9)); sub.setStyleSheet("color:#484f58;")
        vt=QVBoxLayout(); vt.addWidget(title); vt.addWidget(sub); hdr.addLayout(vt); hdr.addStretch()
        cl=QLabel("SERIAL OK" if SERIAL_OK else "SIN CONEXION"); cl.setFont(QFont("Courier New",9,QFont.Bold))
        cl.setStyleSheet(f"color:{'#3fb950' if SERIAL_OK else '#f85149'};padding:4px 10px;border:1px solid #30363d;border-radius:5px;background:#161b22;")
        hdr.addWidget(cl); root.addLayout(hdr)
        # displays
        dr=QHBoxLayout(); dr.setSpacing(10)
        self.disp1=self._mkd("M1  DERECHO","#58a6ff"); self.disp2=self._mkd("M2  IZQUIERDO","#f78166")
        self.dsync=QLabel("SYNC\nOFF"); self.dsync.setAlignment(Qt.AlignCenter); self.dsync.setFont(QFont("Courier New",10,QFont.Bold))
        self.dsync.setStyleSheet("color:#484f58;background:#0d1117;border:1px solid #30363d;border-radius:10px;padding:10px;min-width:80px;"); self.dsync.setMinimumHeight(68)
        dr.addWidget(self.disp1,3); dr.addWidget(self.disp2,3); dr.addWidget(self.dsync,1); root.addLayout(dr)
        # centre
        ctr=QHBoxLayout(); ctr.setSpacing(10)
        lft=QVBoxLayout(); lft.setSpacing(8); lft.addWidget(self._ctrl_panel()); lft.addWidget(self._pid_tabs()); lft.addStretch()
        self._build_plot()
        ctr.addLayout(lft,40); ctr.addWidget(self.canvas,60); root.addLayout(ctr)

    def _ctrl_panel(self):
        box=QGroupBox("CONTROL DE MOVIMIENTO  —  Teclado"); vl=QVBoxLayout(box); vl.setSpacing(10)
        # pad
        pb=QGroupBox("Indicador visual"); pb.setStyleSheet("QGroupBox{background:#0d1117;border:1px solid #21262d;border-radius:6px;margin-top:8px;padding:6px;color:#484f58;font-family:'Courier New';font-size:8pt;}QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 3px;}")
        pl=QVBoxLayout(pb); pl.setSpacing(4)
        ru=QHBoxLayout(); ru.addStretch(); self.a_up=abtn("↑"); ru.addWidget(self.a_up); ru.addStretch()
        rm=QHBoxLayout(); rm.setSpacing(4); self.a_lf=abtn("←"); self.a_st=abtn("■",52); self.a_rt=abtn("→")
        rm.addStretch(); rm.addWidget(self.a_lf); rm.addWidget(self.a_st); rm.addWidget(self.a_rt); rm.addStretch()
        rd=QHBoxLayout(); rd.addStretch(); self.a_dn=abtn("↓"); rd.addWidget(self.a_dn); rd.addStretch()
        pl.addLayout(ru); pl.addLayout(rm); pl.addLayout(rd); vl.addWidget(pb)
        # estado
        self.mlbl=QLabel("ESTADO:  DETENIDO     M1=0  M2=0"); self.mlbl.setAlignment(Qt.AlignCenter)
        self.mlbl.setFont(QFont("Courier New",10,QFont.Bold)); self.mlbl.setStyleSheet("color:#484f58;background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:6px;"); vl.addWidget(self.mlbl)
        # rpm base
        rr=QHBoxLayout(); rl=QLabel("RPM base:"); rl.setFont(QFont("Courier New",10)); rl.setStyleSheet("color:#8b949e;")
        self.rpm_inp=QLineEdit("50"); self.rpm_inp.setFont(QFont("Courier New",11)); self.rpm_inp.setFixedWidth(70); self.rpm_inp.setAlignment(Qt.AlignCenter)
        self.rpm_inp.setStyleSheet("background:#0d1117;color:#c9d1d9;border:1px solid #30363d;border-radius:5px;padding:4px;")
        self.rpm_inp.editingFinished.connect(self._upd_base)
        rh=QLabel("(10–150)"); rh.setFont(QFont("Courier New",8)); rh.setStyleSheet("color:#484f58;")
        rr.addWidget(rl); rr.addWidget(self.rpm_inp); rr.addWidget(rh); rr.addStretch(); vl.addLayout(rr)
        # sync
        sb=QGroupBox("SINCRONIZACION LINEAL"); sb.setStyleSheet("QGroupBox{background:#0d1117;border:1px solid #1f6feb;border-radius:6px;margin-top:6px;padding:8px;color:#58a6ff;font-family:'Courier New';font-size:9pt;}QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;}")
        sl=QVBoxLayout(sb)
        kr=QHBoxLayout(); kl=QLabel("Kp sync:"); kl.setFont(QFont("Courier New",10)); kl.setStyleSheet("color:#8b949e;")
        self.kpsync=QLineEdit("0.5"); self.kpsync.setFont(QFont("Courier New",10)); self.kpsync.setFixedWidth(70); self.kpsync.setAlignment(Qt.AlignCenter)
        self.kpsync.setStyleSheet("background:#0d1117;color:#c9d1d9;border:1px solid #30363d;border-radius:5px;padding:3px;")
        kr.addWidget(kl); kr.addWidget(self.kpsync); kr.addStretch(); sl.addLayout(kr)
        self.scb=QCheckBox("Activar sync en avance / retroceso"); self.scb.setStyleSheet("color:#58a6ff;font-family:'Courier New';font-size:9pt;")
        self.scb.stateChanged.connect(self._tog_sync); sl.addWidget(self.scb); vl.addWidget(sb)
        # stop btn
        bs=sbtn("STOP EMERGENCIA  (Espacio)","#3d0c0c","#f85149",44); bs.clicked.connect(lambda:self._move("stop")); vl.addWidget(bs)
        return box

    def _pid_tabs(self):
        tabs=QTabWidget()
        tf=QWidget(); tf.setStyleSheet("background:#161b22;"); vf=QVBoxLayout(tf); vf.setSpacing(6)
        inf=QLabel("Activo cuando setpoint > 0  (avance)"); inf.setFont(QFont("Courier New",8)); inf.setStyleSheet("color:#3fb950;padding:2px 4px;"); vf.addWidget(inf)
        r1f,self.kp1f,self.ki1f,self.kd1f=pid_row("M1 DER","#58a6ff",1.35,0.500,0.038)
        r2f,self.kp2f,self.ki2f,self.kd2f=pid_row("M2 IZQ","#f78166",1.35,0.545,0.028)
        vf.addWidget(r1f); vf.addWidget(r2f)
        bf=sbtn("Aplicar PID Avance","#0e2a47","#1f6feb",34); bf.clicked.connect(self._spf); vf.addWidget(bf); vf.addStretch()
        tr=QWidget(); tr.setStyleSheet("background:#161b22;"); vr=QVBoxLayout(tr); vr.setSpacing(6)
        inr=QLabel("Activo cuando setpoint < 0  (retroceso)"); inr.setFont(QFont("Courier New",8)); inr.setStyleSheet("color:#f85149;padding:2px 4px;"); vr.addWidget(inr)
        r1r,self.kp1r,self.ki1r,self.kd1r=pid_row("M1 DER","#ff7b72",1.35,0.500,0.038)
        r2r,self.kp2r,self.ki2r,self.kd2r=pid_row("M2 IZQ","#ffa657",1.35,0.545,0.028)
        vr.addWidget(r1r); vr.addWidget(r2r)
        br=sbtn("Aplicar PID Retroceso","#3d0c0c","#da3633",34); br.clicked.connect(self._spr); vr.addWidget(br); vr.addStretch()
        tabs.addTab(tf,"AVANCE"); tabs.addTab(tr,"RETROCESO"); return tabs

    def _build_plot(self):
        self.figure=Figure(facecolor="#0d1117"); self.canvas=FigureCanvas(self.figure)
        self.canvas.setStyleSheet("background:#0d1117;border-radius:8px;"); self.canvas.setFocusPolicy(Qt.NoFocus)
        gs=gridspec.GridSpec(2,1,figure=self.figure,hspace=0.48,top=0.93,bottom=0.08,left=0.09,right=0.97)
        self.ax_rpm=self.figure.add_subplot(gs[0]); self.ax_err=self.figure.add_subplot(gs[1])
        for ax in (self.ax_rpm,self.ax_err):
            ax.set_facecolor("#161b22"); ax.tick_params(colors="#484f58",labelsize=8)
            ax.grid(True,alpha=0.15,color="#30363d"); ax.axhline(0,color="#30363d",linewidth=0.8)
            for sp in ax.spines.values(): sp.set_edgecolor("#30363d")
        self.ax_rpm.set_title("RPM vs Target",color="#8b949e",fontsize=9,pad=4)
        self.ax_err.set_title("Error PID",color="#8b949e",fontsize=9,pad=4)

    # ── serial ──
    def _start_serial(self):
        if SERIAL_OK: threading.Thread(target=self._rserial,daemon=True).start()
    def _rserial(self):
        while True:
            try:
                raw=ser.readline().decode(errors="ignore").strip()
                if not raw: continue
                if "RPM1:" in raw and "Target1:" in raw:
                    try:
                        p={}
                        for t in raw.split("\t"):
                            if ":" in t: k,v=t.split(":",1); p[k.strip()]=v.strip()
                        self.tgt1=float(p.get("Target1",0)); self.rpm1=float(p.get("RPM1",0))
                        self.err1=float(p.get("Err1",0));   self.out1=float(p.get("Out1",0))
                        self.tgt2=float(p.get("Target2",0)); self.rpm2=float(p.get("RPM2",0))
                        self.err2=float(p.get("Err2",0));   self.out2=float(p.get("Out2",0))
                        self.d_rpm1.append(self.rpm1); self.d_rpm2.append(self.rpm2)
                        self.d_tgt1.append(self.tgt1); self.d_tgt2.append(self.tgt2)
                        self.d_err1.append(self.err1); self.d_err2.append(self.err2)
                    except Exception as e: print(f"Parse: {e}")
                elif raw and "CMD_RECIBIDO" not in raw: print(f"ESP32: {raw}")
            except Exception: pass
    def _send(self,cmd):
        if not SERIAL_OK: print(f"[SIM] {cmd}"); return
        try: ser.write((cmd+"\n").encode())
        except Exception as e: print(f"Serial: {e}")

    # ── acciones ──
    def _upd_base(self):
        global BASE_RPM, MOVES
        try:
            v=max(10.,min(150.,abs(float(self.rpm_inp.text())))); BASE_RPM=v
            MOVES={"forward":(+v,+v),"backward":(-v,-v),"left":(+v,-v),"right":(-v,+v),"stop":(0.,0.)}
            self.rpm_inp.setText(str(int(v))); print(f"RPM base: {v:.0f}")
        except ValueError: pass
    def _tog_sync(self,s):
        self.sync_active=bool(s)
        if s: self._send(f"kpsync={self.kpsync.text()}"); self._send("sync=1"); print("Sync ON")
        else: self._send("sync=0"); print("Sync OFF")
    def _spf(self):
        try:
            for c in [f"kp1f={float(self.kp1f.text())}",f"ki1f={float(self.ki1f.text())}",f"kd1f={float(self.kd1f.text())}",
                      f"kp2f={float(self.kp2f.text())}",f"ki2f={float(self.ki2f.text())}",f"kd2f={float(self.kd2f.text())}"]:
                self._send(c); time.sleep(0.03)
            print("PID Avance OK")
        except ValueError: print("PID invalido")
    def _spr(self):
        try:
            for c in [f"kp1r={float(self.kp1r.text())}",f"ki1r={float(self.ki1r.text())}",f"kd1r={float(self.kd1r.text())}",
                      f"kp2r={float(self.kp2r.text())}",f"ki2r={float(self.ki2r.text())}",f"kd2r={float(self.kd2r.text())}"]:
                self._send(c); time.sleep(0.03)
            print("PID Retroceso OK")
        except ValueError: print("PID invalido")

    # ── refresh ──
    def _refresh(self): self._upd_disp(); self._upd_plot()
    def _hi(self,mv):
        self.a_up.setStyleSheet(ACT  if mv=="forward"  else IDLE)
        self.a_dn.setStyleSheet(ACT  if mv=="backward" else IDLE)
        self.a_lf.setStyleSheet(ACT  if mv=="left"     else IDLE)
        self.a_rt.setStyleSheet(ACT  if mv=="right"    else IDLE)
        self.a_st.setStyleSheet(STOPC if mv=="stop"    else IDLE)
        txt={"forward":"AVANZANDO #3fb950","backward":"RETROCEDIENDO #f85149","left":"GIRANDO IZQ #d29922","right":"GIRANDO DER #d29922","stop":"DETENIDO #484f58"}
        parts=txt[mv].rsplit(" ",1); label=parts[0]; col=parts[1]
        m1,m2=MOVES[mv]; self.mlbl.setText(f"{label}     M1={m1:+.0f}  M2={m2:+.0f}")
        self.mlbl.setStyleSheet(f"color:{col};background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:6px;font-family:'Courier New';font-size:10pt;font-weight:bold;")
    def _upd_disp(self):
        d1="FWD" if self.rpm1>=0 else "REV"; d2="FWD" if self.rpm2>=0 else "REV"
        c1="#58a6ff" if self.rpm1>=0 else "#ff7b72"; c2="#f78166" if self.rpm2>=0 else "#ffa657"
        self.disp1.setText(f"M1 DERECHO [{d1}]\n{self.rpm1:+.1f} RPM")
        self.disp1.setStyleSheet(f"color:{c1};background:#0d1117;border:1px solid #30363d;border-radius:10px;padding:10px;")
        self.disp2.setText(f"M2 IZQUIERDO [{d2}]\n{self.rpm2:+.1f} RPM")
        self.disp2.setStyleSheet(f"color:{c2};background:#0d1117;border:1px solid #30363d;border-radius:10px;padding:10px;")
        if self.sync_active:
            diff=abs(self.rpm1)-abs(self.rpm2); sc="#3fb950" if abs(diff)<5 else "#d29922"
            self.dsync.setText(f"SYNC\nON\nD{diff:+.1f}")
            self.dsync.setStyleSheet(f"color:{sc};background:#0d1117;border:1px solid {sc};border-radius:10px;padding:10px;min-width:80px;")
        else:
            self.dsync.setText("SYNC\nOFF"); self.dsync.setStyleSheet("color:#484f58;background:#0d1117;border:1px solid #30363d;border-radius:10px;padding:10px;min-width:80px;")
    def _upd_plot(self):
        if not self.d_rpm1: return
        r1=list(self.d_rpm1); r2=list(self.d_rpm2); t1=list(self.d_tgt1); t2=list(self.d_tgt2)
        e1=list(self.d_err1); e2=list(self.d_err2); xs=range(len(r1))
        self.ax_rpm.clear(); self.ax_rpm.set_facecolor("#161b22"); self.ax_rpm.grid(True,alpha=0.15,color="#30363d"); self.ax_rpm.axhline(0,color="#30363d",lw=0.8)
        self.ax_rpm.plot(xs,t1,color="#1f6feb",lw=1.2,ls="--",alpha=0.7,label="Tgt M1"); self.ax_rpm.plot(xs,t2,color="#da3633",lw=1.2,ls="--",alpha=0.7,label="Tgt M2")
        self.ax_rpm.plot(xs,r1,color="#58a6ff",lw=2.2,label="M1 DER"); self.ax_rpm.plot(xs,r2,color="#f78166",lw=2.2,label="M2 IZQ")
        self.ax_rpm.set_title("RPM vs Target",color="#8b949e",fontsize=9,pad=4); self.ax_rpm.set_ylabel("RPM",color="#8b949e",fontsize=8); self.ax_rpm.tick_params(colors="#484f58",labelsize=7)
        for sp in self.ax_rpm.spines.values(): sp.set_edgecolor("#30363d")
        self.ax_rpm.legend(loc="upper left",fontsize=7,framealpha=0.7,facecolor="#161b22",edgecolor="#30363d",labelcolor="#c9d1d9")
        self.ax_err.clear(); self.ax_err.set_facecolor("#161b22"); self.ax_err.grid(True,alpha=0.15,color="#30363d"); self.ax_err.axhline(0,color="#3fb950",lw=0.8,alpha=0.5)
        self.ax_err.fill_between(xs,e1,alpha=0.15,color="#58a6ff"); self.ax_err.fill_between(xs,e2,alpha=0.15,color="#f78166")
        self.ax_err.plot(xs,e1,color="#58a6ff",lw=1.8,label="Err M1"); self.ax_err.plot(xs,e2,color="#f78166",lw=1.8,label="Err M2")
        self.ax_err.set_title("Error PID",color="#8b949e",fontsize=9,pad=4); self.ax_err.set_ylabel("error",color="#8b949e",fontsize=8); self.ax_err.set_xlabel("muestras",color="#8b949e",fontsize=8); self.ax_err.tick_params(colors="#484f58",labelsize=7)
        for sp in self.ax_err.spines.values(): sp.set_edgecolor("#30363d")
        self.ax_err.legend(loc="upper left",fontsize=7,framealpha=0.7,facecolor="#161b22",edgecolor="#30363d",labelcolor="#c9d1d9")
        self.figure.canvas.draw_idle()
    def _mkd(self,name,color):
        l=QLabel(f"{name}\n0.00 RPM"); l.setAlignment(Qt.AlignCenter); l.setFont(QFont("Courier New",12,QFont.Bold))
        l.setStyleSheet(f"color:{color};background:#0d1117;border:1px solid #30363d;border-radius:10px;padding:10px;"); l.setMinimumHeight(68); return l

if __name__=="__main__":
    app=QApplication(sys.argv); app.setStyle("Fusion"); w=PIDDashboard(); w.show(); sys.exit(app.exec_())