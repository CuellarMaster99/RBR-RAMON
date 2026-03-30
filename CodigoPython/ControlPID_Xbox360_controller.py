"""
ESP32 PID Robot — Control Xbox 360 via Bluetooth
==================================================
Fix: el D-Pad ahora mantiene el movimiento mientras
     el boton esta presionado (reenvio periodico).

Pasos para conectar:
  1. Sube motor_pid_bt.ino al ESP32 y alimentalo
  2. Windows: Configuracion > Bluetooth > Agregar dispositivo
     Busca "ESP32_ROBOT" y empareja (sin PIN o PIN: 1234)
  3. Panel de control > Dispositivos Bluetooth > COM Ports
     Anota el puerto COM "Outgoing"
  4. Cambia BT_PORT abajo con ese numero
  5. Corre este script

Indices de tu mando (confirmados):
  Eje 0 = Joystick IZQ X
  Eje 4 = LT
  Eje 5 = RT
  Hat 0 = D-Pad
"""

import pygame, serial, time, sys, threading

# ═══════════════════════════════════════════════
#  CONFIGURACION
# ═══════════════════════════════════════════════
BT_PORT   = "COM3"      # <-- cambia este numero
BAUD      = 115200
MAX_RPM   = 150.0
DPAD_RPM  = 100.0
DEAD_JOY  = 0.08
DEAD_TRIG = 0.03

AX_JOY_IZQ_X = 0
AX_LT        = 4
AX_RT        = 5

# Cada cuantos ciclos se reenvian los setpoints al ESP32
# 50 Hz * 0.1s = cada 5 ciclos → reenvio cada 100ms
RESEND_EVERY = 5

# ═══════════════════════════════════════════════
#  CONEXION BLUETOOTH
# ═══════════════════════════════════════════════
def connect_bt(port, baud, retries=5):
    for i in range(retries):
        try:
            s = serial.Serial(port, baud, timeout=1)
            time.sleep(1.5)
            print(f"Bluetooth OK -> {port}")
            return s
        except serial.SerialException as e:
            print(f"Intento {i+1}/{retries}: {e}")
            time.sleep(2)
    return None

print(f"Conectando a ESP32_ROBOT en {BT_PORT}...")
ser = connect_bt(BT_PORT, BAUD)
SERIAL_OK = ser is not None
if not SERIAL_OK:
    print("No se pudo conectar. Continuando en modo simulacion...\n")

# ═══════════════════════════════════════════════
#  LECTURA EN TIEMPO REAL (hilo separado)
# ═══════════════════════════════════════════════
rpm1_live = 0.0; rpm2_live = 0.0
tgt1_live = 0.0; tgt2_live = 0.0

def reader_thread():
    global rpm1_live, rpm2_live, tgt1_live, tgt2_live
    while True:
        try:
            if not SERIAL_OK: time.sleep(0.5); continue
            raw = ser.readline().decode(errors="ignore").strip()
            if "RPM1:" in raw and "Target1:" in raw:
                parts = {}
                for token in raw.split("\t"):
                    if ":" in token:
                        k, v = token.split(":", 1)
                        parts[k.strip()] = v.strip()
                rpm1_live = float(parts.get("RPM1",    0))
                rpm2_live = float(parts.get("RPM2",    0))
                tgt1_live = float(parts.get("Target1", 0))
                tgt2_live = float(parts.get("Target2", 0))
        except Exception:
            pass

if SERIAL_OK:
    threading.Thread(target=reader_thread, daemon=True).start()

# ═══════════════════════════════════════════════
#  SERIAL HELPERS
#  set_motors() ahora tiene dos modos:
#    force=False  solo envia si cambio >= 1 RPM (gatillos)
#    force=True   envia siempre              (D-Pad, reenvio)
# ═══════════════════════════════════════════════
_last_m1 = None
_last_m2 = None

def send(cmd):
    if not SERIAL_OK:
        print(f"[SIM] {cmd}"); return
    try:
        ser.write((cmd + "\n").encode())
    except Exception as e:
        print(f"BT error: {e}")

def set_motors(m1: float, m2: float, force: bool = False):
    global _last_m1, _last_m2
    m1 = round(max(-MAX_RPM, min(MAX_RPM, m1)), 1)
    m2 = round(max(-MAX_RPM, min(MAX_RPM, m2)), 1)
    if force or _last_m1 is None or abs(m1 - _last_m1) >= 1.0:
        send(f"m1={m1}"); _last_m1 = m1
    if force or _last_m2 is None or abs(m2 - _last_m2) >= 1.0:
        send(f"m2={m2}"); _last_m2 = m2

# ═══════════════════════════════════════════════
#  LOGICA DE CONTROL
# ═══════════════════════════════════════════════
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

# ═══════════════════════════════════════════════
#  BUCLE PRINCIPAL
# ═══════════════════════════════════════════════
def main():
    pygame.init()
    pygame.joystick.init()

    screen = pygame.display.set_mode((640, 380))
    pygame.display.set_caption(f"ESP32 Robot BT — {BT_PORT}")

    if pygame.joystick.get_count() == 0:
        print("No se detecto ningun control Xbox.")
        pygame.quit(); sys.exit()

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Control: {joy.get_name()}")

    font_big = pygame.font.SysFont("consolas", 17, bold=True)
    font_med = pygame.font.SysFont("consolas", 14)
    font_sml = pygame.font.SysFont("consolas", 12)

    clock    = pygame.time.Clock()
    m1_disp  = 0.0
    m2_disp  = 0.0
    mode_str   = "DETENIDO"
    mode_color = (100, 100, 100)

    cycle      = 0    # contador de ciclos para reenvio periodico
    ping_timer = 0

    while True:
        clock.tick(50)   # 50 Hz
        cycle += 1
        ping_timer += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                send("stop"); pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                send("stop"); pygame.quit(); sys.exit()

        pygame.event.pump()

        # Ping cada 5 s para mantener el BT activo
        if ping_timer >= 250:
            send("ping"); ping_timer = 0

        # ── Leer estado del mando (polling, no eventos) ──
        rt_rpm = read_trigger(joy, AX_RT)
        lt_rpm = read_trigger(joy, AX_LT)
        joy_x  = joy.get_axis(AX_JOY_IZQ_X)
        dpad   = joy.get_hat(0)   # (x,y) — siempre refleja el estado actual

        base_rpm     = 0.0
        use_steering = True

        # ── Forzar reenvio periodico para D-Pad ──
        # Los gatillos cambian valor continuamente (analogico) → el filtro
        # de 1 RPM los reenvía naturalmente. El D-Pad es digital y fijo,
        # así que hay que forzar el reenvio cada RESEND_EVERY ciclos.
        force_send = (cycle % RESEND_EVERY == 0)

        # ── Prioridad: Gatillos > D-Pad > Stop ──
        if rt_rpm > 0.5 and lt_rpm > 0.5:
            base_rpm = rt_rpm if rt_rpm >= lt_rpm else -lt_rpm
            mode_str   = f"{'RT AVANCE' if base_rpm>0 else 'LT RETROCESO'}  {base_rpm:+.0f} RPM"
            mode_color = (80,220,80) if base_rpm>0 else (240,100,100)

        elif rt_rpm > 0.5:
            base_rpm   = rt_rpm
            mode_str   = f"RT  {rt_rpm:+.0f} RPM"
            mode_color = (80, 220, 80)

        elif lt_rpm > 0.5:
            base_rpm   = -lt_rpm
            mode_str   = f"LT  {-lt_rpm:+.0f} RPM"
            mode_color = (240, 100, 100)

        elif dpad[1] == 1:      # D-Pad arriba — avance continuo
            base_rpm   = DPAD_RPM
            mode_str   = f"D-PAD UP  +{DPAD_RPM:.0f} RPM"
            mode_color = (80, 180, 255)

        elif dpad[1] == -1:     # D-Pad abajo — retroceso continuo
            base_rpm   = -DPAD_RPM
            mode_str   = f"D-PAD DOWN  -{DPAD_RPM:.0f} RPM"
            mode_color = (255, 160, 50)

        elif dpad[0] == -1:     # D-Pad izquierda — giro continuo
            m1_out, m2_out = +DPAD_RPM, -DPAD_RPM
            set_motors(m1_out, m2_out, force=force_send)
            m1_disp, m2_disp = m1_out, m2_out
            use_steering = False
            mode_str   = f"D-PAD IZQ  M1={m1_out:+.0f} M2={m2_out:+.0f}"
            mode_color = (200, 160, 255)

        elif dpad[0] == 1:      # D-Pad derecha — giro continuo
            m1_out, m2_out = -DPAD_RPM, +DPAD_RPM
            set_motors(m1_out, m2_out, force=force_send)
            m1_disp, m2_disp = m1_out, m2_out
            use_steering = False
            mode_str   = f"D-PAD DER  M1={m1_out:+.0f} M2={m2_out:+.0f}"
            mode_color = (200, 160, 255)

        else:
            # Ninguna entrada activa → stop (solo si acaba de cambiar)
            if _last_m1 != 0.0 or _last_m2 != 0.0:
                set_motors(0.0, 0.0, force=True)
            base_rpm   = 0.0
            mode_str   = "DETENIDO"
            mode_color = (100, 100, 100)

        if use_steering:
            m1_out, m2_out = apply_steering(base_rpm, joy_x)
            set_motors(m1_out, m2_out, force=force_send)
            m1_disp, m2_disp = m1_out, m2_out

        # ════════════════════════════════════════
        #  PANTALLA
        # ════════════════════════════════════════
        screen.fill((13, 17, 23))

        bt_col = (80,220,80) if SERIAL_OK else (200,80,80)
        bt_txt = f"BT {BT_PORT}  CONECTADO" if SERIAL_OK else f"BT {BT_PORT}  SIMULACION"
        screen.blit(font_big.render("ESP32 ROBOT  —  Xbox 360  BT", True, (88,166,255)), (18,12))
        screen.blit(font_sml.render(bt_txt, True, bt_col), (18, 36))
        screen.blit(font_big.render(mode_str, True, mode_color), (18, 54))

        # Barras motores (setpoint + RPM real superpuesta)
        def motor_bar(setpt, rpm_real, y, label):
            pygame.draw.rect(screen, (28,34,44), (18,y,200,20), border_radius=4)
            w = int(abs(setpt)/MAX_RPM*200)
            col_bg = (40,120,40) if setpt>=0 else (120,40,40)
            if w>0: pygame.draw.rect(screen, col_bg, (18,y,min(w,200),20), border_radius=4)
            w2 = int(abs(rpm_real)/MAX_RPM*200)
            col_fg = (70,210,70) if rpm_real>=0 else (230,90,90)
            if w2>0: pygame.draw.rect(screen, col_fg, (18,y,min(w2,200),10), border_radius=4)
            screen.blit(font_med.render(
                f"{label}  cmd={setpt:+5.0f}  real={rpm_real:+5.1f}",
                True,(190,190,190)),(228,y+2))

        motor_bar(m1_disp, rpm1_live, 88,  "M1 DER")
        motor_bar(m2_disp, rpm2_live, 115, "M2 IZQ")

        # Gatillos
        def trig_bar(rpm, y, label, col):
            pygame.draw.rect(screen, (28,34,44), (18,y,130,12), border_radius=3)
            w = int(rpm/MAX_RPM*130)
            if w>0: pygame.draw.rect(screen, col, (18,y,min(w,130),12), border_radius=3)
            screen.blit(font_sml.render(f"{label} {rpm:.0f} RPM", True,(130,130,130)),(158,y))

        trig_bar(rt_rpm, 150, "RT", (70,210,70))
        trig_bar(lt_rpm, 168, "LT", (230,90,90))

        # Joystick IZQ
        jx = dead(joy_x, DEAD_JOY)
        cx, cy = 110, 238
        pygame.draw.circle(screen, (28,34,44), (cx,cy), 46)
        pygame.draw.circle(screen, (55,65,76), (cx,cy), 46, 1)
        pygame.draw.line(screen, (50,60,70), (cx-46,cy),(cx+46,cy), 1)
        dot_x = cx + int(jx*44)
        pygame.draw.circle(screen, (88,166,255) if abs(jx)>0.05 else (55,65,76), (dot_x,cy), 10)
        screen.blit(font_sml.render(f"Joy X {joy_x:+.3f}", True,(80,90,100)),(70,290))

        # D-Pad
        dpx, dpy = 310, 232
        dc=(88,166,255); di=(32,40,50)
        pygame.draw.rect(screen, dc if dpad[1]== 1 else di, (dpx-12,dpy-44,24,34), border_radius=4)
        pygame.draw.rect(screen, dc if dpad[1]==-1 else di, (dpx-12,dpy+10,24,34), border_radius=4)
        pygame.draw.rect(screen, dc if dpad[0]==-1 else di, (dpx-54,dpy-12,36,24), border_radius=4)
        pygame.draw.rect(screen, dc if dpad[0]== 1 else di, (dpx+18,dpy-12,36,24), border_radius=4)
        pygame.draw.rect(screen, (40,50,60), (dpx-12,dpy-10,24,20), border_radius=3)

        # RPM real (lado derecho)
        screen.blit(font_sml.render("RPM en tiempo real:", True,(70,80,90)),(440,190))
        def mini_bar(val, y, label):
            pygame.draw.rect(screen, (28,34,44), (440,y,140,14), border_radius=3)
            w = int(abs(val)/MAX_RPM*140)
            col = (70,210,70) if val>=0 else (230,90,90)
            if w>0: pygame.draw.rect(screen, col, (440,y,min(w,140),14), border_radius=3)
            screen.blit(font_sml.render(f"{label} {val:+.0f}", True,(150,150,150)),(590,y))
        mini_bar(rpm1_live, 210, "M1")
        mini_bar(rpm2_live, 230, "M2")

        # Leyenda
        hints = [
            "RT=avance proporcional  |  LT=retroceso proporcional",
            "D-Pad UD=+-50RPM fijo   |  D-Pad LR=giro en sitio",
            "Joy IZQ X=direccion     |  ESC=stop y salir",
        ]
        for i,h in enumerate(hints):
            screen.blit(font_sml.render(h, True,(55,65,75)),(18,318+i*17))

        pygame.display.flip()

if __name__ == "__main__":
    main()