"""
ESP32 PID Robot — Control Xbox 360 via WiFi UDP
=================================================
Pasos para conectar:
  1. Sube motor_pid_wifi.ino al ESP32
  2. Cambia WIFI_SSID y WIFI_PASS en el .ino con los datos
     de tu hotspot y vuelve a subirlo
  3. Activa el hotspot en tu telefono
  4. Conecta el PC al mismo hotspot
  5. Enciende el robot (sin cable USB)
  6. Mira el Serial Monitor — anota la IP que muestra:
        IP del ESP32: 192.168.x.x
  7. Pon esa IP en ESP32_IP abajo
  8. Corre este script

Indices confirmados de tu mando:
  Eje 0 = Joystick IZQ X
  Eje 4 = LT
  Eje 5 = RT
  Hat 0 = D-Pad
"""

import pygame
import socket
import time
import sys
import threading

# ═══════════════════════════════════════════════
#  CONFIGURACION — ajusta ESP32_IP
# ═══════════════════════════════════════════════
ESP32_IP     = "192.168.211.147"   # <-- IP del ESP32 (ver Serial Monitor)
UDP_PORT_TX  = 4210            # puerto donde el ESP32 escucha comandos
UDP_PORT_RX  = 4211           # puerto donde el ESP32 envia telemetria
MAX_RPM      = 150.0
DPAD_RPM     = 50.0
DEAD_JOY     = 0.08
DEAD_TRIG    = 0.03
RESEND_EVERY = 5               # reenviar D-Pad cada 5 ciclos (100ms)

AX_JOY_IZQ_X = 0
AX_LT        = 4
AX_RT        = 5

# ═══════════════════════════════════════════════
#  SOCKET UDP
# ═══════════════════════════════════════════════
sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_tx.settimeout(0.01)

sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx.bind(("", UDP_PORT_RX))
sock_rx.settimeout(0.01)

WIFI_OK = ESP32_IP != "192.168.211.147"

def send(cmd):
    if not WIFI_OK:
        print(f"[SIM] {cmd}"); return
    try:
        sock_tx.sendto(cmd.encode(), (ESP32_IP, UDP_PORT_TX))
    except Exception as e:
        print(f"UDP error: {e}")

# ═══════════════════════════════════════════════
#  TELEMETRIA EN TIEMPO REAL (hilo separado)
# ═══════════════════════════════════════════════
rpm1_live = 0.0; rpm2_live = 0.0
tgt1_live = 0.0; tgt2_live = 0.0
wifi_latency = 0

def receiver_thread():
    global rpm1_live, rpm2_live, tgt1_live, tgt2_live
    while True:
        try:
            data, _ = sock_rx.recvfrom(512)
            raw = data.decode(errors="ignore").strip()
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

threading.Thread(target=receiver_thread, daemon=True).start()

# ═══════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════
_last_m1 = None
_last_m2 = None

def set_motors(m1: float, m2: float, force: bool = False):
    global _last_m1, _last_m2
    m1 = round(max(-MAX_RPM, min(MAX_RPM, m1)), 1)
    m2 = round(max(-MAX_RPM, min(MAX_RPM, m2)), 1)
    if force or _last_m1 is None or abs(m1 - _last_m1) >= 1.0:
        send(f"m1={m1}"); _last_m1 = m1
    if force or _last_m2 is None or abs(m2 - _last_m2) >= 1.0:
        send(f"m2={m2}"); _last_m2 = m2

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
    pygame.display.set_caption(f"ESP32 Robot WiFi — {ESP32_IP}")

    if pygame.joystick.get_count() == 0:
        print("No se detecto ningun control Xbox.")
        pygame.quit(); sys.exit()

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Control: {joy.get_name()}")
    print(f"ESP32 IP: {ESP32_IP}")
    if not WIFI_OK:
        print("AVISO: Cambia ESP32_IP con la IP real del ESP32")

    font_big = pygame.font.SysFont("consolas", 17, bold=True)
    font_med = pygame.font.SysFont("consolas", 14)
    font_sml = pygame.font.SysFont("consolas", 12)

    clock      = pygame.time.Clock()
    m1_disp    = 0.0; m2_disp = 0.0
    mode_str   = "DETENIDO"
    mode_color = (100, 100, 100)
    cycle      = 0
    ping_timer = 0

    while True:
        clock.tick(50)
        cycle += 1
        ping_timer += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                send("stop"); pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                send("stop"); pygame.quit(); sys.exit()

        pygame.event.pump()

        # Ping cada 2 segundos para resetear el watchdog del ESP32
        if ping_timer >= 100:
            send("ping"); ping_timer = 0

        # Leer mando
        rt_rpm = read_trigger(joy, AX_RT)
        lt_rpm = read_trigger(joy, AX_LT)
        joy_x  = joy.get_axis(AX_JOY_IZQ_X)
        dpad   = joy.get_hat(0)

        base_rpm     = 0.0
        use_steering = True
        force_send   = (cycle % RESEND_EVERY == 0)

        # Prioridad: Gatillos > D-Pad > Stop
        if rt_rpm > 0.5 and lt_rpm > 0.5:
            base_rpm   = rt_rpm if rt_rpm >= lt_rpm else -lt_rpm
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
        elif dpad[1] == 1:
            base_rpm   = DPAD_RPM
            mode_str   = f"D-PAD UP  +{DPAD_RPM:.0f} RPM"
            mode_color = (80, 180, 255)
        elif dpad[1] == -1:
            base_rpm   = -DPAD_RPM
            mode_str   = f"D-PAD DOWN  -{DPAD_RPM:.0f} RPM"
            mode_color = (255, 160, 50)
        elif dpad[0] == -1:
            m1_out, m2_out = +DPAD_RPM, -DPAD_RPM
            set_motors(m1_out, m2_out, force=force_send)
            m1_disp, m2_disp = m1_out, m2_out
            use_steering = False
            mode_str   = f"D-PAD IZQ  M1={m1_out:+.0f} M2={m2_out:+.0f}"
            mode_color = (200, 160, 255)
        elif dpad[0] == 1:
            m1_out, m2_out = -DPAD_RPM, +DPAD_RPM
            set_motors(m1_out, m2_out, force=force_send)
            m1_disp, m2_disp = m1_out, m2_out
            use_steering = False
            mode_str   = f"D-PAD DER  M1={m1_out:+.0f} M2={m2_out:+.0f}"
            mode_color = (200, 160, 255)
        else:
            if _last_m1 != 0.0 or _last_m2 != 0.0:
                set_motors(0.0, 0.0, force=True)
            base_rpm   = 0.0
            mode_str   = "DETENIDO"
            mode_color = (100, 100, 100)

        if use_steering:
            m1_out, m2_out = apply_steering(base_rpm, joy_x)
            set_motors(m1_out, m2_out, force=force_send)
            m1_disp, m2_disp = m1_out, m2_out

        # ── Pantalla ──
        screen.fill((13, 17, 23))

        wifi_col = (80,220,80) if WIFI_OK else (200,80,80)
        wifi_txt = f"WiFi  {ESP32_IP}" if WIFI_OK else "WiFi  configura ESP32_IP"
        screen.blit(font_big.render("ESP32 ROBOT  —  Xbox 360  WiFi", True, (88,166,255)), (18,12))
        screen.blit(font_sml.render(wifi_txt, True, wifi_col), (18, 36))
        screen.blit(font_big.render(mode_str, True, mode_color), (18, 54))

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

        def trig_bar(rpm, y, label, col):
            pygame.draw.rect(screen, (28,34,44), (18,y,130,12), border_radius=3)
            w = int(rpm/MAX_RPM*130)
            if w>0: pygame.draw.rect(screen, col, (18,y,min(w,130),12), border_radius=3)
            screen.blit(font_sml.render(f"{label} {rpm:.0f} RPM", True,(130,130,130)),(158,y))

        trig_bar(rt_rpm, 150, "RT", (70,210,70))
        trig_bar(lt_rpm, 168, "LT", (230,90,90))

        jx = dead(joy_x, DEAD_JOY)
        cx, cy = 110, 238
        pygame.draw.circle(screen, (28,34,44), (cx,cy), 46)
        pygame.draw.circle(screen, (55,65,76), (cx,cy), 46, 1)
        pygame.draw.line(screen, (50,60,70), (cx-46,cy),(cx+46,cy), 1)
        pygame.draw.circle(screen, (88,166,255) if abs(jx)>0.05 else (55,65,76),
                           (cx+int(jx*44), cy), 10)
        screen.blit(font_sml.render(f"Joy X {joy_x:+.3f}", True,(80,90,100)),(70,290))

        dpx, dpy = 310, 232
        dc=(88,166,255); di=(32,40,50)
        pygame.draw.rect(screen, dc if dpad[1]== 1 else di, (dpx-12,dpy-44,24,34), border_radius=4)
        pygame.draw.rect(screen, dc if dpad[1]==-1 else di, (dpx-12,dpy+10,24,34), border_radius=4)
        pygame.draw.rect(screen, dc if dpad[0]==-1 else di, (dpx-54,dpy-12,36,24), border_radius=4)
        pygame.draw.rect(screen, dc if dpad[0]== 1 else di, (dpx+18,dpy-12,36,24), border_radius=4)
        pygame.draw.rect(screen, (40,50,60), (dpx-12,dpy-10,24,20), border_radius=3)

        screen.blit(font_sml.render("RPM en tiempo real:", True,(70,80,90)),(440,190))
        def mini_bar(val, y, label):
            pygame.draw.rect(screen, (28,34,44), (440,y,140,14), border_radius=3)
            w = int(abs(val)/MAX_RPM*140)
            col = (70,210,70) if val>=0 else (230,90,90)
            if w>0: pygame.draw.rect(screen, col, (440,y,min(w,140),14), border_radius=3)
            screen.blit(font_sml.render(f"{label} {val:+.0f}", True,(150,150,150)),(590,y))
        mini_bar(rpm1_live, 210, "M1")
        mini_bar(rpm2_live, 230, "M2")

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