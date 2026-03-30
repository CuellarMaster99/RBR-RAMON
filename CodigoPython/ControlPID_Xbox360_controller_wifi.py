"""
ESP32 PID Robot — Control Xbox 360 via WiFi WebSocket
=======================================================
Protocolo JSON bidireccional. Sin cable USB, sin Bluetooth.

PASOS:
  1. Flashear esp32_pid_wifi.ino al ESP32
  2. Ver la IP en el Serial Monitor del Arduino IDE
  3. Cambiar ESP32_IP abajo con esa IP
  4. Instalar dependencias:
       pip install pygame websocket-client
  5. Ejecutar este script

FORMATO JSON ENVIADO AL ESP32:
  Setpoint:   {"cmd":"set",  "m1":80, "m2":80}
  Stop:       {"cmd":"stop"}
  PID:        {"cmd":"pid",  "dir":"fwd","motor":1,"kp":1.35,"ki":0.5,"kd":0.038}
  Sync:       {"cmd":"sync", "enable":true, "kp":0.5}
  Fixsign:    {"cmd":"fixsign","enable":true}
  Estado:     {"cmd":"status"}
  Reset:      {"cmd":"reset"}
  Ping:       {"cmd":"ping"}

FORMATO JSON RECIBIDO DEL ESP32:
  Telemetría: {"type":"telemetry","rpm1":78.3,"target1":80.0,...}
  ACK:        {"type":"ack","cmd":"set"}
  Error:      {"type":"error","msg":"..."}
  Config:     {"type":"config","pid":{...},"sync":false,...}
"""

import pygame
import json
import time
import sys
import threading
import websocket  # pip install websocket-client

# ═══════════════════════════════════════════════
#  CONFIGURACION
# ═══════════════════════════════════════════════
ESP32_IP  = "192.168.1.53"   # <-- CAMBIA POR LA IP QUE VES EN SERIAL MONITOR
WS_URL    = f"ws://{ESP32_IP}:81"

MAX_RPM   = 150.0
DPAD_RPM  = 100.0
DEAD_JOY  = 0.08
DEAD_TRIG = 0.03

AX_JOY_IZQ_X = 0
AX_LT        = 4
AX_RT        = 5

# Cada cuántos ciclos (50 Hz) se reenvía D-Pad = cada 100 ms
RESEND_EVERY = 5

# ═══════════════════════════════════════════════
#  ESTADO GLOBAL
# ═══════════════════════════════════════════════
rpm1_live  = 0.0
rpm2_live  = 0.0
tgt1_live  = 0.0
tgt2_live  = 0.0
ws_ok      = False
_ws_conn   = None
_last_m1   = None
_last_m2   = None

# ═══════════════════════════════════════════════
#  WEBSOCKET — hilo de fondo
# ═══════════════════════════════════════════════

def on_open(ws):
    global ws_ok, _ws_conn
    _ws_conn = ws
    ws_ok    = True
    print(f"✓ WebSocket conectado  {WS_URL}")
    # Pedir configuración actual al conectar
    ws_send_json({"cmd": "status"})


def on_close(ws, code, msg):
    global ws_ok
    ws_ok = False
    print("✗ WebSocket desconectado — reintentando...")


def on_error(ws, err):
    global ws_ok
    ws_ok = False
    print(f"  WS error: {err}")


def on_message(ws, message):
    """Recibe JSON del ESP32 y actualiza el estado global."""
    global rpm1_live, rpm2_live, tgt1_live, tgt2_live
    try:
        data = json.loads(message)
        msg_type = data.get("type", "")

        if msg_type == "telemetry":
            rpm1_live = data.get("rpm1",    0.0)
            rpm2_live = data.get("rpm2",    0.0)
            tgt1_live = data.get("target1", 0.0)
            tgt2_live = data.get("target2", 0.0)

        elif msg_type == "config":
            print(f"[CONFIG] {json.dumps(data, indent=2)}")

        elif msg_type == "ack":
            pass  # silencioso

        elif msg_type == "error":
            print(f"[ESP32 ERROR] {data.get('msg','?')}")

    except json.JSONDecodeError:
        print(f"[WARN] JSON inválido recibido: {message}")


def _ws_thread():
    """Hilo de reconexión automática."""
    while True:
        try:
            app = websocket.WebSocketApp(
                WS_URL,
                on_open    = on_open,
                on_close   = on_close,
                on_error   = on_error,
                on_message = on_message,
            )
            app.run_forever(ping_interval=10, ping_timeout=5)
        except Exception as e:
            print(f"  Reconectando... ({e})")
        time.sleep(3)


# ═══════════════════════════════════════════════
#  HELPERS DE ENVIO JSON
# ═══════════════════════════════════════════════

def ws_send_json(obj: dict):
    """Serializa un dict como JSON y lo envía por WebSocket."""
    if ws_ok and _ws_conn:
        try:
            _ws_conn.send(json.dumps(obj))
        except Exception as e:
            print(f"  Send error: {e}")
    else:
        print(f"[SIM] {json.dumps(obj)}")


def set_motors(m1: float, m2: float, force: bool = False):
    """
    Envía setpoint de ambos motores en un único mensaje JSON.
    Solo envía si el valor cambió >= 1 RPM (o si force=True).
    """
    global _last_m1, _last_m2
    m1 = round(max(-MAX_RPM, min(MAX_RPM, m1)), 1)
    m2 = round(max(-MAX_RPM, min(MAX_RPM, m2)), 1)

    m1_changed = force or _last_m1 is None or abs(m1 - _last_m1) >= 1.0
    m2_changed = force or _last_m2 is None or abs(m2 - _last_m2) >= 1.0

    if m1_changed or m2_changed:
        payload = {"cmd": "set"}
        if m1_changed:
            payload["m1"] = m1
            _last_m1 = m1
        if m2_changed:
            payload["m2"] = m2
            _last_m2 = m2
        ws_send_json(payload)


def send_stop(motor: int = 0):
    """Detiene motores: 0=ambos, 1=M1, 2=M2."""
    global _last_m1, _last_m2
    ws_send_json({"cmd": "stop", "motor": motor})
    if motor in (0, 1): _last_m1 = 0.0
    if motor in (0, 2): _last_m2 = 0.0


def send_pid(motor: int, direction: str, kp: float, ki: float, kd: float):
    """
    Envía ganancias PID.
    motor: 1 o 2
    direction: "fwd" (avance) o "rev" (retroceso)
    """
    ws_send_json({"cmd": "pid", "motor": motor, "dir": direction,
                  "kp": kp, "ki": ki, "kd": kd})


def send_sync(enable: bool, kp: float = None):
    """Activa/desactiva sincronización lineal y opcionalmente cambia Kp_sync."""
    payload = {"cmd": "sync", "enable": enable}
    if kp is not None:
        payload["kp"] = kp
    ws_send_json(payload)


# ═══════════════════════════════════════════════
#  LOGICA DE MANDO
# ═══════════════════════════════════════════════

def read_trigger(joy, axis):
    raw = max(0.0, min(1.0, joy.get_axis(axis)))
    if raw < DEAD_TRIG:
        return 0.0
    return (raw - DEAD_TRIG) / (1.0 - DEAD_TRIG) * MAX_RPM


def dead(val, zone):
    if abs(val) < zone:
        return 0.0
    sign = 1.0 if val > 0 else -1.0
    return sign * (abs(val) - zone) / (1.0 - zone)


def apply_steering(base_rpm: float, joy_x: float):
    jx = dead(joy_x, DEAD_JOY)
    if abs(base_rpm) < 1.0:
        spin = jx * MAX_RPM
        return -spin, spin
    if jx > 0:
        return base_rpm * (1.0 - 2.0 * jx), base_rpm
    else:
        return base_rpm, base_rpm * (1.0 - 2.0 * abs(jx))


# ═══════════════════════════════════════════════
#  BUCLE PRINCIPAL
# ═══════════════════════════════════════════════

def main():
    pygame.init()
    pygame.joystick.init()

    screen = pygame.display.set_mode((660, 400))
    pygame.display.set_caption(f"ESP32 Robot WiFi — {ESP32_IP}")

    if pygame.joystick.get_count() == 0:
        print("No se detectó ningún control Xbox.")
        pygame.quit()
        sys.exit()

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

    cycle      = 0
    ping_timer = 0

    while True:
        clock.tick(50)
        cycle      += 1
        ping_timer += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                send_stop(); pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                send_stop(); pygame.quit(); sys.exit()

        pygame.event.pump()

        # Ping cada 5 s para mantener el WS vivo
        if ping_timer >= 250:
            ws_send_json({"cmd": "ping"})
            ping_timer = 0

        # ── Leer mando ──
        rt_rpm = read_trigger(joy, AX_RT)
        lt_rpm = read_trigger(joy, AX_LT)
        joy_x  = joy.get_axis(AX_JOY_IZQ_X)
        dpad   = joy.get_hat(0)

        base_rpm     = 0.0
        use_steering = True
        force_send   = (cycle % RESEND_EVERY == 0)

        # ── Prioridad: Gatillos > D-Pad > Stop ──
        if rt_rpm > 0.5 and lt_rpm > 0.5:
            base_rpm   = rt_rpm if rt_rpm >= lt_rpm else -lt_rpm
            mode_str   = f"{'RT AVANCE' if base_rpm>0 else 'LT RETROCESO'}  {base_rpm:+.0f} RPM"
            mode_color = (80, 220, 80) if base_rpm > 0 else (240, 100, 100)

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
            mode_str   = f"D-PAD ↑  +{DPAD_RPM:.0f} RPM"
            mode_color = (80, 180, 255)

        elif dpad[1] == -1:
            base_rpm   = -DPAD_RPM
            mode_str   = f"D-PAD ↓  -{DPAD_RPM:.0f} RPM"
            mode_color = (255, 160, 50)

        elif dpad[0] == -1:
            m1_out, m2_out = +DPAD_RPM, -DPAD_RPM
            set_motors(m1_out, m2_out, force=force_send)
            m1_disp, m2_disp = m1_out, m2_out
            use_steering = False
            mode_str   = f"D-PAD ← M1={m1_out:+.0f} M2={m2_out:+.0f}"
            mode_color = (200, 160, 255)

        elif dpad[0] == 1:
            m1_out, m2_out = -DPAD_RPM, +DPAD_RPM
            set_motors(m1_out, m2_out, force=force_send)
            m1_disp, m2_disp = m1_out, m2_out
            use_steering = False
            mode_str   = f"D-PAD → M1={m1_out:+.0f} M2={m2_out:+.0f}"
            mode_color = (200, 160, 255)

        else:
            if _last_m1 != 0.0 or _last_m2 != 0.0:
                send_stop()
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

        ws_col = (80, 220, 80) if ws_ok else (200, 80, 80)
        ws_txt = f"WiFi {ESP32_IP}  CONECTADO" if ws_ok else f"WiFi {ESP32_IP}  RECONECTANDO..."

        screen.blit(font_big.render("ESP32 ROBOT  —  Xbox 360  WiFi/JSON", True, (88, 166, 255)), (18, 12))
        screen.blit(font_sml.render(ws_txt, True, ws_col), (18, 36))
        screen.blit(font_big.render(mode_str, True, mode_color), (18, 54))

        def motor_bar(setpt, rpm_real, y, label):
            pygame.draw.rect(screen, (28, 34, 44), (18, y, 200, 20), border_radius=4)
            w = int(abs(setpt) / MAX_RPM * 200)
            col_bg = (40, 120, 40) if setpt >= 0 else (120, 40, 40)
            if w > 0:
                pygame.draw.rect(screen, col_bg, (18, y, min(w, 200), 20), border_radius=4)
            w2 = int(abs(rpm_real) / MAX_RPM * 200)
            col_fg = (70, 210, 70) if rpm_real >= 0 else (230, 90, 90)
            if w2 > 0:
                pygame.draw.rect(screen, col_fg, (18, y, min(w2, 200), 10), border_radius=4)
            screen.blit(font_med.render(
                f"{label}  cmd={setpt:+5.0f}  real={rpm_real:+5.1f}",
                True, (190, 190, 190)), (228, y + 2))

        motor_bar(m1_disp, rpm1_live, 88,  "M1 DER")
        motor_bar(m2_disp, rpm2_live, 115, "M2 IZQ")

        def trig_bar(rpm, y, label, col):
            pygame.draw.rect(screen, (28, 34, 44), (18, y, 130, 12), border_radius=3)
            w = int(rpm / MAX_RPM * 130)
            if w > 0:
                pygame.draw.rect(screen, col, (18, y, min(w, 130), 12), border_radius=3)
            screen.blit(font_sml.render(f"{label} {rpm:.0f} RPM", True, (130, 130, 130)), (158, y))

        trig_bar(rt_rpm, 150, "RT", (70, 210, 70))
        trig_bar(lt_rpm, 168, "LT", (230, 90, 90))

        # Joystick
        jx  = dead(joy_x, DEAD_JOY)
        cx, cy = 110, 245
        pygame.draw.circle(screen, (28, 34, 44), (cx, cy), 46)
        pygame.draw.circle(screen, (55, 65, 76), (cx, cy), 46, 1)
        pygame.draw.line(screen, (50, 60, 70), (cx - 46, cy), (cx + 46, cy), 1)
        dot_x = cx + int(jx * 44)
        pygame.draw.circle(screen, (88, 166, 255) if abs(jx) > 0.05 else (55, 65, 76), (dot_x, cy), 10)
        screen.blit(font_sml.render(f"Joy X {joy_x:+.3f}", True, (80, 90, 100)), (70, 298))

        # D-Pad
        dpx, dpy = 310, 239
        dc = (88, 166, 255); di = (32, 40, 50)
        pygame.draw.rect(screen, dc if dpad[1] == 1  else di, (dpx-12, dpy-44, 24, 34), border_radius=4)
        pygame.draw.rect(screen, dc if dpad[1] == -1 else di, (dpx-12, dpy+10, 24, 34), border_radius=4)
        pygame.draw.rect(screen, dc if dpad[0] == -1 else di, (dpx-54, dpy-12, 36, 24), border_radius=4)
        pygame.draw.rect(screen, dc if dpad[0] == 1  else di, (dpx+18, dpy-12, 36, 24), border_radius=4)
        pygame.draw.rect(screen, (40, 50, 60), (dpx-12, dpy-10, 24, 20), border_radius=3)

        # RPM en tiempo real
        screen.blit(font_sml.render("RPM en tiempo real:", True, (70, 80, 90)), (450, 195))
        def mini_bar(val, y, label):
            pygame.draw.rect(screen, (28, 34, 44), (450, y, 140, 14), border_radius=3)
            w = int(abs(val) / MAX_RPM * 140)
            col = (70, 210, 70) if val >= 0 else (230, 90, 90)
            if w > 0:
                pygame.draw.rect(screen, col, (450, y, min(w, 140), 14), border_radius=3)
            screen.blit(font_sml.render(f"{label} {val:+.0f}", True, (150, 150, 150)), (600, y))
        mini_bar(rpm1_live, 215, "M1")
        mini_bar(rpm2_live, 235, "M2")

        # JSON info
        screen.blit(font_sml.render(f"JSON TX → {{\"cmd\":\"set\",\"m1\":{m1_disp:+.0f},\"m2\":{m2_disp:+.0f}}}", True, (55, 100, 55)), (18, 328))

        # Hints
        hints = [
            "RT=avance proporcional  |  LT=retroceso proporcional",
            "D-Pad UD=+-100RPM fijo  |  D-Pad LR=giro en sitio",
            "Joy IZQ X=dirección     |  ESC=stop y salir",
        ]
        for i, h in enumerate(hints):
            screen.blit(font_sml.render(h, True, (55, 65, 75)), (18, 350 + i * 16))

        pygame.display.flip()


# ═══════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════
if __name__ == "__main__":
    # Verificar dependencias
    try:
        import websocket
    except ImportError:
        import subprocess
        subprocess.run([sys.executable, "-m", "pip", "install", "websocket-client"], check=True)
        import websocket

    # Lanzar hilo WebSocket en fondo (reconexión automática)
    threading.Thread(target=_ws_thread, daemon=True).start()
    time.sleep(0.5)  # dar tiempo a la conexión inicial

    main()