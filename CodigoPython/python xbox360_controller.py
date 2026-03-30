"""
Xbox 360 – Diagnóstico RAW para Windows (versión corregida)
=============================================================
Muestra en consola EXACTAMENTE qué índice cambia al presionar
cada botón, mover los joysticks o los gatillos.

Requisitos:  pip install pygame
"""

import pygame
import sys
from datetime import datetime

def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def main():
    pygame.init()
    pygame.joystick.init()

    screen = pygame.display.set_mode((480, 140))
    pygame.display.set_caption("Xbox 360 – Diagnóstico RAW  (ver consola)")
    font = pygame.font.SysFont("consolas", 14)

    if pygame.joystick.get_count() == 0:
        print("No se detectó ningún control. Conéctalo y vuelve a ejecutar.")
        pygame.quit()
        sys.exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"\n{'='*55}")
    print(f"  Control detectado: {joystick.get_name()}")
    print(f"  Ejes   : {joystick.get_numaxes()}")
    print(f"  Botones: {joystick.get_numbuttons()}")
    print(f"  Hats   : {joystick.get_numhats()}")
    print(f"{'='*55}")
    print("  Mueve/presiona cada control para ver su índice...\n")

    prev_axes    = [joystick.get_axis(i)   for i in range(joystick.get_numaxes())]
    prev_buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    prev_hats    = [joystick.get_hat(i)    for i in range(joystick.get_numhats())]

    DEAD_ZONE = 0.02
    clock = pygame.time.Clock()

    while True:
        clock.tick(60)

        # ── Procesar eventos de ventana (cerrar con X o ESC) ─────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.quit()
                sys.exit()

        # ── Actualizar estado del joystick via polling ────────────────────
        pygame.event.pump()

        cur_axes    = [joystick.get_axis(i)   for i in range(joystick.get_numaxes())]
        cur_buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
        cur_hats    = [joystick.get_hat(i)    for i in range(joystick.get_numhats())]

        # ── Detectar cambios en BOTONES ───────────────────────────────────
        for i, (prev, cur) in enumerate(zip(prev_buttons, cur_buttons)):
            if prev != cur:
                estado = "PRESIONADO" if cur else "soltado  "
                print(f"[{ts()}]  BOTÓN   índice={i:<2}  →  {estado}")

        # ── Detectar cambios en EJES ──────────────────────────────────────
        for i, (prev, cur) in enumerate(zip(prev_axes, cur_axes)):
            if abs(cur - prev) > DEAD_ZONE:
                bar_len = int((cur + 1) / 2 * 20)
                bar = "[" + "█" * bar_len + "░" * (20 - bar_len) + "]"
                print(f"[{ts()}]  EJE     índice={i:<2}  →  {cur:+.4f}  {bar}")

        # ── Detectar cambios en HATS (D-Pad) ─────────────────────────────
        for i, (prev, cur) in enumerate(zip(prev_hats, cur_hats)):
            if prev != cur:
                dirs = []
                if cur[1] ==  1: dirs.append("ARRIBA")
                if cur[1] == -1: dirs.append("ABAJO")
                if cur[0] == -1: dirs.append("IZQUIERDA")
                if cur[0] ==  1: dirs.append("DERECHA")
                texto = " + ".join(dirs) if dirs else "neutro"
                print(f"[{ts()}]  D-PAD   hat={i}  valor={cur}  →  {texto}")

        prev_axes    = cur_axes
        prev_buttons = cur_buttons
        prev_hats    = cur_hats

        # ── Info en ventana ───────────────────────────────────────────────
        screen.fill((15, 15, 15))
        lines = [
            f"Control: {joystick.get_name()}",
            f"Ejes: {joystick.get_numaxes()}   Botones: {joystick.get_numbuttons()}   Hats: {joystick.get_numhats()}",
            ">> Mueve el control y mira la CONSOLA <<",
            "ESC para salir",
        ]
        colors = [(255,255,255), (180,180,180), (80,220,80), (120,120,120)]
        for j, (line, color) in enumerate(zip(lines, colors)):
            screen.blit(font.render(line, True, color), (20, 18 + j * 24))

        pygame.display.flip()


if __name__ == "__main__":
    main()