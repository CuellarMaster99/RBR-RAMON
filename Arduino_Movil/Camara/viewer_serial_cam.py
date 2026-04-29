import argparse
import struct
import time
from urllib.parse import urlsplit

import cv2
import numpy as np
import requests
import serial
from serial.tools import list_ports

# OpenCV / libjpeg escriben "Corrupt JPEG data..." a stderr aunque imdecode falle
# (sobre todo con ráfagas seriales, timeouts cortos o tráfico de texto + FRAM mezclado).
try:
    cv2.setLogLevel(0)  # LOG_LEVEL_SILENT
except (AttributeError, TypeError):
    try:
        cv2.utils.logging.setLogLevel(0)  # type: ignore[attr-defined]
    except (AttributeError, TypeError):
        pass

MAGIC = b"FRAM"
HEADER_LEN = 8  # 4 bytes magic + 4 bytes payload length
MAX_FRAME_SIZE = 50000

# A 115200, un JPEG de ~20–40 KB tarda 1,5–3 s. timeout global corto hace que
# read() devuelva 0 mientras aún faltan bytes; read_exact antiguo interpretaba
# eso como fallo y descartaba bytes ya leídos → basura y "bad Huffman code".
# Usamos timeout=None en lecturas de bloque fijo, o reintentar sin perder el buffer.
_SERIAL_BLOCK_TIMEOUT = 120.0  # s máximo por tramo (payload grande a bajo baudios)


def read_exact(ser: serial.Serial, nbytes: int) -> bytes | None:
    """Lee exactamente nbytes, sin perder progreso si pyserial hace un timeout
    a mitad (problema típico con --timeout=1.0 e imágenes > ~10 KB a 115200)."""
    data = b""
    deadline = time.monotonic() + _SERIAL_BLOCK_TIMEOUT
    while len(data) < nbytes:
        if time.monotonic() > deadline:
            return None
        n = nbytes - len(data)
        # Bloquea hasta n bytes: evita corte a mitad por timeout por defecto
        # (en run_serial_viewer forzamos timeout=None durante cabecera+JPEG).
        chunk = ser.read(n)
        if not chunk:
            if len(data) == 0:
                return None
            # Sigue esperando: no soltar data parcial (p. ej. timeout entre ráfagas USB)
            continue
        data += chunk
    return data


def sync_to_magic(ser: serial.Serial) -> bool:
    window = b""
    while True:
        b = ser.read(1)
        if not b:
            return False
        window = (window + b)[-4:]
        if window == MAGIC:
            return True


def choose_serial_port(port_arg: str | None) -> str:
    if port_arg:
        return port_arg

    ports = sorted(list_ports.comports(), key=lambda p: p.device)
    if not ports:
        raise RuntimeError("No se detectaron puertos seriales.")

    print("Puertos seriales disponibles:")
    for idx, p in enumerate(ports, start=1):
        desc = p.description or "Sin descripcion"
        print(f"  {idx}) {p.device} - {desc}")

    while True:
        choice = input("Selecciona puerto (numero o COMx): ").strip()
        if not choice:
            continue
        if choice.upper().startswith("COM"):
            return choice.upper()
        if choice.isdigit():
            i = int(choice)
            if 1 <= i <= len(ports):
                return ports[i - 1].device
        print("Entrada invalida. Ejemplos: 1 o COM6")


def show_frame(frame, fps):
    if fps > 0:
        cv2.putText(
            frame,
            f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    cv2.imshow("ESP32 Cam Viewer", frame)
    return (cv2.waitKey(1) & 0xFF) == ord("q")


def run_url_viewer(url: str, retry_forever: bool, max_retries: int) -> None:
    print(f"[INFO] Stream URL: {url}")
    print("[INFO] Presiona 'q' para salir.")
    last_fps_t = time.time()
    fps_count = 0
    fps = 0.0

    session = requests.Session()
    # Evita que requests tome proxies del sistema (frecuente causa de timeout local).
    session.trust_env = False

    parts = urlsplit(url)
    status_url = f"{parts.scheme}://{parts.netloc}/status"
    try:
        status_resp = session.get(status_url, timeout=(4, 6))
        print(f"[INFO] /status -> HTTP {status_resp.status_code}: {status_resp.text[:200]}")
    except Exception as exc:
        print(f"[WARN] No se pudo leer /status ({status_url}): {exc}")

    retries = 0
    while True:
        try:
            with session.get(url, stream=True, timeout=(10, 30)) as resp:
                if resp.status_code != 200:
                    raise RuntimeError(f"HTTP {resp.status_code} al abrir stream")

                data = b""
                for chunk in resp.iter_content(chunk_size=4096):
                    if not chunk:
                        continue
                    data += chunk

                    # Extrae JPEGs por marcadores SOI/EOI
                    while True:
                        a = data.find(b"\xff\xd8")
                        if a < 0:
                            # Evitar crecimiento infinito si aun no hay SOI.
                            if len(data) > 2_000_000:
                                data = data[-100_000:]
                            break
                        b = data.find(b"\xff\xd9", a + 2)
                        if b < 0:
                            # Esperar mas bytes del stream.
                            if a > 0:
                                data = data[a:]
                            break

                        jpg = data[a:b + 2]
                        data = data[b + 2:]
                        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if frame is None:
                            continue

                        fps_count += 1
                        now = time.time()
                        if now - last_fps_t >= 1.0:
                            fps = fps_count / (now - last_fps_t)
                            fps_count = 0
                            last_fps_t = now

                        if show_frame(frame, fps):
                            return
        except Exception as exc:
            retries += 1
            print(f"[WARN] Stream caido/no disponible: {exc}")
            if not retry_forever and retries >= max_retries:
                raise RuntimeError(
                    f"No se pudo abrir stream tras {retries} intentos. "
                    "Verifica que /status reporte frames>0 y que no haya otra app usando /stream."
                ) from exc
            print(f"[INFO] Reintentando en 2s... ({retries})")
            time.sleep(2)


def run_serial_viewer(port: str, baud: int, timeout: float) -> None:
    # timeout corto: solo buscando "FRAM" (no acumulamos milisegundos a mitad de JPEG)
    ser = serial.Serial(port, baud, timeout=timeout)
    print(f"[INFO] Abierto {port} @ {baud}")
    print("[INFO] Presiona 'q' para salir.")
    print("[INFO] Esperando frames...")
    if baud <= 230400:
        print(
            "[HINT] A 115200/230400, JPEGs de varios tens de KB tardan >1s; "
            "si ves cortes, sube en ESP32+PC: --baud 921600 (o no uses --timeout bajo con TX mixto LoRa+FRAM)."
        )

    last_fps_t = time.time()
    last_wait_msg_t = time.time()
    fps_count = 0
    fps = 0.0

    try:
        while True:
            if not sync_to_magic(ser):
                now = time.time()
                if now - last_wait_msg_t >= 2.0:
                    print("[INFO] Sin datos de video aun. Verifica enlace ESP-NOW y MAC.")
                    last_wait_msg_t = now
                continue
            # Sin límite por byte entre longitud+JPEG: un timeout=1.0 en read(n)
            # hace perder tramos largos; volvemos al timeout "normal" después.
            _saved = ser.timeout
            ser.timeout = None
            try:
                len_bytes = read_exact(ser, 4)
            finally:
                ser.timeout = _saved
            if len_bytes is None:
                continue
            (frame_len,) = struct.unpack("<I", len_bytes)

            if frame_len == 0 or frame_len > MAX_FRAME_SIZE:
                # Longitud invalida: resincronizar buscando siguiente magic.
                continue

            ser.timeout = None
            try:
                jpeg = read_exact(ser, frame_len)
            finally:
                ser.timeout = _saved
            if jpeg is None:
                continue
            # Filtra frames claramente corruptos para evitar warnings de JPEG.
            if len(jpeg) < 4 or not (jpeg[0] == 0xFF and jpeg[1] == 0xD8 and jpeg[-2] == 0xFF and jpeg[-1] == 0xD9):
                continue
            if len(jpeg) != frame_len:
                continue

            img_array = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_UNCHANGED)
            if frame is None or frame.size == 0:
                continue
            if len(frame.shape) < 2:
                continue
            # Asegurar 3 canales BGR para imshow
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            fps_count += 1
            now = time.time()
            if now - last_fps_t >= 1.0:
                fps = fps_count / (now - last_fps_t)
                fps_count = 0
                last_fps_t = now

            if show_frame(frame, fps):
                break
    finally:
        ser.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Viewer ESP32 por serial FRAM o URL MJPEG.")
    parser.add_argument("--url", help="URL de stream MJPEG, ej. http://192.168.4.1/stream")
    parser.add_argument("--port", help="Puerto serial, ej. COM6. Si se omite, se elige por menu.")
    parser.add_argument("--baud", type=int, default=921600, help="Baudrate (default: 921600)")
    parser.add_argument("--timeout", type=float, default=1.0, help="Timeout serial en segundos")
    parser.add_argument("--retry-forever", action="store_true", help="Reintenta URL indefinidamente")
    parser.add_argument("--max-retries", type=int, default=10, help="Intentos maximos URL antes de salir")
    args = parser.parse_args()

    try:
        if args.url:
            run_url_viewer(args.url, retry_forever=args.retry_forever, max_retries=args.max_retries)
        elif not args.port:
            mode = input("Modo [1=URL, 2=COM] (default 2): ").strip()
            if mode == "1":
                url = input("Ingresa URL de stream (ej. http://192.168.4.1/stream): ").strip()
                if not url:
                    raise RuntimeError("No se ingreso URL.")
                run_url_viewer(url, retry_forever=args.retry_forever, max_retries=args.max_retries)
            else:
                port = choose_serial_port(args.port)
                run_serial_viewer(port, args.baud, args.timeout)
        else:
            port = choose_serial_port(args.port)
            run_serial_viewer(port, args.baud, args.timeout)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        print("[INFO] Viewer cerrado.")


if __name__ == "__main__":
    main()
