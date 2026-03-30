"""
DIAGNOSTICO WiFi ESP32 <-> Python
===================================
Este script ayuda a identificar problemas de comunicación
"""

import socket
import time
import sys

# ═══════════════════════════════════════════════
#  CONFIGURACION
# ═══════════════════════════════════════════════
ESP32_IP = "192.168.211.147"  # <-- Cambiar por la IP real del ESP32
UDP_PORT_TX = 4210  # Puerto donde ESP32 ESCUCHA
UDP_PORT_RX = 4211  # Puerto donde ESP32 ENVIA

print("═" * 60)
print("  DIAGNÓSTICO WiFi ESP32 <-> Python")
print("═" * 60)
print(f"ESP32 IP:       {ESP32_IP}")
print(f"Puerto TX:      {UDP_PORT_TX} (Python → ESP32)")
print(f"Puerto RX:      {UDP_PORT_RX} (ESP32 → Python)")
print("═" * 60)

# ═══════════════════════════════════════════════
#  TEST 1: Verificar IP local de la PC
# ═══════════════════════════════════════════════
print("\n[TEST 1] IP de tu PC:")
try:
    # Crear socket temporal para obtener IP local
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    local_ip = s.getsockname()[0]
    s.close()
    print(f"  ✓ Tu PC tiene IP: {local_ip}")
    
    # Verificar si están en la misma red
    esp32_prefix = ".".join(ESP32_IP.split(".")[0:3])
    pc_prefix = ".".join(local_ip.split(".")[0:3])
    
    if esp32_prefix == pc_prefix:
        print(f"  ✓ Misma red: {esp32_prefix}.x")
    else:
        print(f"  ✗ ERROR: Diferentes redes!")
        print(f"    ESP32: {esp32_prefix}.x")
        print(f"    PC:    {pc_prefix}.x")
        print("  → Conecta la PC al mismo hotspot que el ESP32")
        sys.exit(1)
        
except Exception as e:
    print(f"  ✗ No se pudo obtener IP: {e}")
    sys.exit(1)

# ═══════════════════════════════════════════════
#  TEST 2: Crear sockets UDP
# ═══════════════════════════════════════════════
print("\n[TEST 2] Creando sockets UDP:")
try:
    # Socket para ENVIAR a ESP32
    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_tx.settimeout(1.0)
    print(f"  ✓ Socket TX creado (Python → ESP32:{UDP_PORT_TX})")
    
    # Socket para RECIBIR de ESP32
    sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rx.bind(("", UDP_PORT_RX))
    sock_rx.settimeout(2.0)
    print(f"  ✓ Socket RX creado (escuchando en puerto {UDP_PORT_RX})")
    
except Exception as e:
    print(f"  ✗ Error creando sockets: {e}")
    sys.exit(1)

# ═══════════════════════════════════════════════
#  TEST 3: Enviar comando PING
# ═══════════════════════════════════════════════
print("\n[TEST 3] Enviando comando 'ping' al ESP32:")
print(f"  → Destino: {ESP32_IP}:{UDP_PORT_TX}")

try:
    # Enviar comando
    mensaje = "ping"
    sock_tx.sendto(mensaje.encode(), (ESP32_IP, UDP_PORT_TX))
    print(f"  ✓ Comando enviado: '{mensaje}'")
    
    # Esperar respuesta PONG por UDP
    print("  → Esperando respuesta 'PONG' por UDP...")
    
    start_time = time.time()
    respuesta_recibida = False
    
    while time.time() - start_time < 3.0:
        try:
            data, addr = sock_rx.recvfrom(512)
            mensaje_rx = data.decode(errors='ignore').strip()
            print(f"  ✓ Respuesta UDP recibida de {addr}: '{mensaje_rx}'")
            respuesta_recibida = True
            break
        except socket.timeout:
            continue
    
    if not respuesta_recibida:
        print("  ✗ No se recibió respuesta UDP en 3 segundos")
        print("\n  POSIBLES CAUSAS:")
        print("  1. ESP32 no está conectado al hotspot")
        print("  2. IP incorrecta en ESP32_IP")
        print("  3. Firewall bloqueando UDP")
        print("  4. Puerto UDP incorrecto")
        print("\n  QUÉ HACER:")
        print("  → Verifica Serial Monitor del ESP32")
        print("  → Debe mostrar: >> PONG")
        print("  → Si no muestra nada, el ESP32 no recibe comandos")
        
except Exception as e:
    print(f"  ✗ Error: {e}")

# ═══════════════════════════════════════════════
#  TEST 4: Enviar comando de motor
# ═══════════════════════════════════════════════
print("\n[TEST 4] Enviando comando 'm1=30' al ESP32:")

try:
    mensaje = "m1=30"
    sock_tx.sendto(mensaje.encode(), (ESP32_IP, UDP_PORT_TX))
    print(f"  ✓ Comando enviado: '{mensaje}'")
    
    # Esperar telemetría
    print("  → Esperando telemetría...")
    
    start_time = time.time()
    telemetria_recibida = False
    
    while time.time() - start_time < 3.0:
        try:
            data, addr = sock_rx.recvfrom(512)
            mensaje_rx = data.decode(errors='ignore').strip()
            
            if "Target1:" in mensaje_rx:
                print(f"  ✓ Telemetría recibida:")
                print(f"    {mensaje_rx}")
                telemetria_recibida = True
                break
                
        except socket.timeout:
            continue
    
    if not telemetria_recibida:
        print("  ✗ No se recibió telemetría")
    
    # Detener motor
    time.sleep(0.5)
    sock_tx.sendto(b"stop", (ESP32_IP, UDP_PORT_TX))
    print("  → Comando 'stop' enviado")
    
except Exception as e:
    print(f"  ✗ Error: {e}")

# ═══════════════════════════════════════════════
#  TEST 5: Verificar firewall
# ═══════════════════════════════════════════════
print("\n[TEST 5] Verificación de firewall:")
print("  Si los tests anteriores fallaron, verifica:")
print("  • Windows: Firewall de Windows puede bloquear UDP")
print("  • Mac: Configuración de red en Preferencias del Sistema")
print("  • Linux: iptables puede bloquear puertos UDP")

# ═══════════════════════════════════════════════
#  RESUMEN
# ═══════════════════════════════════════════════
print("\n" + "═" * 60)
print("  RESUMEN Y PRÓXIMOS PASOS")
print("═" * 60)
print("\n1. Verifica el Serial Monitor del ESP32:")
print("   - Debe mostrar la IP correcta")
print("   - Al enviar 'ping', debe mostrar: >> PONG")
print("   - Al enviar 'm1=30', debe mostrar: >> M1 setpoint: 30.0")

print("\n2. Si ESP32 NO responde:")
print("   - Verifica WIFI_SSID y WIFI_PASS en el código ESP32")
print("   - Verifica que el hotspot esté activo")
print("   - Reinicia el ESP32")

print("\n3. Si ESP32 responde en Serial pero Python no recibe:")
print("   - Problema de firewall")
print("   - Verifica puertos UDP (4210 y 4211)")

print("\n4. Si todo funciona:")
print("   - Ejecuta xbox_wifi.py normalmente")

print("\n" + "═" * 60)

# Cerrar sockets
sock_tx.close()
sock_rx.close()