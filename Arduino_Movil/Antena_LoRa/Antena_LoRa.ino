// =====================================================
//   ESP32 ANTENA — LoRa TX  (4 orugas)
//   Recibe comandos de Python por Serial USB
//   Los reenvía a la ESP32 receptora por LoRa
//   Recibe telemetría de vuelta y la reenvía a Python
//
//   Comandos de oruga:
//     l=80   → oruga izquierda (M1+M3)
//     r=80   → oruga derecha   (M2+M4)
//     lr=80  → ambas orugas
//     stop   → parar todo
//     cualquier otro → reenviar directo al Mega
//
//   PINES LoRa Ra-02 → ESP32:
//   NSS/CS → GPIO 5   RST  → GPIO 14
//   SCK    → GPIO 18  DIO0 → GPIO 26
//   MOSI   → GPIO 23  MISO → GPIO 19
//   VCC    → 3.3V     GND  → GND
//
//   LIBRERÍA: LoRa by Sandeep Mistry
//   COMPILAR: ESP32 Dev Module  |  BAUD: 115200
// =====================================================

#include <SPI.h>
#include <LoRa.h>

#define LORA_NSS  5
#define LORA_RST  14
#define LORA_DIO0 26

#define LORA_FREQ  433E6
#define LORA_SF    7
#define LORA_BW    125E3
#define LORA_CR    5
#define LORA_SYNCWORD 0x34
#define LORA_TX_POWER 17

#define ACK_TIMEOUT_MS 5000
#define KEEPALIVE_MS   500

// ── Estructura de paquete — idéntica en ambas ESP32 ──
// l  = setpoint oruga izquierda (M1+M3)
// r  = setpoint oruga derecha   (M2+M4)
#pragma pack(push, 1)
struct Packet {
  uint8_t seq;
  uint8_t type;    // 'M'=orugas  'X'=cmd extra  'S'=stop
  float   l;       // oruga izquierda  (-150 a +150 RPM)
  float   r;       // oruga derecha    (-150 a +150 RPM)
  char    cmd[18]; // comando extra (kpl=, kpr=, pid, etc.)
};
// sizeof = 1+1+4+4+18 = 28 bytes
#pragma pack(pop)

uint8_t seqNum   = 0;
float   targetL  = 0.0f;
float   targetR  = 0.0f;
bool    pendingCmd    = false;
char    pendingCmdStr[18] = {0};

unsigned long lastSend = 0;
unsigned long lastAck  = 0;
bool waitingAck        = false;

static String serialLineBuf;

// =====================================================
//  SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  delay(500);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  ESP32 ANTENA — LoRa TX  (4 orugas)");
  Serial.println("==========================================");

  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERR:LORA_INIT"); while (1) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNCWORD);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.enableCrc();

  Serial.println("OK:LORA_READY");
  Serial.println("OK:ANTENA_TX");
  Serial.println("  Comandos: l=  r=  lr=  stop  kpl=  kpr= ...");
  lastAck = lastSend = millis();
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  // Recibir ACK / telemetría primero — no bloquear con Serial
  int pktSize = LoRa.parsePacket();
  if (pktSize > 0) {
    uint8_t buf[128] = {0};
    int len = 0;
    while (LoRa.available() && len < (int)sizeof(buf)-1)
      buf[len++] = LoRa.read();
    buf[len] = 0;

    String msg = String((char*)buf);
    msg.trim();
    if (msg.length() > 0) {
      if (msg.startsWith("ACK:")) {
        lastAck = now;
        waitingAck = false;
      } else {
        Serial.println(msg);   // → Python
      }
    }
  }

  // Leer comandos (no bloqueante: acumular hasta salto de línea)
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialLineBuf.trim();
      if (serialLineBuf.length() > 0) parseCommand(serialLineBuf);
      serialLineBuf = "";
    } else {
      if (serialLineBuf.length() < 96) serialLineBuf += c;
      else serialLineBuf = "";
    }
  }

  // Keepalive — mantiene watchdog del Mega
  if (now - lastSend >= KEEPALIVE_MS) {
    lastSend = now;
    sendPacket();
  }

  // Detectar pérdida de enlace
  if (waitingAck && now - lastAck > ACK_TIMEOUT_MS) {
    waitingAck = false;
    Serial.println("ERR:LORA_TIMEOUT");
  }
}

// =====================================================
//  PARSEAR COMANDO DE PYTHON
// =====================================================
void parseCommand(const String& raw) {
  String cmd = raw;
  cmd.trim(); cmd.toLowerCase();

  // Aliases de movimiento por palabra
  if (cmd == "forward") {
    targetL = 80.0f; targetR = 80.0f;
    sendPacket(); return;
  }
  if (cmd == "back") {
    targetL = -80.0f; targetR = -80.0f;
    sendPacket(); return;
  }
  if (cmd == "left") {
    targetL = -80.0f; targetR = 80.0f;
    sendPacket(); return;
  }
  if (cmd == "right") {
    targetL = 80.0f; targetR = -80.0f;
    sendPacket(); return;
  }

  // Setpoints de orugas — enviar de inmediato
  if (cmd.startsWith("l=")) {
    targetL = constrain(cmd.substring(2).toFloat(), -150.0f, 150.0f);
    sendPacket(); return;
  }
  if (cmd.startsWith("r=")) {
    targetR = constrain(cmd.substring(2).toFloat(), -150.0f, 150.0f);
    sendPacket(); return;
  }
  if (cmd.startsWith("lr=")) {
    float v = constrain(cmd.substring(3).toFloat(), -150.0f, 150.0f);
    targetL = v; targetR = v;
    sendPacket(); return;
  }
  if (cmd == "stop") {
    targetL = 0; targetR = 0;
    for (int i = 0; i < 3; i++) {
      sendStop();
      delay(5);
    }
    Serial.println("OK:stop"); return;
  }

  // Cualquier otro comando (PID, diag, etc.) → campo cmd
  pendingCmd = true;
  memset(pendingCmdStr, 0, sizeof(pendingCmdStr));
  raw.toCharArray(pendingCmdStr, sizeof(pendingCmdStr));
  sendPacket();
}

// =====================================================
//  ENVIAR PAQUETE DE ORUGAS
// =====================================================
void sendPacket() {
  Packet pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.seq  = seqNum++;
  pkt.l    = targetL;
  pkt.r    = targetR;

  if (pendingCmd) {
    pkt.type = 'X';
    strncpy(pkt.cmd, pendingCmdStr, sizeof(pkt.cmd)-1);
    pendingCmd = false;
    memset(pendingCmdStr, 0, sizeof(pendingCmdStr));
  } else {
    pkt.type = 'M';
  }

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();

  waitingAck = true;
  lastSend   = millis();

  Serial.print("OK:L="); Serial.print(pkt.l, 0);
  Serial.print(" R=");    Serial.println(pkt.r, 0);
}

// =====================================================
//  STOP EXPLÍCITO
// =====================================================
void sendStop() {
  Packet pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.seq  = seqNum++;
  pkt.type = 'S';
  pkt.l    = 0; pkt.r = 0;
  strncpy(pkt.cmd, "stop", sizeof(pkt.cmd)-1);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();
}
