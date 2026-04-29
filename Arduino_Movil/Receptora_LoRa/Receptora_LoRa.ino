// =====================================================
//   ESP32 RECEPTORA — LoRa RX + UART → Mega
//   4 orugas — reenvía l= / r= / lr= al Mega
//
//   PINES LoRa Ra-02 → ESP32:
//   NSS/CS → GPIO 5   RST  → GPIO 14
//   SCK    → GPIO 18  DIO0 → GPIO 26
//   MOSI   → GPIO 23  MISO → GPIO 19
//
//   UART → Arduino Mega:
//   TX2 (GPIO 17) → RX1 Mega (Pin 19)
//   RX2 (GPIO 16) ← TX1 Mega (Pin 18)
//   GND → GND común
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

#define UART_BAUD   115200
#define WATCHDOG_MS 2000
#define TELEM_GAP_MS 150
#define CMD_GUARD_MS 100

// ── Estructura idéntica a la antena ──
#pragma pack(push, 1)
struct Packet {
  uint8_t seq;
  uint8_t type;
  float   l;
  float   r;
  char    cmd[18];
};
#pragma pack(pop)

uint8_t       lastSeq  = 255;
unsigned long lastPkt  = 0;
bool          wdFired  = false;
String        megaBuf  = "";
String        pendingTelem = "";
unsigned long lastCmdRxMs = 0;
unsigned long lastTelemTxMs = 0;

uint32_t pktOK  = 0;
uint32_t pktDup = 0;
uint32_t pktBad = 0;

void sendAck(uint8_t seq);

// =====================================================
//  SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, 16, 17);
  delay(500);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  ESP32 RECEPTORA — LoRa RX + UART Mega");
  Serial.println("  4 orugas");
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
  Serial.println("OK:UART_MEGA_READY");
  lastPkt = millis();
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  // Recibir paquete LoRa
  int pktSize = LoRa.parsePacket();
  if (pktSize == sizeof(Packet)) {
    Packet pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(pkt));
    lastPkt = now; wdFired = false; lastCmdRxMs = now;

    if (pkt.seq == lastSeq) {
      pktDup++;
      sendAck(pkt.seq);
    } else {
      lastSeq = pkt.seq;
      pktOK++;
      processPacket(pkt);
      sendAck(pkt.seq);
    }
  } else if (pktSize > 0) {
    while (LoRa.available()) LoRa.read();
    pktBad++;
    Serial.print("WARN:PKT_SIZE="); Serial.println(pktSize);
  }

  // Watchdog → stop al Mega
  if (!wdFired && now - lastPkt > WATCHDOG_MS) {
    wdFired = true;
    Serial2.println("stop");
    Serial.println("WARN:LORA_TIMEOUT — stop al Mega");
  }

  // Leer telemetría del Mega y reenviar por LoRa
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      megaBuf.trim();
      if (megaBuf.length() > 0) pendingTelem = megaBuf;
      megaBuf = "";
    } else {
      megaBuf += c;
      if (megaBuf.length() > 160) megaBuf = "";
    }
  }

  if (pendingTelem.length() > 0 &&
      now - lastCmdRxMs >= CMD_GUARD_MS &&
      now - lastTelemTxMs >= TELEM_GAP_MS) {
    forwardTelemetry(pendingTelem);
    pendingTelem = "";
    lastTelemTxMs = now;
  }

  // Debug periódico
  static unsigned long lastDbg = 0;
  if (now - lastDbg > 10000) {
    lastDbg = now;
    Serial.print("DBG ok="); Serial.print(pktOK);
    Serial.print(" dup=");   Serial.print(pktDup);
    Serial.print(" bad=");   Serial.println(pktBad);
  }
}

// =====================================================
//  PROCESAR PAQUETE → REENVIAR AL MEGA
// =====================================================
void processPacket(const Packet& pkt) {
  if (pkt.type == 'S') {
    for (int i = 0; i < 3; i++) { Serial2.println("stop"); delay(5); }
    Serial.println("FWD:stop x3");
    return;
  }

  if (pkt.type == 'M' || pkt.type == 'X') {
    // Enviar setpoints de orugas (epsilon: floats del setpoint)
    float dlr = pkt.l - pkt.r;
    if (dlr < 0.0f) dlr = -dlr;
    if (dlr < 0.05f) {
      // Misma velocidad ambas → lr=
      Serial2.print("lr="); Serial2.println(pkt.l, 1);
    } else {
      Serial2.print("l="); Serial2.println(pkt.l, 1);
      delay(3);
      Serial2.print("r="); Serial2.println(pkt.r, 1);
    }
  }

  if (pkt.type == 'X') {
    // Comando extra (PID, diag, etc.)
    String extra = String(pkt.cmd);
    extra.trim();
    if (extra.length() > 0) {
      delay(3);
      Serial2.println(extra);
      Serial.print("FWD:CMD="); Serial.println(extra);
    }
  }
}

// =====================================================
//  REENVIAR TELEMETRÍA DEL MEGA POR LORA
//  El Mega ahora envía TL: TR: RPM1..4 Err1..4 Out1..4
// =====================================================
void forwardTelemetry(const String& line) {
  // Filtrar solo líneas de telemetría real
  bool isTelem = line.indexOf("RPM1:") >= 0 ||
                 line.indexOf("TL:")   >= 0;

  if (!isTelem) {
    Serial.print("MEGA: "); Serial.println(line);
    return;
  }

  LoRa.beginPacket();
  LoRa.print(line);
  LoRa.endPacket();
}

void sendAck(uint8_t seq) {
  LoRa.beginPacket();
  LoRa.print("ACK:");
  LoRa.print(seq);
  LoRa.endPacket();
}
