// =====================================================
//   ESP32 ANTENA — LoRa TX  (4 orugas) + ESP-NOW video RX
//   - LoRa: igual que Antena: Serial USB <-> base por LoRa
//   - Video: recibe fragmentos por ESP-NOW (mismo TX S3) y
//     envia por el MISMO USB serial: "FRAM" + uint32 len + JPEG
//     (el PC debe parsear lineas de texto + bloques binarios; ver Control_LoRa_Cam o viewer)
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
//   COMPILAR: ESP32 Dev Module  |  USB: 115200 (o 921600 si mezclas mucho video)
// =====================================================

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include "esp_heap_caps.h"

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

// ——— ESP-NOW (mismo protocolo que esp32s3_cam_espnow_tx) ———
// Debe coincidir: canal y VIDEO_CHUNK_PAYLOAD con el emisor
#define ESPNOW_CHANNEL        6
#define VIDEO_CHUNK_PAYLOAD   200
#define MAX_VIDEO_FRAME_SIZE  50000
#define MAX_VIDEO_CHUNKS      300
// Sin texto extra del modulo video (solo FRAM+JPEG); LoRa sigue mostrando OK/TL/ERR/telemetría
#define ESPNOW_SERIAL_BINARY_ONLY 1

static const uint8_t SERIAL_FRAME_MAGIC[4] = { 'F', 'R', 'A', 'M' };

#pragma pack(push, 1)
struct VideoChunk {
  uint16_t frameId;
  uint16_t chunkId;
  uint16_t totalChunks;
  uint16_t payloadLen;
  uint8_t  payload[VIDEO_CHUNK_PAYLOAD];
};
#pragma pack(pop)

static uint8_t* frameBuffer = nullptr;
static bool chunkReceived[MAX_VIDEO_CHUNKS];

volatile uint16_t currentFrameId = 0xFFFF;
volatile uint16_t receivedChunks = 0;
volatile uint16_t expectedChunks = 0;
volatile size_t   currentFrameLen = 0;
volatile bool     frameReady = false;

portMUX_TYPE espNowMux = portMUX_INITIALIZER_UNLOCKED;
static bool espnowOk = false;

// ── Estructura de paquete — idéntica en ambas ESP32 ──
#pragma pack(push, 1)
struct Packet {
  uint8_t seq;
  uint8_t type;    // 'M'=orugas  'X'=cmd extra  'S'=stop
  float   l;       // oruga izquierda  (-150 a +150 RPM)
  float   r;       // oruga derecha    (-150 a +150 RPM)
  char    cmd[18];
};
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

// ------------------ ESP-NOW ensamblado de frames ------------------
void resetFrameAssembler(uint16_t newFrameId, uint16_t totalChunks) {
  currentFrameId   = newFrameId;
  expectedChunks  = totalChunks;
  receivedChunks  = 0;
  currentFrameLen = 0;
  frameReady      = false;
  memset(chunkReceived, 0, sizeof(chunkReceived));
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);

  esp_err_t chErr = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  (void)chErr;
#if !ESPNOW_SERIAL_BINARY_ONLY
  if (chErr != ESP_OK) {
    Serial.printf("[ESP-NOW] Error set_channel: %d\n", chErr);
  }
#endif

  if (esp_now_init() != ESP_OK) {
#if !ESPNOW_SERIAL_BINARY_ONLY
    Serial.println("[ESP-NOW] esp_now_init fail");
#endif
    return;
  }

  uint8_t mac[6] = {0};
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
#if !ESPNOW_SERIAL_BINARY_ONLY
    Serial.printf("OK:ESPNOW_MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
  }
#if !ESPNOW_SERIAL_BINARY_ONLY
  else {
    Serial.print("OK:ESPNOW_MAC ");
    Serial.println(WiFi.macAddress());
  }
  Serial.printf("OK:ESPNOW_CH %d\n", ESPNOW_CHANNEL);
#endif
  espnowOk = true;
}

void onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;
  if (!frameBuffer) return;
  if (len < (int)(sizeof(VideoChunk) - VIDEO_CHUNK_PAYLOAD)) return;

  const VideoChunk* chunk = reinterpret_cast<const VideoChunk*>(data);
  if (chunk->payloadLen > VIDEO_CHUNK_PAYLOAD) return;
  if (chunk->chunkId >= MAX_VIDEO_CHUNKS) return;
  if (chunk->totalChunks > MAX_VIDEO_CHUNKS) return;

  size_t offset = (size_t)chunk->chunkId * VIDEO_CHUNK_PAYLOAD;
  if (offset + chunk->payloadLen > MAX_VIDEO_FRAME_SIZE) return;

  portENTER_CRITICAL_ISR(&espNowMux);

  if (currentFrameId != chunk->frameId) {
    resetFrameAssembler(chunk->frameId, chunk->totalChunks);
  }

  if (!chunkReceived[chunk->chunkId]) {
    memcpy(&frameBuffer[offset], chunk->payload, chunk->payloadLen);
    chunkReceived[chunk->chunkId] = true;
    receivedChunks++;

    size_t candidateLen = offset + chunk->payloadLen;
    if (candidateLen > currentFrameLen) {
      currentFrameLen = candidateLen;
    }
  }

  if (expectedChunks > 0 && receivedChunks == expectedChunks) {
    frameReady = true;
  }

  portEXIT_CRITICAL_ISR(&espNowMux);
}

static bool isLikelyJpeg(const uint8_t* data, size_t l) {
  if (data == nullptr || l < 4) return false;
  return data[0] == 0xFF && data[1] == 0xD8 && data[l - 2] == 0xFF && data[l - 1] == 0xD9;
}

void publishFrameStatusToPc() {
  if (!frameBuffer) return;

  static uint16_t lastPrintedFrameId = 0xFFFF;
  static unsigned long lastStatsMs = 0;
  static uint32_t frameCounterPc = 0;

  bool  readyLocal = false;
  uint16_t frameIdLocal = 0;
  size_t  frameLenLocal = 0;
  uint16_t chunksLocal  = 0;

  portENTER_CRITICAL(&espNowMux);
  if (frameReady && currentFrameId != lastPrintedFrameId) {
    readyLocal     = true;
    frameIdLocal   = currentFrameId;
    frameLenLocal  = currentFrameLen;
    chunksLocal    = expectedChunks;
    frameReady     = false;
  }
  portEXIT_CRITICAL(&espNowMux);

  if (!readyLocal) return;
  if (!isLikelyJpeg(frameBuffer, frameLenLocal)) return;

  uint32_t len32 = (uint32_t)frameLenLocal;
  Serial.write(SERIAL_FRAME_MAGIC, sizeof(SERIAL_FRAME_MAGIC));
  Serial.write((const uint8_t*)&len32, sizeof(len32));
  Serial.write(frameBuffer, frameLenLocal);
  frameCounterPc++;
  lastPrintedFrameId = frameIdLocal;

#if !ESPNOW_SERIAL_BINARY_ONLY
  unsigned long now = millis();
  if (now - lastStatsMs >= 1000) {
    Serial.printf("[VIDEO] frame=%u bytes=%u chunks=%u fps=%u\n",
                  frameIdLocal, (unsigned)frameLenLocal, chunksLocal, (unsigned)frameCounterPc);
    frameCounterPc = 0;
    lastStatsMs    = now;
  }
#endif
}

// =====================================================
//  SETUP
// =====================================================
void setup() {
  Serial.begin(921600);
  Serial.setTimeout(5);
  delay(500);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  ESP32 ANTENA — LoRa TX  (4 orugas)");
  Serial.println("  + ESP-NOW video (FRAM+JPEG por serial)");
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

  // Buffer de video: PSRAM si hay; si no, RAM interna
  const size_t fbSize = (size_t)MAX_VIDEO_FRAME_SIZE;
  if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) > fbSize + 10000) {
    frameBuffer = (uint8_t*)heap_caps_malloc(fbSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!frameBuffer) {
    frameBuffer = (uint8_t*)malloc(fbSize);
  }
  if (frameBuffer) {
    setupEspNow();
    if (espnowOk) {
      esp_now_register_recv_cb(onEspNowReceive);
      Serial.println("OK:ESPNOW_RX");
    } else {
      Serial.println("ERR:ESPNOW_INIT (solo LoRa)");
    }
  } else {
    Serial.println("ERR:VIDEO_BUF (solo LoRa)");
  }
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  int pktSize = LoRa.parsePacket();
  if (pktSize > 0) {
    uint8_t buf[128] = {0};
    int len = 0;
    while (LoRa.available() && len < (int)sizeof(buf) - 1) {
      buf[len++] = LoRa.read();
    }
    buf[len] = 0;

    String msg = String((char*)buf);
    msg.trim();
    if (msg.length() > 0) {
      if (msg.startsWith("ACK:")) {
        lastAck = now;
        waitingAck = false;
      } else {
        Serial.println(msg);
      }
    }
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialLineBuf.trim();
      if (serialLineBuf.length() > 0) {
        parseCommand(serialLineBuf);
      }
      serialLineBuf = "";
    } else {
      if (serialLineBuf.length() < 96) {
        serialLineBuf += c;
      } else {
        serialLineBuf = "";
      }
    }
  }

  if (now - lastSend >= KEEPALIVE_MS) {
    lastSend = now;
    sendPacket();
  }

  if (waitingAck && now - lastAck > ACK_TIMEOUT_MS) {
    waitingAck = false;
    Serial.println("ERR:LORA_TIMEOUT");
  }

  publishFrameStatusToPc();
}

// =====================================================
//  PARSEAR COMANDO DE PYTHON
// =====================================================
void parseCommand(const String& raw) {
  String cmd = raw;
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "forward") {
    targetL = 80.0f; targetR = 80.0f;
    sendPacket();
    return;
  }
  if (cmd == "back") {
    targetL = -80.0f; targetR = -80.0f;
    sendPacket();
    return;
  }
  if (cmd == "left") {
    targetL = -80.0f; targetR = 80.0f;
    sendPacket();
    return;
  }
  if (cmd == "right") {
    targetL = 80.0f; targetR = -80.0f;
    sendPacket();
    return;
  }

  if (cmd.startsWith("l=")) {
    targetL = constrain(cmd.substring(2).toFloat(), -150.0f, 150.0f);
    sendPacket();
    return;
  }
  if (cmd.startsWith("r=")) {
    targetR = constrain(cmd.substring(2).toFloat(), -150.0f, 150.0f);
    sendPacket();
    return;
  }
  if (cmd.startsWith("lr=")) {
    float v = constrain(cmd.substring(3).toFloat(), -150.0f, 150.0f);
    targetL = v;
    targetR = v;
    sendPacket();
    return;
  }
  if (cmd == "stop") {
    targetL = 0;
    targetR = 0;
    for (int i = 0; i < 3; i++) {
      sendStop();
      delay(5);
    }
    Serial.println("OK:stop");
    return;
  }

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
    strncpy(pkt.cmd, pendingCmdStr, sizeof(pkt.cmd) - 1);
    pendingCmd = false;
    memset(pendingCmdStr, 0, sizeof(pendingCmdStr));
  } else {
    pkt.type = 'M';
  }

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();

  waitingAck  = true;
  lastSend    = millis();

  Serial.print("OK:L=");
  Serial.print(pkt.l, 0);
  Serial.print(" R=");
  Serial.println(pkt.r, 0);
}

// =====================================================
//  STOP EXPLÍCITO
// =====================================================
void sendStop() {
  Packet pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.seq  = seqNum++;
  pkt.type = 'S';
  pkt.l    = 0;
  pkt.r    = 0;
  strncpy(pkt.cmd, "stop", sizeof(pkt.cmd) - 1);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();
}
