/*
  ESP32 Gateway basico: ESP-NOW RX de video -> Serial (PC)
  ---------------------------------------------------------
  - Recibe video fragmentado por ESP-NOW desde ESP32-S3 + OV2640.
  - Envia cada frame por Serial con formato:
      "FRAM" + uint32(len) + jpeg bytes
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>

// -------------------------- ESP-NOW ---------------------------
#define ESPNOW_CHANNEL 6
#define VIDEO_CHUNK_PAYLOAD 200
#define MAX_VIDEO_FRAME_SIZE 50000
#define MAX_VIDEO_CHUNKS 300
// Protocolo serial binario para PC: "FRAM" + uint32_t len + jpeg bytes
static const uint8_t SERIAL_FRAME_MAGIC[4] = {'F', 'R', 'A', 'M'};
#define SERIAL_BINARY_ONLY 1

#pragma pack(push, 1)
struct VideoChunk {
  uint16_t frameId;
  uint16_t chunkId;
  uint16_t totalChunks;
  uint16_t payloadLen;
  uint8_t payload[VIDEO_CHUNK_PAYLOAD];
};
#pragma pack(pop)

// ---------------------- Estado de frame -----------------------
uint8_t frameBuffer[MAX_VIDEO_FRAME_SIZE];
bool chunkReceived[MAX_VIDEO_CHUNKS];

volatile uint16_t currentFrameId = 0xFFFF;
volatile uint16_t receivedChunks = 0;
volatile uint16_t expectedChunks = 0;
volatile size_t currentFrameLen = 0;
volatile bool frameReady = false;

portMUX_TYPE espNowMux = portMUX_INITIALIZER_UNLOCKED;

// ------------------------- Utilidades -------------------------
void resetFrameAssembler(uint16_t newFrameId, uint16_t totalChunks) {
  currentFrameId = newFrameId;
  expectedChunks = totalChunks;
  receivedChunks = 0;
  currentFrameLen = 0;
  frameReady = false;
  memset(chunkReceived, 0, sizeof(chunkReceived));
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);

  esp_err_t chErr = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (chErr != ESP_OK) {
#if !SERIAL_BINARY_ONLY
    Serial.printf("[ESP-NOW] Error set_channel: %d\n", chErr);
#endif
  }

  esp_err_t initErr = esp_now_init();
  if (initErr != ESP_OK) {
#if !SERIAL_BINARY_ONLY
    Serial.printf("[ESP-NOW] Error init: %d\n", initErr);
#endif
    return;
  }

  uint8_t mac[6] = {0};
  esp_err_t macErr = esp_read_mac(mac, ESP_MAC_WIFI_STA);
  if (macErr == ESP_OK) {
#if !SERIAL_BINARY_ONLY
    Serial.printf("[ESP-NOW] MAC receptor: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("[ESP-NOW] Pegar en TX: {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
  } else {
#if !SERIAL_BINARY_ONLY
    Serial.printf("[ESP-NOW] Error leyendo MAC: %d\n", macErr);
    Serial.print("[ESP-NOW] Fallback MAC: ");
    Serial.println(WiFi.macAddress());
#endif
  }
#if !SERIAL_BINARY_ONLY
  Serial.printf("[ESP-NOW] Canal: %d\n", ESPNOW_CHANNEL);
#endif
}

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;

  if (len < (int)(sizeof(VideoChunk) - VIDEO_CHUNK_PAYLOAD)) return;

  const VideoChunk *chunk = reinterpret_cast<const VideoChunk *>(data);
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

static bool isLikelyJpeg(const uint8_t* data, size_t len) {
  if (data == nullptr || len < 4) return false;
  // JPEG should start with SOI and end with EOI markers.
  return data[0] == 0xFF && data[1] == 0xD8 && data[len - 2] == 0xFF && data[len - 1] == 0xD9;
}

void publishFrameStatusToPc() {
  static uint16_t lastPrintedFrameId = 0xFFFF;
  static unsigned long lastStatsMs = 0;
  static uint32_t frameCounterPc = 0;

  bool readyLocal = false;
  uint16_t frameIdLocal = 0;
  size_t frameLenLocal = 0;
  uint16_t chunksLocal = 0;

  portENTER_CRITICAL(&espNowMux);
  if (frameReady && currentFrameId != lastPrintedFrameId) {
    readyLocal = true;
    frameIdLocal = currentFrameId;
    frameLenLocal = currentFrameLen;
    chunksLocal = expectedChunks;
    frameReady = false;  // marca consumido
  }
  portEXIT_CRITICAL(&espNowMux);

  if (!readyLocal) return;
  if (!isLikelyJpeg(frameBuffer, frameLenLocal)) return;

  // Stream binario para visor Python:
  // [4 bytes magic "FRAM"][4 bytes len little-endian][jpeg bytes]
  uint32_t len32 = (uint32_t)frameLenLocal;
  Serial.write(SERIAL_FRAME_MAGIC, sizeof(SERIAL_FRAME_MAGIC));
  Serial.write((const uint8_t *)&len32, sizeof(len32));
  Serial.write(frameBuffer, frameLenLocal);
  frameCounterPc++;

  // Mantener logs de estado de forma limitada para no saturar el puerto serial.
#if !SERIAL_BINARY_ONLY
  unsigned long now = millis();
  if (now - lastStatsMs >= 1000) {
    Serial.printf("[VIDEO] frame=%u bytes=%u chunks=%u fps_out=%u\n",
                  frameIdLocal, (unsigned)frameLenLocal, chunksLocal, (unsigned)frameCounterPc);
    frameCounterPc = 0;
    lastStatsMs = now;
  }
#endif

  lastPrintedFrameId = frameIdLocal;
}

void setup() {
  Serial.begin(115200);
  delay(500);
#if !SERIAL_BINARY_ONLY
  Serial.println();
  Serial.println("=== ESP32 Gateway LoRa + ESP-NOW ===");
#endif

  setupEspNow();
  esp_now_register_recv_cb(onEspNowReceive);
}

void loop() {
  publishFrameStatusToPc();
  delay(2);
}
