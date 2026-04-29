#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_heap_caps.h>

// -------------------- AP local --------------------
#define AP_SSID      "ESP32_CAM_AP"
#define AP_PASSWORD  "12345678"   // minimo 8 chars
#define AP_CHANNEL   6            // debe coincidir con ESP-NOW

// ------------------- ESP-NOW video ----------------
#define ESPNOW_CHANNEL       AP_CHANNEL
#define VIDEO_CHUNK_PAYLOAD  300
#define MAX_VIDEO_FRAME_SIZE 50000
#define MAX_VIDEO_CHUNKS     300

#pragma pack(push, 1)
struct VideoChunk {
  uint16_t frameId;
  uint16_t chunkId;
  uint16_t totalChunks;
  uint16_t payloadLen;
  uint8_t payload[VIDEO_CHUNK_PAYLOAD];
};
#pragma pack(pop)

WebServer server(80);
portMUX_TYPE espNowMux = portMUX_INITIALIZER_UNLOCKED;

// Buffer de ensamblado del frame actual
uint8_t* frameBuffer = nullptr;
bool chunkReceived[MAX_VIDEO_CHUNKS];
volatile uint16_t currentFrameId = 0xFFFF;
volatile uint16_t receivedChunks = 0;
volatile uint16_t expectedChunks = 0;
volatile size_t currentFrameLen = 0;
volatile bool frameReady = false;

// Ultimo frame completo listo para servir por HTTP
uint8_t* latestJpeg = nullptr;
volatile size_t latestJpegLen = 0;
volatile uint32_t latestFrameCounter = 0;

void resetFrameAssembler(uint16_t newFrameId, uint16_t totalChunks) {
  currentFrameId = newFrameId;
  expectedChunks = totalChunks;
  receivedChunks = 0;
  currentFrameLen = 0;
  frameReady = false;
  memset(chunkReceived, 0, sizeof(chunkReceived));
}

static bool isLikelyJpeg(const uint8_t* data, size_t len) {
  if (data == nullptr || len < 4) return false;
  return data[0] == 0xFF && data[1] == 0xD8 && data[len - 2] == 0xFF && data[len - 1] == 0xD9;
}

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len < (int)(sizeof(VideoChunk) - VIDEO_CHUNK_PAYLOAD)) return;

  const VideoChunk *chunk = reinterpret_cast<const VideoChunk *>(data);
  if (chunk->payloadLen > VIDEO_CHUNK_PAYLOAD) return;
  if (chunk->chunkId >= MAX_VIDEO_CHUNKS) return;
  if (chunk->totalChunks == 0 || chunk->totalChunks > MAX_VIDEO_CHUNKS) return;

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
    if (candidateLen > currentFrameLen) currentFrameLen = candidateLen;
  }

  if (expectedChunks > 0 && receivedChunks == expectedChunks) {
    frameReady = true;
  }

  portEXIT_CRITICAL_ISR(&espNowMux);
}

void handleRoot() {
  String html;
  html.reserve(512);
  html += "<!doctype html><html><head><meta charset='utf-8'><title>ESP32 ESP-NOW Cam</title></head><body>";
  html += "<h2>ESP32 ESP-NOW Cam (AP local)</h2>";
  html += "<p>Stream: <a href='/stream'>/stream</a> | Estado: <a href='/status'>/status</a></p>";
  html += "<img src='/stream' style='max-width:95vw;border:1px solid #888' />";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleStatus() {
  String s = "{";
  s += "\"frames\":" + String((unsigned long)latestFrameCounter) + ",";
  s += "\"jpeg_len\":" + String((unsigned long)latestJpegLen) + ",";
  s += "\"ap_ip\":\"" + WiFi.softAPIP().toString() + "\"";
  s += "}";
  server.send(200, "application/json", s);
}

void handleStream() {
  WiFiClient client = server.client();
  String hdr =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: close\r\n\r\n";
  client.print(hdr);

  uint32_t lastSentFrame = 0;
  // Buffer temporal por conexion HTTP (heap)
  uint8_t* outBuf = (uint8_t*)heap_caps_malloc(MAX_VIDEO_FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!outBuf) {
    outBuf = (uint8_t*)malloc(MAX_VIDEO_FRAME_SIZE);
  }
  if (!outBuf) {
    client.print("HTTP/1.1 500 Internal Server Error\r\nConnection: close\r\n\r\nNo buffer\r\n");
    return;
  }
  while (client.connected()) {
    size_t len = 0;
    uint32_t frameNo = 0;

    portENTER_CRITICAL(&espNowMux);
    len = latestJpegLen;
    frameNo = latestFrameCounter;
    if (len > 0 && len <= MAX_VIDEO_FRAME_SIZE && frameNo != lastSentFrame) {
      memcpy(outBuf, latestJpeg, len);
    } else {
      len = 0;
    }
    portEXIT_CRITICAL(&espNowMux);

    if (len > 0) {
      client.print("--frame\r\n");
      client.print("Content-Type: image/jpeg\r\n");
      client.print("Content-Length: ");
      client.print(len);
      client.print("\r\n\r\n");
      client.write(outBuf, len);
      client.print("\r\n");
      lastSentFrame = frameNo;
    } else {
      delay(15);
    }
  }
  free(outBuf);
}

void setup() {
  Serial.begin(921600);
  delay(300);
  Serial.println();
  Serial.println("=== ESPNOW_receptorAPLOCAL ===");

  // Reservar buffers grandes fuera de DRAM (.bss) para evitar overflow de linker.
  frameBuffer = (uint8_t*)heap_caps_malloc(MAX_VIDEO_FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  latestJpeg  = (uint8_t*)heap_caps_malloc(MAX_VIDEO_FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!frameBuffer || !latestJpeg) {
    // Fallback si PSRAM no esta disponible.
    if (!frameBuffer) frameBuffer = (uint8_t*)malloc(MAX_VIDEO_FRAME_SIZE);
    if (!latestJpeg)  latestJpeg  = (uint8_t*)malloc(MAX_VIDEO_FRAME_SIZE);
  }
  if (!frameBuffer || !latestJpeg) {
    Serial.println("[MEM] Error: no hay memoria para buffers JPEG");
    while (true) delay(1000);
  }

  // AP local para visualizacion desde navegador sin router externo
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  bool apOk = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false, 2);
  Serial.print("[AP] Estado: "); Serial.println(apOk ? "OK" : "FAIL");
  Serial.print("[AP] SSID: "); Serial.println(AP_SSID);
  Serial.print("[AP] IP: "); Serial.println(WiFi.softAPIP());

  // El AP ya define el canal; se deja llamado explicito por robustez.
  esp_err_t chErr = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (chErr != ESP_OK) {
    Serial.printf("[ESP-NOW] set_channel error: %d\n", chErr);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init error");
  } else {
    esp_now_register_recv_cb(onEspNowReceive);
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
      Serial.printf("[ESP-NOW] MAC RX: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    Serial.printf("[ESP-NOW] Canal: %d\n", ESPNOW_CHANNEL);
  }

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/stream", HTTP_GET, handleStream);
  server.begin();
  Serial.println("[HTTP] listo: http://192.168.4.1/");
  Serial.println("[HTTP] prueba tambien: http://192.168.4.1/status");
}

void loop() {
  server.handleClient();

  bool ready = false;
  size_t frameLenLocal = 0;
  portENTER_CRITICAL(&espNowMux);
  if (frameReady) {
    frameReady = false;
    frameLenLocal = currentFrameLen;
    ready = true;
  }
  portEXIT_CRITICAL(&espNowMux);

  if (ready && frameLenLocal > 0 && frameLenLocal <= MAX_VIDEO_FRAME_SIZE && isLikelyJpeg(frameBuffer, frameLenLocal)) {
    portENTER_CRITICAL(&espNowMux);
    memcpy(latestJpeg, frameBuffer, frameLenLocal);
    latestJpegLen = frameLenLocal;
    latestFrameCounter++;
    portEXIT_CRITICAL(&espNowMux);
  }
}
