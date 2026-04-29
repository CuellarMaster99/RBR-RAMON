/*
  ESP32-S3 + OV2640: TX de video por ESP-NOW
  ------------------------------------------------------------
  Objetivo:
  - Capturar JPEG desde OV2640.
  - Fragmentar en paquetes ESP-NOW.
  - Enviar a ESP32 gateway (LoRa + ESP-NOW RX).

  IMPORTANTE:
  - Ajusta el pinout de camara segun tu placa ESP32-S3 exacta.
  - Ajusta GATEWAY_MAC al MAC de la ESP32 receptora.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include "esp_camera.h"

// --------------------- Ajustes ESP-NOW ------------------------
#define ESPNOW_CHANNEL 6
#define VIDEO_CHUNK_PAYLOAD 200
// 1 = prueba solo captura de camara (sin ESP-NOW)
#define CAMERA_DIAGNOSTIC_ONLY 0

// Reemplazar con MAC de la ESP32 gateway
uint8_t GATEWAY_MAC[6] = {0x94, 0x54, 0xC5, 0xB2, 0x55, 0xD0};

#pragma pack(push, 1)
struct VideoChunk {
  uint16_t frameId;
  uint16_t chunkId;
  uint16_t totalChunks;
  uint16_t payloadLen;
  uint8_t payload[VIDEO_CHUNK_PAYLOAD];
};
#pragma pack(pop)

volatile bool espNowReady = false;
uint16_t frameCounter = 0;
volatile bool sendCallbackDone = true;

// --------------------- Pinout de ejemplo ----------------------
// Este pinout es orientativo; debes ajustarlo a tu modulo S3 camara.
// Si usas un board conocido (XIAO ESP32S3 Sense, ESP32-S3-EYE, etc.),
// usa sus pines oficiales.
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13

bool setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // Modo conservador para ESP-NOW:
  // menos bytes por frame => mas probabilidad de recepcion completa.
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  bool hasPsram = psramFound();
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init error: 0x%x\n", err);
    // Fallback mas conservador
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("[CAM] Init fallback error: 0x%x\n", err);
      return false;
    }
    Serial.println("[CAM] Inicializada con fallback.");
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);
  }

  Serial.printf("[CAM] Inicializada. PSRAM=%s\n", hasPsram ? "SI" : "NO");
  return true;
}

void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status){
  (void)info;
  (void)status;
  sendCallbackDone = true;
}

bool setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  esp_wifi_set_ps(WIFI_PS_NONE);  // evita latencias por power-save

  esp_err_t chErr = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (chErr != ESP_OK) {
    Serial.printf("[ESP-NOW] set_channel error: %d\n", chErr);
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init error.");
    return false;
  }

  esp_now_register_send_cb(onEspNowSent);



  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, GATEWAY_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW] Error add_peer.");
    return false;
  }

  Serial.print("[ESP-NOW] MAC TX: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("[ESP-NOW] MAC gateway destino: %02X:%02X:%02X:%02X:%02X:%02X\n",
                GATEWAY_MAC[0], GATEWAY_MAC[1], GATEWAY_MAC[2],
                GATEWAY_MAC[3], GATEWAY_MAC[4], GATEWAY_MAC[5]);
  Serial.printf("[ESP-NOW] Canal: %d\n", ESPNOW_CHANNEL);
  espNowReady = true;
  return true;
}

void sendFrameByEspNow(const uint8_t *jpeg, size_t jpegLen, uint16_t frameId) {
  if (!espNowReady || jpeg == nullptr || jpegLen == 0) return;

  uint16_t totalChunks = (jpegLen + VIDEO_CHUNK_PAYLOAD - 1) / VIDEO_CHUNK_PAYLOAD;
  if (totalChunks == 0) return;

  VideoChunk chunk = {};
  chunk.frameId = frameId;
  chunk.totalChunks = totalChunks;

  for (uint16_t i = 0; i < totalChunks; i++) {
    size_t offset = (size_t)i * VIDEO_CHUNK_PAYLOAD;
    size_t remaining = jpegLen - offset;
    size_t thisLen = remaining > VIDEO_CHUNK_PAYLOAD ? VIDEO_CHUNK_PAYLOAD : remaining;

    chunk.chunkId = i;
    chunk.payloadLen = (uint16_t)thisLen;
    memcpy(chunk.payload, jpeg + offset, thisLen);

    bool sent = false;
    int attempt = 0;
    while (!sent && attempt < 20) {
      attempt++;
      sendCallbackDone = false;
      esp_err_t err = esp_now_send(GATEWAY_MAC, (uint8_t *)&chunk, sizeof(VideoChunk));
      if (err == ESP_OK) {
        // Espera a liberar cola TX para evitar NO_MEM en el siguiente chunk.
        unsigned long t0 = millis();
        while (!sendCallbackDone && (millis() - t0) < 50) {
          delay(1);
        }
        sent = true;
      } else if (err == ESP_ERR_ESPNOW_NO_MEM) {
        // Backoff fuerte para que el driver libere cola interna.
        delay(20);
      } else {
        Serial.printf("[ESP-NOW] Send err=%d (%s) chunk=%u try=%d\n",
                      err, esp_err_to_name(err), i, attempt);
        delay(10);
      }
    }
    if (!sent) {
      Serial.printf("[ESP-NOW] Drop chunk=%u frame=%u\n", i, (unsigned)chunk.frameId);
    }
    delay(8);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== ESP32-S3 CAM ESP-NOW TX ===");

  if (!setupCamera()) {
    Serial.println("Fallo camara. Revisa pinout.");
    while (true) delay(1000);
  }

#if !CAMERA_DIAGNOSTIC_ONLY
  if (!setupEspNow()) {
    Serial.println("Fallo ESP-NOW.");
    while (true) delay(1000);
  }
#else
  Serial.println("[DIAG] CAMERA_DIAGNOSTIC_ONLY=1 (ESP-NOW deshabilitado)");
#endif
}

void loop() {
  static uint16_t fbNullCount = 0;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CAM] fb null");
    fbNullCount++;
    if (fbNullCount >= 20) {
      Serial.println("[CAM] Reiniciando driver de camara por fb null repetido...");
      esp_camera_deinit();
      delay(200);
      if (!setupCamera()) {
        Serial.println("[CAM] Reinit fallo.");
      }
      fbNullCount = 0;
    }
    delay(60);
    return;
  }
  fbNullCount = 0;

  uint16_t frameId = frameCounter;
  Serial.printf("[TX] frame=%u bytes=%u\n", (unsigned)frameId, (unsigned)fb->len);
#if !CAMERA_DIAGNOSTIC_ONLY
  sendFrameByEspNow(fb->buf, fb->len, frameId);
#endif
  frameCounter++;

  esp_camera_fb_return(fb);
  delay(140);
}
