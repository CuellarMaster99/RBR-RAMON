// =====================================================
//   ESP32 — TRANSMISOR NRF24L01
//   Recibe comandos de Python por Serial USB
//   Los reenvía al Arduino Mega por RF (NRF24L01)
//
//   PINOUT NRF24L01 → ESP32
//   ─────────────────────────────────────────────
//   GND  → GND
//   VCC  → 3.3V   ⚠ NO conectar a 5V
//   CE   → GPIO 4
//   CSN  → GPIO 5
//   SCK  → GPIO 18  (SPI hardware)
//   MOSI → GPIO 23  (SPI hardware)
//   MISO → GPIO 19  (SPI hardware)
//   IRQ  → Sin conectar
//
//   LIBRERÍAS (Library Manager):
//   - RF24 de TMRh20
//
//   COMPILAR COMO: ESP32 Dev Module
// =====================================================

#include <SPI.h>
#include <RF24.h>

#define CE_PIN   4
#define CSN_PIN  5

RF24 radio(CE_PIN, CSN_PIN);
const byte ADDRESS[6] = "ROBO1";   // Debe coincidir con el Mega

// Intervalo mínimo entre paquetes RF (ms)
#define RF_DELAY_MS  5

unsigned long lastRF = 0;

// Buffer de telemetría recibida del Mega (ACK payload)
// El Mega puede enviar datos de vuelta usando el pipe de ACK
char ackBuf[32] = {0};

// =====================================================
//  SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  if (!radio.begin()) {
    Serial.println("ERR:NRF24_NO_FOUND");
    while (1) { delay(1000); }
  }

  radio.openWritingPipe(ADDRESS);
  radio.setPALevel(RF24_PA_LOW);      // RF24_PA_HIGH para más alcance
  radio.setDataRate(RF24_250KBPS);    // Igual que el receptor
  radio.setChannel(108);
  radio.setRetries(5, 15);            // 5 reintentos, espera 15*250µs
  radio.stopListening();              // Modo TX

  // ACK payload: permite recibir telemetría del Mega en la respuesta ACK
  radio.enableAckPayload();

  Serial.println("OK:ESP32_TX_READY");
  Serial.println("OK:NRF24_INIT");
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  // Leer comandos de Python por Serial USB
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      sendCommand(line);
    }
  }

  // Si hay ACK payload del Mega, reenviarlo a Python por Serial
  if (radio.available()) {
    radio.read(&ackBuf, sizeof(ackBuf));
    Serial.println(String(ackBuf));
  }
}

// =====================================================
//  ENVIAR COMANDO AL MEGA POR RF
//  Empaqueta el string en 32 bytes y lo transmite
// =====================================================
void sendCommand(const String& cmd) {
  // Respetar intervalo mínimo entre envíos RF
  unsigned long now = millis();
  if (now - lastRF < RF_DELAY_MS) {
    delay(RF_DELAY_MS - (now - lastRF));
  }
  lastRF = millis();

  char buf[32] = {0};
  cmd.toCharArray(buf, sizeof(buf));

  bool ok = radio.write(&buf, sizeof(buf));

  // Responder a Python con confirmación
  if (ok) {
    Serial.print("OK:"); Serial.println(cmd);
  } else {
    Serial.print("ERR:RF_FAIL:"); Serial.println(cmd);
  }
}
