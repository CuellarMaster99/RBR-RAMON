// =====================================================
//   ARDUINO MEGA — Test básico nRF24  SIN ACK payload
//   Par del esp32_test_basico.ino
//
//   COMPILAR: Arduino Mega or Mega 2560  |  BAUD: 115200
// =====================================================
#include <SPI.h>
#include <RF24.h>

#define PIN_CE   48
#define PIN_CSN  49

RF24 radio(PIN_CE, PIN_CSN);
const uint64_t PIPE_ADDR = 0xE8E8F0F0E1LL;

#pragma pack(push, 1)
struct Packet { char cmd; float left; float right; uint8_t seq; };
#pragma pack(pop)

uint32_t pktTotal = 0;
unsigned long lastPktTime = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== MEGA TEST BASICO SIN ACK PAYLOAD ===");

  pinMode(PIN_CE,  OUTPUT); digitalWrite(PIN_CE,  LOW);
  pinMode(PIN_CSN, OUTPUT); digitalWrite(PIN_CSN, HIGH);
  SPI.begin();
  delay(100);

  // Escribir CONFIG=0x08 para reset limpio
  digitalWrite(PIN_CSN, LOW);
  SPI.transfer(0x20); SPI.transfer(0x08);
  digitalWrite(PIN_CSN, HIGH);
  SPI.end();
  delay(500);

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  radio.begin();
  SPI.endTransaction();
  delay(20);

  // Configuracion MINIMA — sin enableAckPayload
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setRetries(15, 15);
  radio.setPayloadSize(sizeof(Packet));
  radio.openReadingPipe(1, PIPE_ADDR);
  radio.startListening();

  Serial.print("isChipConnected(): "); Serial.println(radio.isChipConnected() ? "SI" : "NO");
  Serial.println("Escuchando paquetes de la ESP32...");
  Serial.println("[RX] = paquete recibido (ACK automatico HW enviado)");
  Serial.println();
  lastPktTime = millis();
}

void loop() {
  if (radio.available()) {
    Packet pkt;
    radio.read(&pkt, sizeof(pkt));
    pktTotal++;
    lastPktTime = millis();

    Serial.print("[RX] seq="); Serial.print(pkt.seq);
    Serial.print("  cmd=");    Serial.print(pkt.cmd);
    Serial.print("  L=");      Serial.print(pkt.left,  1);
    Serial.print("  R=");      Serial.print(pkt.right, 1);
    Serial.print("  total=");  Serial.println(pktTotal);
  }

  if (millis() - lastPktTime > 5000) {
    lastPktTime = millis();
    Serial.print("[WAIT] Esperando... recibidos: "); Serial.println(pktTotal);
  }
}
