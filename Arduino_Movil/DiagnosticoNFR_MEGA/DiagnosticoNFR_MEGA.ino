// =====================================================
//   ESP32 — Test básico nRF24  SIN ACK payload
//   Solo verifica que el ACK hardware funciona
//   Si esto funciona, el problema es enableAckPayload
//
//   COMPILAR: ESP32 Dev Module  |  BAUD: 115200
// =====================================================
#include <SPI.h>
#include <RF24.h>

#define PIN_CE   5
#define PIN_CSN  4
#define PIN_SCK  18
#define PIN_MOSI 23
#define PIN_MISO 19

SPIClass hspi(HSPI);
RF24 radio(PIN_CE, PIN_CSN);

const uint64_t PIPE_ADDR = 0xE8E8F0F0E1LL;

#pragma pack(push, 1)
struct Packet { char cmd; float left; float right; uint8_t seq; };
#pragma pack(pop)

uint8_t  seqNum  = 0;
uint32_t pktOK   = 0;
uint32_t pktFail = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== ESP32 TEST BASICO SIN ACK PAYLOAD ===");

  pinMode(PIN_CE,  OUTPUT); digitalWrite(PIN_CE,  LOW);
  pinMode(PIN_CSN, OUTPUT); digitalWrite(PIN_CSN, HIGH);
  delay(100);

  hspi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  hspi.setFrequency(2000000);   // 2 MHz
  radio.begin(&hspi);
  delay(20);

  // Configuracion MINIMA — sin enableAckPayload
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setRetries(15, 15);
  radio.setPayloadSize(sizeof(Packet));
  radio.openWritingPipe(PIPE_ADDR);
  radio.stopListening();

  bool conn = radio.isChipConnected();
  Serial.print("isChipConnected(): "); Serial.println(conn ? "SI" : "NO");
  Serial.println("Transmitiendo cada 500ms...");
  Serial.println("Si ves [TX OK] el ACK hardware funciona");
  Serial.println("Si ves [TX FAIL] hay problema de enlace RF");
  Serial.println();
}

void loop() {
  Packet pkt = {'T', 50.0f, 50.0f, seqNum++};

  bool ok = radio.write(&pkt, sizeof(pkt));

  if (ok) {
    pktOK++;
    Serial.print("[TX OK]   seq="); Serial.print(pkt.seq);
    Serial.print("  ok="); Serial.print(pktOK);
    Serial.print("  fail="); Serial.println(pktFail);
  } else {
    pktFail++;
    Serial.print("[TX FAIL] seq="); Serial.print(pkt.seq);
    Serial.print("  ok="); Serial.print(pktOK);
    Serial.print("  fail="); Serial.println(pktFail);
  }

  delay(500);
}
