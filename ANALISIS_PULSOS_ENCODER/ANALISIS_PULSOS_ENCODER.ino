// ===============================
// MEDICIÓN REAL DE PPR - ESP32
// ===============================

#define ENCODER_A 34
#define ENCODER_B 35

volatile long encoderCount = 0;

// ========= INTERRUPCIÓN =========
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_B))
    encoderCount++;
  else
    encoderCount--;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  Serial.println("=================================");
  Serial.println(" MEDICION REAL DE PPR ");
  Serial.println("=================================");
  Serial.println("Comandos:");
  Serial.println(" r  -> resetear contador");
  Serial.println(" s  -> mostrar valor actual");
  Serial.println("=================================");
}

void loop() {

  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'r') {
      encoderCount = 0;
      Serial.println("Contador reiniciado.");
    }

    if (cmd == 's') {
      Serial.print("Pulsos contados: ");
      Serial.println(abs(encoderCount));
    }
  }
}
