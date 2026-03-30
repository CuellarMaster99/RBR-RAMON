// =====================================================
//   ESP32 CONTROL DOBLE MOTOR + PID + SYNC + DATA
// =====================================================

// ====== PROTOTIPOS ISR ======
void IRAM_ATTR encoder1ISR();
void IRAM_ATTR encoder2ISR();

// ====== MOTOR 1 ======
#define ENA 25
#define IN1 26
#define IN2 27
#define ENCODER1_A 34
#define ENCODER1_B 35

// ====== MOTOR 2 ======
#define ENB 33
#define IN3 14
#define IN4 12
#define ENCODER2_A 32
#define ENCODER2_B 13

// ====== PWM CONFIG ======
#define PWM_FREQ 31000
#define PWM_RESOLUTION 8

// ====== ENCODERS ======
volatile long encoderPos1 = 0;
volatile long encoderPos2 = 0;

long lastEncoderPos1 = 0;
long lastEncoderPos2 = 0;

const int PPR = 234;

// ====== TIEMPO ======
unsigned long lastTime = 0;

// ====== VELOCIDADES ======
float rpm1 = 0, rpmFiltrada1 = 0, targetRPM1 = 0;
float rpm2 = 0, rpmFiltrada2 = 0, targetRPM2 = 0;

// ====== PID MOTOR 1 ======
float Kp1 = 1.35, Ki1 = 0.500, Kd1 = 0.038;
float error1 = 0, lastError1 = 0, integral1 = 0, output1 = 0;

// ====== PID MOTOR 2 ======
float Kp2 = 1.35, Ki2 = 0.545, Kd2 = 0.028;
float error2 = 0, lastError2 = 0, integral2 = 0, output2 = 0;

// ====== MODO SINCRONIZACIÓN ======
bool syncMode = false;

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);
  delay(2000);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENCODER1_A, INPUT);
  pinMode(ENCODER1_B, INPUT);
  pinMode(ENCODER2_A, INPUT);
  pinMode(ENCODER2_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2ISR, RISING);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  stopMotor1();
  stopMotor2();

  lastTime = millis();

  Serial.println("Sistema listo.");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  if (millis() - lastTime >= 20) {
    calcularRPM();
    pidControl();
    imprimirDatos();
  }

  if (Serial.available()) processSerialCommand();
}

// =====================================================
// CALCULAR RPM
// =====================================================
void calcularRPM() {

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  if (deltaTime <= 0) return;

  long pos1, pos2;

  noInterrupts();
  pos1 = encoderPos1;
  pos2 = encoderPos2;
  interrupts();

  long delta1 = pos1 - lastEncoderPos1;
  long delta2 = pos2 - lastEncoderPos2;

  lastEncoderPos1 = pos1;
  lastEncoderPos2 = pos2;

  rpm1 = (delta1 * 60.0) / (PPR * deltaTime);
  rpm2 = (delta2 * 60.0) / (PPR * deltaTime);

  rpm1 = fabs(rpm1);
  rpm2 = fabs(rpm2);

  rpmFiltrada1 = 0.7 * rpmFiltrada1 + 0.3 * rpm1;
  rpmFiltrada2 = 0.7 * rpmFiltrada2 + 0.3 * rpm2;

  rpm1 = rpmFiltrada1;
  rpm2 = rpmFiltrada2;

  lastTime = currentTime;
}

// =====================================================
// PID CONTROL
// =====================================================
void pidControl() {

  float dt = 0.02;

  // ---- Motor 1 ----
  error1 = targetRPM1 - rpm1;
  integral1 += error1 * dt;
  integral1 = constrain(integral1, -400, 400);
  float derivative1 = (error1 - lastError1) / dt;

  output1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;
  output1 = constrain(output1, -255, 255);

  setMotor1(output1);
  lastError1 = error1;

  // ---- Motor 2 ----
  error2 = targetRPM2 - rpm2;
  integral2 += error2 * dt;
  integral2 = constrain(integral2, -400, 400);
  float derivative2 = (error2 - lastError2) / dt;

  output2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;
  output2 = constrain(output2, -255, 255);

  // ===== SINCRONIZACIÓN =====
  if (syncMode) {
    float syncError = rpm1 - rpm2;
    output2 += syncError * 0.5;  // ajuste fino
  }

  setMotor2(output2);
  lastError2 = error2;
}

// =====================================================
// IMPRIMIR DATA ESTRUCTURADA
// =====================================================
void imprimirDatos() {

  Serial.print("DATA,");
  Serial.print(rpm1); Serial.print(",");
  Serial.print(rpm2); Serial.print(",");
  Serial.print(targetRPM1); Serial.print(",");
  Serial.print(targetRPM2); Serial.print(",");
  Serial.print(output1); Serial.print(",");
  Serial.print(output2); Serial.print(",");
  Serial.print(Kp1); Serial.print(",");
  Serial.print(Ki1); Serial.print(",");
  Serial.print(Kd1); Serial.print(",");
  Serial.print(Kp2); Serial.print(",");
  Serial.print(Ki2); Serial.print(",");
  Serial.println(Kd2);
}

// =====================================================
// CONTROL MOTORES
// =====================================================
void setMotor1(float speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, abs(speed));
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(ENA, abs(speed));
  } else stopMotor1();
}

void stopMotor1() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, 0);
  integral1 = 0;
}

void setMotor2(float speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, abs(speed));
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(ENB, abs(speed));
  } else stopMotor2();
}

void stopMotor2() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, 0);
  integral2 = 0;
}

// =====================================================
// ISR ENCODERS
// =====================================================
void IRAM_ATTR encoder1ISR() {
  if (digitalRead(ENCODER1_B)) encoderPos1++;
  else encoderPos1--;
}

void IRAM_ATTR encoder2ISR() {
  if (digitalRead(ENCODER2_B)) encoderPos2++;
  else encoderPos2--;
}

// =====================================================
// SERIAL COMMANDS
// =====================================================
// =====================================================
// SERIAL COMMANDS (MEJORADO PARA DEBUG)
// =====================================================
void processSerialCommand() {

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  Serial.print("CMD_RECIBIDO:");
  Serial.println(cmd);   // 🔥 Debug visible en Python

  // ================= MOTOR 1 =================
  if (cmd.startsWith("m1=")) {
    targetRPM1 = cmd.substring(3).toFloat();
    Serial.println("ACK_M1");
  }

  // ================= MOTOR 2 =================
  else if (cmd.startsWith("m2=")) {
    targetRPM2 = cmd.substring(3).toFloat();
    Serial.println("ACK_M2");
  }

  // ================= AMBOS =================
  else if (cmd.startsWith("m12=")) {
    float val = cmd.substring(4).toFloat();
    targetRPM1 = val;
    targetRPM2 = val;
    Serial.println("ACK_M12");
  }

  // ================= PID MOTOR 1 =================
  else if (cmd.startsWith("kp1=")) {
    Kp1 = cmd.substring(4).toFloat();
    Serial.println("ACK_KP1");
  }

  else if (cmd.startsWith("ki1=")) {
    Ki1 = cmd.substring(4).toFloat();
    Serial.println("ACK_KI1");
  }

  else if (cmd.startsWith("kd1=")) {
    Kd1 = cmd.substring(4).toFloat();
    Serial.println("ACK_KD1");
  }

  // ================= PID MOTOR 2 =================
  else if (cmd.startsWith("kp2=")) {
    Kp2 = cmd.substring(4).toFloat();
    Serial.println("ACK_KP2");
  }

  else if (cmd.startsWith("ki2=")) {
    Ki2 = cmd.substring(4).toFloat();
    Serial.println("ACK_KI2");
  }

  else if (cmd.startsWith("kd2=")) {
    Kd2 = cmd.substring(4).toFloat();
    Serial.println("ACK_KD2");
  }

  // ================= SYNC =================
  else if (cmd == "sync=1") {
    syncMode = true;
    Serial.println("SYNC_ON");
  }

  else if (cmd == "sync=0") {
    syncMode = false;
    Serial.println("SYNC_OFF");
  }

  // ================= STOP =================
  else if (cmd == "stop1") {
    targetRPM1 = 0;
    stopMotor1();
    Serial.println("STOP1_OK");
  }

  else if (cmd == "stop2") {
    targetRPM2 = 0;
    stopMotor2();
    Serial.println("STOP2_OK");
  }

  // ================= PING TEST =================
  else if (cmd == "ping") {
    Serial.println("ESP32_OK");
  }

  else {
    Serial.println("CMD_DESCONOCIDO");
  }
}
