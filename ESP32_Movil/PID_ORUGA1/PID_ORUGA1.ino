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

const int PPR = 234;   // 🔥 VALOR REAL MEDIDO

// ====== TIEMPO ======
unsigned long lastTime = 0;

// ====== VELOCIDADES ======
float rpm1 = 0, rpmFiltrada1 = 0, targetRPM1 = 0;
float rpm2 = 0, rpmFiltrada2 = 0, targetRPM2 = 0;

// ====== PID MOTOR 1 ======
float Kp1 = 1.2, Ki1 = 0.4, Kd1 = 0.03;
float error1 = 0, lastError1 = 0, integral1 = 0, output1 = 0;

// ====== PID MOTOR 2 ======
float Kp2 = 1.2, Ki2 = 0.4, Kd2 = 0.03;
float error2 = 0, lastError2 = 0, integral2 = 0, output2 = 0;

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

  // PWM moderno ESP32
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  stopMotor1();
  stopMotor2();

  lastTime = millis();

  Serial.println("Sistema iniciado...");
}

void loop() {

  if (millis() - lastTime >= 20) {

    calcularRPM();
    pidControl();
    imprimirDatos();
  }

  if (Serial.available()) processSerialCommand();
}

// ====== IMPRIMIR DATOS ======
void imprimirDatos() {

  Serial.print("RPM1:");
  Serial.print(rpm1);
  Serial.print("  RPM2:");
  Serial.print(rpm2);

  Serial.print("  T1:");
  Serial.print(targetRPM1);
  Serial.print("  T2:");
  Serial.print(targetRPM2);

  Serial.print("  ENC1:");
  Serial.print(encoderPos1);
  Serial.print("  ENC2:");
  Serial.println(encoderPos2);
}

// ====== CALCULAR RPM ======
void calcularRPM() {

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // segundos

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

  // Filtro exponencial
  rpmFiltrada1 = 0.7 * rpmFiltrada1 + 0.3 * rpm1;
  rpmFiltrada2 = 0.7 * rpmFiltrada2 + 0.3 * rpm2;

  rpm1 = rpmFiltrada1;
  rpm2 = rpmFiltrada2;

  lastTime = currentTime;
}

// ====== PID ======
void pidControl() {

  float dt = 0.02;  // 20 ms

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

  setMotor2(output2);
  lastError2 = error2;
}

// ====== CONTROL MOTORES ======
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

// ====== ISR ======
void IRAM_ATTR encoder1ISR() {
  if (digitalRead(ENCODER1_B)) encoderPos1++;
  else encoderPos1--;
}

void IRAM_ATTR encoder2ISR() {
  if (digitalRead(ENCODER2_B)) encoderPos2++;
  else encoderPos2--;
}

// ====== SERIAL ======
void processSerialCommand() {

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("m1=")) targetRPM1 = cmd.substring(3).toFloat();
  else if (cmd.startsWith("m2=")) targetRPM2 = cmd.substring(3).toFloat();
  else if (cmd == "stop1") { targetRPM1 = 0; stopMotor1(); }
  else if (cmd == "stop2") { targetRPM2 = 0; stopMotor2(); }
}
