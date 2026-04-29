// =====================================================
//   ARDUINO MEGA 2560 — PID x4 + 2x TB6612FNG
//   Control por Serial (Python + mando Xbox 360)
//
//   Oruga IZQ: M1 (adelante) + M3 (atrás)  → targetL
//   Oruga DER: M2 (adelante) + M4 (atrás)  → targetR
//
//   ESQUEMA DE CABLEADO (tablas, alimentación, checklist):
//     WIRING_4Orugas.txt  (misma carpeta que este .ino)
//
//   Resumen pines Mega:
//     TB6612#1 STBY=26  PWMA/AIN/M1=2,22,23  PWMB/BIN/M3=3,24,25
//     TB6612#2 STBY=31  PWMA/AIN/M2=4,27,28  PWMB/BIN/M4=5,29,30
//     Encoders A (INT): M1=18 M2=19 M3=20 M4=21
//     Encoders B:        M1=32 M2=33 M3=34 M4=35
//     Serial2 ↔ ESP32: TX2=16, RX2=17 + GND común
//
//   Serial Monitor: 115200 baud, fin de línea "Nueva línea".
//   Placa: Arduino Mega or Mega 2560
//
//   Serial1 del Mega usa D18 (TX) y D19 (RX): no mezclar con otra cosa;
//   los encoders M1/M2 ya usan D18/D19 para canal A → no usar Serial1.
// =====================================================

// ── Puente H #1: M1 (A) + M3 (B) ──
#define PWMA_H1   2
#define AIN1_H1  22
#define AIN2_H1  23
#define PWMB_H1   3
#define BIN1_H1  24
#define BIN2_H1  25
#define STBY_H1  26

// ── Puente H #2: M2 (A) + M4 (B) ──
#define PWMA_H2   4
#define AIN1_H2  27
#define AIN2_H2  28
#define PWMB_H2   5
#define BIN1_H2  29
#define BIN2_H2  30
#define STBY_H2  31

// ── Encoders ──
#define ENC1A  18   // INT5
#define ENC1B  32
#define ENC2A  19   // INT4
#define ENC2B  33
#define ENC3A  20   // INT3
#define ENC3B  34
#define ENC4A  21   // INT2
#define ENC4B  35

// ── Pulsos por revolución — ajusta según tu encoder ──
const int PPR = 234;

// ── Ciclo de control ──
#define INTERVAL_MS 50   // 50 Hz

// Serial1 en Mega = TX pin 18, RX pin 19 → choca con ENC1A/ENC2A. Mantener 0 si usas encoders en 18/19.
#define ENABLE_SERIAL1_TELEMETRY 0
// Solo depuración: envía texto falso por Serial2 (puede molestar a la ESP32)
#define DEBUG_SERIAL2_FAKE_TX 0

// =====================================================
//  VARIABLES ENCODER
// =====================================================
volatile long encPos1 = 0, encPos2 = 0, encPos3 = 0, encPos4 = 0;
long lastPos1 = 0, lastPos2 = 0, lastPos3 = 0, lastPos4 = 0;
int  motorDir1 = 1, motorDir2 = 1, motorDir3 = 1, motorDir4 = 1;

// =====================================================
//  SETPOINTS Y RPM
//  targetL = oruga izquierda (M1+M3)
//  targetR = oruga derecha   (M2+M4)
// =====================================================
float targetL = 0, targetR = 0;   // setpoints por oruga

float rpm1=0, rpmF1=0;
float rpm2=0, rpmF2=0;
float rpm3=0, rpmF3=0;
float rpm4=0, rpmF4=0;

// =====================================================
//  PID x4  (avance y retroceso independientes)
// =====================================================
// Motor 1 — izq adelante
float Kp1f=15.35, Ki1f=1.500, Kd1f=0.038;
float Kp1r=15.35, Ki1r=1.500, Kd1r=0.038;
// Motor 2 — der adelante
float Kp2f=15.35, Ki2f=1.545, Kd2f=0.028;
float Kp2r=15.35, Ki2r=1.545, Kd2r=0.028;
// Motor 3 — izq atrás
float Kp3f=15.35, Ki3f=1.500, Kd3f=0.038;
float Kp3r=15.35, Ki3r=1.500, Kd3r=0.038;
// Motor 4 — der atrás
float Kp4f=15.35, Ki4f=1.545, Kd4f=0.028;
float Kp4r=15.35, Ki4r=1.545, Kd4r=0.028;

float err1=0,lastErr1=0,integ1=0,out1=0;
float err2=0,lastErr2=0,integ2=0,out2=0;
float err3=0,lastErr3=0,integ3=0,out3=0;
float err4=0,lastErr4=0,integ4=0,out4=0;

// fixSign=true → signo por dirección HW (recomendado)
bool fixSign = true;

// Calibración TB6612: sentido motor vs encoder (misma oruga debe medir RPM con signo coherente)
// h2swap=1 → intercambia canales A/B del driver 2 (M2↔M4 en pines)
// m3inv=1  → invierte PWM motor 3 (canal B driver 1) si RPM3 va al revés de M1
// m4inv=1  → invierte PWM motor 4 (canal B driver 2)
bool swapH2Motors = false;
bool invertM3Out  = false;
bool invertM4Out  = false;

unsigned long lastTime = 0;
String serial2Buf = "";
String serialUsbBuf = "";

// =====================================================
//  WATCHDOG
// =====================================================
const unsigned long WATCHDOG_MS = 5000;
unsigned long lastCmdTime = 0;

// =====================================================
//  SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  // Mega: Serial2 TX=16, RX=17 (no usar 18/19: son Serial1 y encoders M1/M2)
  Serial2.begin(115200);
  Serial.setTimeout(5);
  Serial2.setTimeout(5);

  // Puente H #1
  pinMode(AIN1_H1,OUTPUT); pinMode(AIN2_H1,OUTPUT);
  pinMode(BIN1_H1,OUTPUT); pinMode(BIN2_H1,OUTPUT);
  pinMode(STBY_H1,OUTPUT); digitalWrite(STBY_H1, HIGH);

  // Puente H #2
  pinMode(AIN1_H2,OUTPUT); pinMode(AIN2_H2,OUTPUT);
  pinMode(BIN1_H2,OUTPUT); pinMode(BIN2_H2,OUTPUT);
  pinMode(STBY_H2,OUTPUT); digitalWrite(STBY_H2, HIGH);

  //stopAll();

  // Encoders con pull-up interno
  pinMode(ENC1A,INPUT_PULLUP); pinMode(ENC1B,INPUT_PULLUP);
  pinMode(ENC2A,INPUT_PULLUP); pinMode(ENC2B,INPUT_PULLUP);
  pinMode(ENC3A,INPUT_PULLUP); pinMode(ENC3B,INPUT_PULLUP);
  pinMode(ENC4A,INPUT_PULLUP); pinMode(ENC4B,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1A), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A), isr2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC3A), isr3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC4A), isr4, RISING);

  lastTime    = millis();
  lastCmdTime = millis();

  printHelp();
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

#if DEBUG_SERIAL2_FAKE_TX
  static unsigned long lastMsg = 0;
  if (millis() - lastMsg > 500) {
    lastMsg = millis();
    Serial2.println("TL: 150.0 RPM1: 100");
  }
#endif

  // Comandos desde ESP32 (no bloqueante, evita congelar ciclo PID)
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n' || c == '\r') {
      serial2Buf.trim();
      if (serial2Buf.length() > 0) {
        lastCmdTime = millis();
        Serial.print("ESP32> ");
        Serial.println(serial2Buf);
        processCommand(serial2Buf);
      }
      serial2Buf = "";
    } else {
      if (serial2Buf.length() < 96) serial2Buf += c;
      else serial2Buf = "";
    }
  }

  if (now - lastTime >= INTERVAL_MS) {
    calcRPM();
    pidControl();
    printData();
  }

  // Watchdog
  if (now - lastCmdTime > WATCHDOG_MS) {
    if (targetL != 0 || targetR != 0) {
      targetL = 0; targetR = 0;
      stopAll();
      Serial.println(">> Watchdog STOP");
    }
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialUsbBuf.trim();
      if (serialUsbBuf.length() > 0) {
        lastCmdTime = millis();
        processCommand(serialUsbBuf);
      }
      serialUsbBuf = "";
    } else {
      if (serialUsbBuf.length() < 96) serialUsbBuf += c;
      else serialUsbBuf = "";
    }
  }
  
}
// =====================================================
//  CALCULAR RPM
// =====================================================
void calcRPM() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) return;

  long p1,p2,p3,p4;
  noInterrupts();
  p1=encPos1; p2=encPos2; p3=encPos3; p4=encPos4;
  interrupts();

  long d1=p1-lastPos1, d2=p2-lastPos2, d3=p3-lastPos3, d4=p4-lastPos4;
  lastPos1=p1; lastPos2=p2; lastPos3=p3; lastPos4=p4;

  float r1=(d1*60.0)/(PPR*dt);
  float r2=(d2*60.0)/(PPR*dt);
  float r3=(d3*60.0)/(PPR*dt);
  float r4=(d4*60.0)/(PPR*dt);

  rpm1 = fixSign ? r1*motorDir1 : r1;
  rpm2 = fixSign ? r2*motorDir2 : r2;
  rpm3 = fixSign ? r3*motorDir3 : r3;
  rpm4 = fixSign ? r4*motorDir4 : r4;

  // Filtro pasa bajos
  rpmF1=0.7*rpmF1+0.3*rpm1; rpm1=rpmF1;
  rpmF2=0.7*rpmF2+0.3*rpm2; rpm2=rpmF2;
  rpmF3=0.7*rpmF3+0.3*rpm3; rpm3=rpmF3;
  rpmF4=0.7*rpmF4+0.3*rpm4; rpm4=rpmF4;

  lastTime = now;
}

// =====================================================
//  PID CONTROL
//  M1 y M3 siguen targetL (oruga izq)
//  M2 y M4 siguen targetR (oruga der)
// =====================================================
void pidControl() {
  float dt = INTERVAL_MS / 1000.0;

  // ── Motor 1 (izq adelante) ──
  float kp=targetL>=0?Kp1f:Kp1r, ki=targetL>=0?Ki1f:Ki1r, kd=targetL>=0?Kd1f:Kd1r;
  err1    = targetL - rpm1;
  integ1 += err1*dt; integ1=constrain(integ1,-400,400);
  out1    = kp*err1 + ki*integ1 + kd*(err1-lastErr1)/dt;
  out1    = constrain(out1,-255,255);
  targetL==0 ? stopMotor1() : setMotor1(out1);
  lastErr1 = err1;

  // ── Motor 2 (der adelante) ──
  kp=targetR>=0?Kp2f:Kp2r; ki=targetR>=0?Ki2f:Ki2r; kd=targetR>=0?Kd2f:Kd2r;
  err2    = targetR - rpm2;
  integ2 += err2*dt; integ2=constrain(integ2,-400,400);
  out2    = kp*err2 + ki*integ2 + kd*(err2-lastErr2)/dt;
  out2    = constrain(out2,-255,255);
  targetR==0 ? stopMotor2() : setMotor2(out2);
  lastErr2 = err2;

  // ── Motor 3 (izq atrás) — mismo setpoint que M1 ──
  kp=targetL>=0?Kp3f:Kp3r; ki=targetL>=0?Ki3f:Ki3r; kd=targetL>=0?Kd3f:Kd3r;
  err3    = targetL - rpm3;
  integ3 += err3*dt; integ3=constrain(integ3,-400,400);
  out3    = kp*err3 + ki*integ3 + kd*(err3-lastErr3)/dt;
  out3    = constrain(out3,-255,255);
  targetL==0 ? stopMotor3() : setMotor3(out3);
  lastErr3 = err3;

  // ── Motor 4 (der atrás) — mismo setpoint que M2 ──
  kp=targetR>=0?Kp4f:Kp4r; ki=targetR>=0?Ki4f:Ki4r; kd=targetR>=0?Kd4f:Kd4r;
  err4    = targetR - rpm4;
  integ4 += err4*dt; integ4=constrain(integ4,-400,400);
  out4    = kp*err4 + ki*integ4 + kd*(err4-lastErr4)/dt;
  out4    = constrain(out4,-255,255);
  targetR==0 ? stopMotor4() : setMotor4(out4);
  lastErr4 = err4;
}

// ── Salidas H2 (canal A = M2 por defecto, B = M4) ──
static void driveH2A(float spd, int* motorDir) {
  if (spd > 0) { *motorDir = 1;  digitalWrite(AIN1_H2, HIGH); digitalWrite(AIN2_H2, LOW); }
  else         { *motorDir = -1; digitalWrite(AIN1_H2, LOW);  digitalWrite(AIN2_H2, HIGH); }
  analogWrite(PWMA_H2, (int)abs(spd));
}
static void driveH2B(float spd, int* motorDir) {
  if (spd > 0) { *motorDir = 1;  digitalWrite(BIN1_H2, HIGH); digitalWrite(BIN2_H2, LOW); }
  else         { *motorDir = -1; digitalWrite(BIN1_H2, LOW);  digitalWrite(BIN2_H2, HIGH); }
  analogWrite(PWMB_H2, (int)abs(spd));
}
static void stopH2A() {
  digitalWrite(AIN1_H2, LOW); digitalWrite(AIN2_H2, LOW); analogWrite(PWMA_H2, 0);
}
static void stopH2B() {
  digitalWrite(BIN1_H2, LOW); digitalWrite(BIN2_H2, LOW); analogWrite(PWMB_H2, 0);
}

// =====================================================
//  TELEMETRÍA
// =====================================================
void printData() {
  Serial.print("TL:");   Serial.print(targetL,1); Serial.print("\t");
  Serial.print("TR:");   Serial.print(targetR,1); Serial.print("\t");
  Serial.print("RPM1:"); Serial.print(rpm1,1);    Serial.print("\t");
  Serial.print("RPM2:"); Serial.print(rpm2,1);    Serial.print("\t");
  Serial.print("RPM3:"); Serial.print(rpm3,1);    Serial.print("\t");
  Serial.print("RPM4:"); Serial.print(rpm4,1);    Serial.print("\t");
  Serial.print("Err1:"); Serial.print(err1,1);    Serial.print("\t");
  Serial.print("Err2:"); Serial.print(err2,1);    Serial.print("\t");
  Serial.print("Err3:"); Serial.print(err3,1);    Serial.print("\t");
  Serial.print("Err4:"); Serial.print(err4,1);    Serial.print("\t");
  Serial.print("Out1:"); Serial.print(out1,1);    Serial.print("\t");
  Serial.print("Out2:"); Serial.print(out2,1);    Serial.print("\t");
  Serial.print("Out3:"); Serial.print(out3,1);    Serial.print("\t");
  Serial.print("Out4:"); Serial.println(out4,1);
#if ENABLE_SERIAL1_TELEMETRY
  Serial1.print("TL:"); Serial1.print(targetL,1); Serial1.print("\t");
  Serial1.print("TR:"); Serial1.print(targetR,1); Serial1.print("\t");
  Serial1.print("RPM1:"); Serial1.print(rpm1,1); Serial1.print("\t");
  Serial1.print("RPM2:"); Serial1.print(rpm2,1); Serial1.print("\t");
  Serial1.print("RPM3:"); Serial1.print(rpm3,1); Serial1.print("\t");
  Serial1.print("RPM4:"); Serial1.println(rpm4,1);
#endif
}

// =====================================================
//  MOTORES — Puente H #1: M1 (canal A), M3 (canal B)
// =====================================================
void setMotor1(float spd) {
  if (spd>0){ motorDir1=1; digitalWrite(AIN1_H1,HIGH); digitalWrite(AIN2_H1,LOW); }
  else      { motorDir1=-1;digitalWrite(AIN1_H1,LOW);  digitalWrite(AIN2_H1,HIGH);}
  analogWrite(PWMA_H1,(int)abs(spd));
}
void stopMotor1() {
  digitalWrite(AIN1_H1,LOW); digitalWrite(AIN2_H1,LOW);
  analogWrite(PWMA_H1,0); integ1=0; motorDir1=1;
}

void setMotor3(float spd) {
  float s = invertM3Out ? -spd : spd;
  if (s>0){ motorDir3=1; digitalWrite(BIN1_H1,HIGH); digitalWrite(BIN2_H1,LOW); }
  else    { motorDir3=-1;digitalWrite(BIN1_H1,LOW);  digitalWrite(BIN2_H1,HIGH);}
  analogWrite(PWMB_H1,(int)abs(s));
}
void stopMotor3() {
  digitalWrite(BIN1_H1,LOW); digitalWrite(BIN2_H1,LOW);
  analogWrite(PWMB_H1,0); integ3=0; motorDir3=1;
}

// =====================================================
//  MOTORES — Puente H #2: M2 (canal A), M4 (canal B)  [swapH2Motors intercambia pines]
// =====================================================
void setMotor2(float spd) {
  if (swapH2Motors) driveH2B(spd, &motorDir2);
  else              driveH2A(spd, &motorDir2);
}
void stopMotor2() {
  if (swapH2Motors) stopH2B(); else stopH2A();
  integ2 = 0; motorDir2 = 1;
}

void setMotor4(float spd) {
  float s = invertM4Out ? -spd : spd;
  if (swapH2Motors) driveH2A(s, &motorDir4);
  else              driveH2B(s, &motorDir4);
}
void stopMotor4() {
  if (swapH2Motors) stopH2A(); else stopH2B();
  integ4 = 0; motorDir4 = 1;
}

void stopAll() {
  stopMotor1(); stopMotor2(); stopMotor3(); stopMotor4();
}

// =====================================================
//  ISR ENCODERS — cuadratura
// =====================================================
void isr1() { encPos1 += (digitalRead(ENC1B)==HIGH) ? 1 : -1; }
void isr2() { encPos2 += (digitalRead(ENC2B)==HIGH) ? 1 : -1; }
void isr3() { encPos3 += (digitalRead(ENC3B)==HIGH) ? 1 : -1; }
void isr4() { encPos4 += (digitalRead(ENC4B)==HIGH) ? 1 : -1; }

// =====================================================
//  COMANDOS SERIAL
// =====================================================
void processCommand(String cmd) {
  cmd.trim(); cmd.toLowerCase();
  if (cmd.length()==0) return;

  // Aliases de movimiento por palabra
  if      (cmd == "forward") cmd = "lr=100";
  else if (cmd == "back")    cmd = "lr=-100";
  else if (cmd == "left")    { targetL = -100; targetR = 100; resetPID_L(); resetPID_R(); Serial.println(">> LEFT"); return; }
  else if (cmd == "right")   { targetL = 100;  targetR = -100; resetPID_L(); resetPID_R(); Serial.println(">> RIGHT"); return; }

  // ── Setpoints por oruga ──
  if      (cmd.startsWith("l="))  { targetL=constrain(cmd.substring(2).toFloat(),-150,150); resetPID_L(); Serial.print(">> Oruga IZQ: "); Serial.println(targetL); }
  else if (cmd.startsWith("r="))  { targetR=constrain(cmd.substring(2).toFloat(),-150,150); resetPID_R(); Serial.print(">> Oruga DER: "); Serial.println(targetR); }
  else if (cmd.startsWith("lr=")) { float v=constrain(cmd.substring(3).toFloat(),-150,150); targetL=v; targetR=v; resetPID_L(); resetPID_R(); Serial.print(">> Ambas: "); Serial.println(v); }

  // ── PID por motor (avance) ──
  else if (cmd.startsWith("kp1f=")) { Kp1f=cmd.substring(5).toFloat(); pok("Kp1f",Kp1f); }
  else if (cmd.startsWith("ki1f=")) { Ki1f=cmd.substring(5).toFloat(); pok("Ki1f",Ki1f); }
  else if (cmd.startsWith("kd1f=")) { Kd1f=cmd.substring(5).toFloat(); pok("Kd1f",Kd1f); }
  else if (cmd.startsWith("kp2f=")) { Kp2f=cmd.substring(5).toFloat(); pok("Kp2f",Kp2f); }
  else if (cmd.startsWith("ki2f=")) { Ki2f=cmd.substring(5).toFloat(); pok("Ki2f",Ki2f); }
  else if (cmd.startsWith("kd2f=")) { Kd2f=cmd.substring(5).toFloat(); pok("Kd2f",Kd2f); }
  else if (cmd.startsWith("kp3f=")) { Kp3f=cmd.substring(5).toFloat(); pok("Kp3f",Kp3f); }
  else if (cmd.startsWith("ki3f=")) { Ki3f=cmd.substring(5).toFloat(); pok("Ki3f",Ki3f); }
  else if (cmd.startsWith("kd3f=")) { Kd3f=cmd.substring(5).toFloat(); pok("Kd3f",Kd3f); }
  else if (cmd.startsWith("kp4f=")) { Kp4f=cmd.substring(5).toFloat(); pok("Kp4f",Kp4f); }
  else if (cmd.startsWith("ki4f=")) { Ki4f=cmd.substring(5).toFloat(); pok("Ki4f",Ki4f); }
  else if (cmd.startsWith("kd4f=")) { Kd4f=cmd.substring(5).toFloat(); pok("Kd4f",Kd4f); }

  // ── PID por motor (retroceso) ──
  else if (cmd.startsWith("kp1r=")) { Kp1r=cmd.substring(5).toFloat(); pok("Kp1r",Kp1r); }
  else if (cmd.startsWith("ki1r=")) { Ki1r=cmd.substring(5).toFloat(); pok("Ki1r",Ki1r); }
  else if (cmd.startsWith("kd1r=")) { Kd1r=cmd.substring(5).toFloat(); pok("Kd1r",Kd1r); }
  else if (cmd.startsWith("kp2r=")) { Kp2r=cmd.substring(5).toFloat(); pok("Kp2r",Kp2r); }
  else if (cmd.startsWith("ki2r=")) { Ki2r=cmd.substring(5).toFloat(); pok("Ki2r",Ki2r); }
  else if (cmd.startsWith("kd2r=")) { Kd2r=cmd.substring(5).toFloat(); pok("Kd2r",Kd2r); }
  else if (cmd.startsWith("kp3r=")) { Kp3r=cmd.substring(5).toFloat(); pok("Kp3r",Kp3r); }
  else if (cmd.startsWith("ki3r=")) { Ki3r=cmd.substring(5).toFloat(); pok("Ki3r",Ki3r); }
  else if (cmd.startsWith("kd3r=")) { Kd3r=cmd.substring(5).toFloat(); pok("Kd3r",Kd3r); }
  else if (cmd.startsWith("kp4r=")) { Kp4r=cmd.substring(5).toFloat(); pok("Kp4r",Kp4r); }
  else if (cmd.startsWith("ki4r=")) { Ki4r=cmd.substring(5).toFloat(); pok("Ki4r",Ki4r); }
  else if (cmd.startsWith("kd4r=")) { Kd4r=cmd.substring(5).toFloat(); pok("Kd4r",Kd4r); }

  // ── Ajuste simétrico por oruga (aplica a M1+M3 o M2+M4) ──
  else if (cmd.startsWith("kpl="))  { Kp1f=Kp1r=Kp3f=Kp3r=cmd.substring(4).toFloat(); pok("Kp oruga izq",Kp1f); }
  else if (cmd.startsWith("kpr="))  { Kp2f=Kp2r=Kp4f=Kp4r=cmd.substring(4).toFloat(); pok("Kp oruga der",Kp2f); }
  else if (cmd.startsWith("kil="))  { Ki1f=Ki1r=Ki3f=Ki3r=cmd.substring(4).toFloat(); pok("Ki oruga izq",Ki1f); }
  else if (cmd.startsWith("kir="))  { Ki2f=Ki2r=Ki4f=Ki4r=cmd.substring(4).toFloat(); pok("Ki oruga der",Ki2f); }
  else if (cmd.startsWith("kdl="))  { Kd1f=Kd1r=Kd3f=Kd3r=cmd.substring(4).toFloat(); pok("Kd oruga izq",Kd1f); }
  else if (cmd.startsWith("kdr="))  { Kd2f=Kd2r=Kd4f=Kd4r=cmd.substring(4).toFloat(); pok("Kd oruga der",Kd2f); }

  // ── Calibración driver 2 (M2/M4): si solo uno reacciona o van cruzados ──
  else if (cmd.startsWith("h2swap=")) {
    swapH2Motors = (cmd.substring(7).toInt() != 0);
    resetPID_R();
    Serial.print(">> swapH2Motors="); Serial.println(swapH2Motors ? "ON (M2↔pines M4)" : "OFF");
  }
  else if (cmd.startsWith("m3inv=")) {
    invertM3Out = (cmd.substring(6).toInt() != 0);
    resetPID_L();
    Serial.print(">> invertM3Out="); Serial.println(invertM3Out ? "ON" : "OFF");
  }
  else if (cmd.startsWith("m4inv=")) {
    invertM4Out = (cmd.substring(6).toInt() != 0);
    resetPID_R();
    Serial.print(">> invertM4Out="); Serial.println(invertM4Out ? "ON" : "OFF");
  }

  // ── Fix sign ──
  else if (cmd=="fixsign=1") { fixSign=true;  Serial.println(">> fixSign ON");  }
  else if (cmd=="fixsign=0") { fixSign=false; Serial.println(">> fixSign OFF"); }

  // ── Control ──
  else if (cmd=="stop")  { targetL=0; targetR=0; stopAll(); Serial.println(">> STOP"); }
  else if (cmd=="reset") {
    noInterrupts(); encPos1=encPos2=encPos3=encPos4=0;
    lastPos1=lastPos2=lastPos3=lastPos4=0; interrupts();
    Serial.println(">> Encoders reseteados");
  }

  // ── Info ──
  else if (cmd=="status") {
    Serial.println("\n-- STATUS --");
    Serial.print("  Oruga IZQ  Tgt="); Serial.print(targetL,1);
    Serial.print("  M1 RPM="); Serial.print(rpm1,1);
    Serial.print("  M3 RPM="); Serial.println(rpm3,1);
    Serial.print("  Oruga DER  Tgt="); Serial.print(targetR,1);
    Serial.print("  M2 RPM="); Serial.print(rpm2,1);
    Serial.print("  M4 RPM="); Serial.println(rpm4,1);
    Serial.print("  fixSign="); Serial.print(fixSign?"ON":"OFF");
    Serial.print("  h2swap="); Serial.print(swapH2Motors?"ON":"OFF");
    Serial.print("  m3inv="); Serial.print(invertM3Out?"ON":"OFF");
    Serial.print("  m4inv="); Serial.println(invertM4Out?"ON":"OFF");
  }
  else if (cmd=="pid") {
    Serial.println("\n-- PID --");
    Serial.print("  M1 fwd Kp="); Serial.print(Kp1f,3); Serial.print(" Ki="); Serial.print(Ki1f,3); Serial.print(" Kd="); Serial.println(Kd1f,3);
    Serial.print("  M2 fwd Kp="); Serial.print(Kp2f,3); Serial.print(" Ki="); Serial.print(Ki2f,3); Serial.print(" Kd="); Serial.println(Kd2f,3);
    Serial.print("  M3 fwd Kp="); Serial.print(Kp3f,3); Serial.print(" Ki="); Serial.print(Ki3f,3); Serial.print(" Kd="); Serial.println(Kd3f,3);
    Serial.print("  M4 fwd Kp="); Serial.print(Kp4f,3); Serial.print(" Ki="); Serial.print(Ki4f,3); Serial.print(" Kd="); Serial.println(Kd4f,3);
  }
  else if (cmd=="diag") {
    Serial.println("\n-- DIAG --");
    Serial.print("  M1 Dir="); Serial.print(motorDir1==1?"FWD":"REV"); Serial.print(" RPM="); Serial.println(rpm1,1);
    Serial.print("  M2 Dir="); Serial.print(motorDir2==1?"FWD":"REV"); Serial.print(" RPM="); Serial.println(rpm2,1);
    Serial.print("  M3 Dir="); Serial.print(motorDir3==1?"FWD":"REV"); Serial.print(" RPM="); Serial.println(rpm3,1);
    Serial.print("  M4 Dir="); Serial.print(motorDir4==1?"FWD":"REV"); Serial.print(" RPM="); Serial.println(rpm4,1);
    Serial.println("  Si RPM>0 al retroceder → escribe: fixsign=1");
  }
  else if (cmd=="enc") {
    long e1, e2, e3, e4;
    noInterrupts();
    e1 = encPos1; e2 = encPos2; e3 = encPos3; e4 = encPos4;
    interrupts();
    Serial.println("\n-- ENC (pulsos acum.) --");
    Serial.print("  enc1="); Serial.print(e1);
    Serial.print("  enc2="); Serial.print(e2);
    Serial.print("  enc3="); Serial.print(e3);
    Serial.print("  enc4="); Serial.println(e4);
    Serial.println("  Gira un eje a mano: si el contador no cambia, revisa cable encoder.");
  }
  else if (cmd=="ping") { Serial.println(">> PONG"); }
  else if (cmd=="help") { printHelp(); }
  else { Serial.print(">> Desconocido: "); Serial.println(cmd); }
}

// ── Helpers ──
void resetPID_L() { integ1=0; lastErr1=0; integ3=0; lastErr3=0; }
void resetPID_R() { integ2=0; lastErr2=0; integ4=0; lastErr4=0; }
void pok(const char* n, float v) { Serial.print(">> "); Serial.print(n); Serial.print("="); Serial.println(v,4); }

// =====================================================
//  AYUDA
// =====================================================
void printHelp() {
  Serial.println("\n+---------------------------------------------------+");
  Serial.println("|  Arduino Mega — PID x4  2xTB6612FNG  4 motores   |");
  Serial.println("|  Baud: 115200  |  Line ending: Nueva linea        |");
  Serial.println("+---------------------------------------------------+");
  Serial.println("  Setpoints por oruga:");
  Serial.println("    l=80  l=-80   (oruga izquierda M1+M3)");
  Serial.println("    r=80  r=-80   (oruga derecha   M2+M4)");
  Serial.println("    lr=80         (ambas orugas)");
  Serial.println("  PID por motor:");
  Serial.println("    kp1f=  ki1f=  kd1f=  (M1 avance)");
  Serial.println("    kp1r=  ki1r=  kd1r=  (M1 retroceso)");
  Serial.println("    ... kp2f .. kp3f .. kp4f  (igual por motor)");
  Serial.println("  PID por oruga (afecta M1+M3 o M2+M4):");
  Serial.println("    kpl=  kil=  kdl=   (oruga izq)");
  Serial.println("    kpr=  kir=  kdr=   (oruga der)");
  Serial.println("  fixsign=1/0  stop  reset  status  pid  diag  enc  ping");
  Serial.println("  Driver H1 (M3 vs M1): m3inv=1   Driver H2: h2swap=1 | m4inv=1");
  Serial.println("---------------------------------------------------\n");
}
