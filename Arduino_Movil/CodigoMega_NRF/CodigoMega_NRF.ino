// =====================================================
//   ARDUINO MEGA — PID DUAL + TB6612FNG
//   Receptor NRF24L01 — comandos desde ESP32
//   (compatible con dashboard Python por Serial USB)
//
//   PINOUT TB6612FNG
//   ─────────────────────────────────────────────
//   PWMA → Pin 5   (PWM Timer0)
//   AIN1 → Pin 7
//   AIN2 → Pin 8
//   PWMB → Pin 6   (PWM Timer0)
//   BIN1 → Pin 9
//   BIN2 → Pin 10
//   STBY → Pin 11
//   VCC  → 5V Arduino
//   VM   → 7.5V fuente externa
//   GND  → GND común
//
//   ENCODERS
//   ─────────────────────────────────────────────
//   Motor 1  Canal A → Pin 2  (INT0)
//   Motor 1  Canal B → Pin 4  (lectura digital)
//   Motor 2  Canal A → Pin 3  (INT1)
//   Motor 2  Canal B → Pin 12 (lectura digital)
//
//   NRF24L01 (pines SPI hardware Mega)
//   ─────────────────────────────────────────────
//   GND  → GND
//   VCC  → 3.3V  ⚠ NO conectar a 5V
//   CE   → Pin 22
//   CSN  → Pin 24
//   SCK  → Pin 52  (SPI SCK)
//   MOSI → Pin 51  (SPI MOSI)
//   MISO → Pin 50  (SPI MISO)
//   IRQ  → Sin conectar
//
//   COMPILAR COMO: Arduino Mega 2560
//   LIBRERÍA: RF24 de TMRh20 (gestor de librerías)
// =====================================================

#include <SPI.h>
#include <RF24.h>

// ── Pines NRF24L01 ──
#define CE_PIN   22
#define CSN_PIN  24

RF24 radio(CE_PIN, CSN_PIN);
const byte ADDRESS[6] = "ROBO1";   // Debe coincidir con la ESP32

// ── Pines TB6612FNG ──
#define PWMA  5
#define AIN1  7
#define AIN2  8
#define PWMB  6
#define BIN1  9
#define BIN2  10
#define STBY  11

// ── Pines encoders ──
#define ENC1A  2
#define ENC1B  4
#define ENC2A  3
#define ENC2B  12

// ── Encoder ──
const int PPR = 234;

// ── Ciclo de control ──
#define INTERVAL_MS 20   // 50 Hz

// =====================================================
//  VARIABLES ENCODER
// =====================================================
volatile long encPos1 = 0, encPos2 = 0;
long lastPos1 = 0, lastPos2 = 0;
int  motorDir1 = 1, motorDir2 = 1;

// =====================================================
//  VELOCIDADES Y PID
// =====================================================
float rpm1 = 0, rpmF1 = 0, target1 = 0;
float rpm2 = 0, rpmF2 = 0, target2 = 0;

// Ganancias PID avance
float Kp1f=15.35, Ki1f=1.500, Kd1f=0.038;
float Kp2f=15.35, Ki2f=1.545, Kd2f=0.028;
// Ganancias PID retroceso
float Kp1r=15.35, Ki1r=1.545, Kd1r=0.038;
float Kp2r=15.35, Ki2r=1.545, Kd2r=0.028;

float err1=0, lastErr1=0, integ1=0, out1=0;
float err2=0, lastErr2=0, integ2=0, out2=0;

bool  syncMode = false;
float Kp_sync  = 0.5;

bool fixSign = true;

unsigned long lastTime = 0;

// =====================================================
//  WATCHDOG DE SEGURIDAD
//  Sin comandos en WATCHDOG_MS → STOP
// =====================================================
const unsigned long WATCHDOG_MS = 2000;
unsigned long lastCmdTime = 0;

// =====================================================
//  SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  // ── Inicializar NRF24L01 ──
  if (!radio.begin()) {
    Serial.println(">> ERROR: NRF24L01 no encontrado. Revisa conexiones.");
    while (1);   // Bloquear si no hay radio
  }
  radio.openReadingPipe(1, ADDRESS);
  radio.setPALevel(RF24_PA_LOW);   // Cambia a RF24_PA_HIGH si necesitas más alcance
  radio.setDataRate(RF24_250KBPS); // Baja velocidad = mayor alcance y estabilidad
  radio.setChannel(108);           // Canal 108 (fuera de WiFi 2.4 GHz)
  radio.enableAckPayload();        // ← NECESARIO: coincide con enableAckPayload() de la ESP32
  radio.setRetries(5, 15);         // ← NECESARIO: mismos reintentos que la ESP32
  preloadAck();                    // Cargar primer ACK payload antes de startListening
  radio.startListening();
  Serial.println(">> NRF24L01 listo — modo RECEPTOR");

  // ── TB6612FNG ──
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);
  pinMode(STBY,OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotor1();
  stopMotor2();

  // ── Encoders ──
  pinMode(ENC1A,INPUT_PULLUP); pinMode(ENC1B,INPUT_PULLUP);
  pinMode(ENC2A,INPUT_PULLUP); pinMode(ENC2B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1A), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A), isr2, RISING);

  lastTime    = millis();
  lastCmdTime = millis();

  printHelp();
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  // ── Ciclo PID cada INTERVAL_MS ──
  if (now - lastTime >= INTERVAL_MS) {
    calcRPM();
    pidControl();
    printData();
  }

  // ── Watchdog ──
  if (now - lastCmdTime > WATCHDOG_MS) {
    if (target1 != 0 || target2 != 0) {
      target1 = 0; target2 = 0;
      stopMotor1(); stopMotor2();
      Serial.println(">> Watchdog STOP");
    }
  }

  // ── Leer paquete NRF24L01 ──
  if (radio.available()) {
    char buf[32] = {0};
    radio.read(&buf, sizeof(buf) - 1);
    lastCmdTime = millis();
    String cmd = String(buf);
    cmd.trim();
    Serial.print(">> RF: "); Serial.println(cmd);
    processCommand(cmd);
    preloadAck();   // ← Recargar ACK payload con telemetría actualizada
  }

  // ── Leer comandos del Serial USB (mantiene compatibilidad con Python) ──
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    lastCmdTime = millis();
    processCommand(cmd);
  }
}

// =====================================================
//  CALCULAR RPM
// =====================================================
void calcRPM() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) return;

  long p1, p2;
  noInterrupts(); p1 = encPos1; p2 = encPos2; interrupts();

  long d1 = p1 - lastPos1;
  long d2 = p2 - lastPos2;
  lastPos1 = p1; lastPos2 = p2;

  float r1 = (d1 * 60.0) / (PPR * dt);
  float r2 = (d2 * 60.0) / (PPR * dt);

  rpm1 = fixSign ? r1 * motorDir1 : r1;
  rpm2 = fixSign ? r2 * motorDir2 : r2;

  rpmF1 = 0.7*rpmF1 + 0.3*rpm1;
  rpmF2 = 0.7*rpmF2 + 0.3*rpm2;
  rpm1  = rpmF1;
  rpm2  = rpmF2;

  lastTime = now;
}

// =====================================================
//  PID CONTROL
// =====================================================
void pidControl() {
  float dt = INTERVAL_MS / 1000.0;

  float Kp1 = (target1 >= 0) ? Kp1f : Kp1r;
  float Ki1 = (target1 >= 0) ? Ki1f : Ki1r;
  float Kd1 = (target1 >= 0) ? Kd1f : Kd1r;
  float Kp2 = (target2 >= 0) ? Kp2f : Kp2r;
  float Ki2 = (target2 >= 0) ? Ki2f : Ki2r;
  float Kd2 = (target2 >= 0) ? Kd2f : Kd2r;

  err1    = target1 - rpm1;
  integ1 += err1 * dt;
  integ1  = constrain(integ1, -400.0, 400.0);
  out1    = Kp1*err1 + Ki1*integ1 + Kd1*(err1-lastErr1)/dt;
  out1    = constrain(out1, -255.0, 255.0);
  if (target1 == 0) stopMotor1(); else setMotor1(out1);
  lastErr1 = err1;

  err2    = target2 - rpm2;
  integ2 += err2 * dt;
  integ2  = constrain(integ2, -400.0, 400.0);
  out2    = Kp2*err2 + Ki2*integ2 + Kd2*(err2-lastErr2)/dt;
  out2    = constrain(out2, -255.0, 255.0);

  if (syncMode && target2 != 0) {
    float corr = Kp_sync * (abs(rpm1) - abs(rpm2));
    out2 = (out2 >= 0) ? out2+corr : out2-corr;
    out2 = constrain(out2, -255.0, 255.0);
  }

  if (target2 == 0) stopMotor2(); else setMotor2(out2);
  lastErr2 = err2;
}

// =====================================================
//  TELEMETRÍA
// =====================================================
void printData() {
  Serial.print("Target1:"); Serial.print(target1,1); Serial.print("\t");
  Serial.print("RPM1:");    Serial.print(rpm1,1);    Serial.print("\t");
  Serial.print("Err1:");    Serial.print(err1,1);    Serial.print("\t");
  Serial.print("Out1:");    Serial.print(out1,1);    Serial.print("\t");
  Serial.print("Target2:"); Serial.print(target2,1); Serial.print("\t");
  Serial.print("RPM2:");    Serial.print(rpm2,1);    Serial.print("\t");
  Serial.print("Err2:");    Serial.print(err2,1);    Serial.print("\t");
  Serial.print("Out2:");    Serial.print(out2,1);    Serial.print("\t");
  Serial.print("Sync:");    Serial.println(syncMode ? 1 : 0);
}

// =====================================================
//  MOTORES — TB6612FNG
// =====================================================
void setMotor1(float spd) {
  if (spd > 0) {
    motorDir1=1;
    digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW);
    analogWrite(PWMA,(int)abs(spd));
  } else {
    motorDir1=-1;
    digitalWrite(AIN1,LOW);  digitalWrite(AIN2,HIGH);
    analogWrite(PWMA,(int)abs(spd));
  }
}
void stopMotor1() {
  digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW);
  analogWrite(PWMA,0);
  integ1=0; motorDir1=1;
}

void setMotor2(float spd) {
  if (spd > 0) {
    motorDir2=1;
    digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW);
    analogWrite(PWMB,(int)abs(spd));
  } else {
    motorDir2=-1;
    digitalWrite(BIN1,LOW);  digitalWrite(BIN2,HIGH);
    analogWrite(PWMB,(int)abs(spd));
  }
}
void stopMotor2() {
  digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW);
  analogWrite(PWMB,0);
  integ2=0; motorDir2=1;
}

// =====================================================
//  ISR ENCODERS
// =====================================================
void isr1() { if (digitalRead(ENC1B)) encPos1++; else encPos1--; }
void isr2() { if (digitalRead(ENC2B)) encPos2++; else encPos2--; }

// =====================================================
//  COMANDOS SERIAL / RF
// =====================================================
void processCommand(String cmd) {
  cmd.trim(); cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("m1=")) {
    target1=constrain(cmd.substring(3).toFloat(),-150,150);
    integ1=0; lastErr1=0;
    Serial.print(">> M1: "); Serial.println(target1);
  }
  else if (cmd.startsWith("m2=")) {
    target2=constrain(cmd.substring(3).toFloat(),-150,150);
    integ2=0; lastErr2=0;
    Serial.print(">> M2: "); Serial.println(target2);
  }
  else if (cmd.startsWith("m12=")) {
    float v=constrain(cmd.substring(4).toFloat(),-150,150);
    target1=v; target2=v; integ1=0; integ2=0;
    Serial.print(">> M1+M2: "); Serial.println(v);
  }
  else if (cmd.startsWith("kp1f=")) { Kp1f=cmd.substring(5).toFloat(); Serial.print(">> Kp1f="); Serial.println(Kp1f,4); }
  else if (cmd.startsWith("ki1f=")) { Ki1f=cmd.substring(5).toFloat(); Serial.print(">> Ki1f="); Serial.println(Ki1f,4); }
  else if (cmd.startsWith("kd1f=")) { Kd1f=cmd.substring(5).toFloat(); Serial.print(">> Kd1f="); Serial.println(Kd1f,4); }
  else if (cmd.startsWith("kp1r=")) { Kp1r=cmd.substring(5).toFloat(); Serial.print(">> Kp1r="); Serial.println(Kp1r,4); }
  else if (cmd.startsWith("ki1r=")) { Ki1r=cmd.substring(5).toFloat(); Serial.print(">> Ki1r="); Serial.println(Ki1r,4); }
  else if (cmd.startsWith("kd1r=")) { Kd1r=cmd.substring(5).toFloat(); Serial.print(">> Kd1r="); Serial.println(Kd1r,4); }
  else if (cmd.startsWith("kp2f=")) { Kp2f=cmd.substring(5).toFloat(); Serial.print(">> Kp2f="); Serial.println(Kp2f,4); }
  else if (cmd.startsWith("ki2f=")) { Ki2f=cmd.substring(5).toFloat(); Serial.print(">> Ki2f="); Serial.println(Ki2f,4); }
  else if (cmd.startsWith("kd2f=")) { Kd2f=cmd.substring(5).toFloat(); Serial.print(">> Kd2f="); Serial.println(Kd2f,4); }
  else if (cmd.startsWith("kp2r=")) { Kp2r=cmd.substring(5).toFloat(); Serial.print(">> Kp2r="); Serial.println(Kp2r,4); }
  else if (cmd.startsWith("ki2r=")) { Ki2r=cmd.substring(5).toFloat(); Serial.print(">> Ki2r="); Serial.println(Ki2r,4); }
  else if (cmd.startsWith("kd2r=")) { Kd2r=cmd.substring(5).toFloat(); Serial.print(">> Kd2r="); Serial.println(Kd2r,4); }
  else if (cmd == "sync=1")            { syncMode=true;  Serial.println(">> Sync ON"); }
  else if (cmd == "sync=0")            { syncMode=false; Serial.println(">> Sync OFF"); }
  else if (cmd.startsWith("kpsync=")) { Kp_sync=cmd.substring(7).toFloat(); Serial.print(">> Kp_sync="); Serial.println(Kp_sync,3); }
  else if (cmd == "fixsign=1") { fixSign=true;  Serial.println(">> fixSign ON"); }
  else if (cmd == "fixsign=0") { fixSign=false; Serial.println(">> fixSign OFF"); }
  else if (cmd == "stop")  { target1=0; target2=0; stopMotor1(); stopMotor2(); Serial.println(">> STOP"); }
  else if (cmd == "stop1") { target1=0; stopMotor1(); Serial.println(">> STOP M1"); }
  else if (cmd == "stop2") { target2=0; stopMotor2(); Serial.println(">> STOP M2"); }
  else if (cmd == "reset") {
    noInterrupts(); encPos1=0; encPos2=0; lastPos1=0; lastPos2=0; interrupts();
    Serial.println(">> Encoders reseteados");
  }
  else if (cmd == "status") {
    Serial.println("\n-- STATUS --");
    Serial.print("  M1 RPM="); Serial.print(rpm1,1); Serial.print(" Tgt="); Serial.print(target1,1); Serial.print(" Out="); Serial.println(out1,1);
    Serial.print("  M2 RPM="); Serial.print(rpm2,1); Serial.print(" Tgt="); Serial.print(target2,1); Serial.print(" Out="); Serial.println(out2,1);
    Serial.print("  Sync="); Serial.print(syncMode?"ON":"OFF");
    Serial.print("  fixSign="); Serial.println(fixSign?"ON":"OFF");
  }
  else if (cmd == "diag") {
    Serial.println("\n-- DIAG --");
    Serial.print("  M1 Dir="); Serial.print(motorDir1==1?"AVANCE":"RETRO"); Serial.print(" RPM="); Serial.println(rpm1,1);
    Serial.print("  M2 Dir="); Serial.print(motorDir2==1?"AVANCE":"RETRO"); Serial.print(" RPM="); Serial.println(rpm2,1);
    Serial.println("  RPM>0 al retroceder → escribe: fixsign=1");
  }
  else if (cmd == "pid") {
    Serial.println("\n-- PID --");
    Serial.print("  M1 fwd Kp="); Serial.print(Kp1f,3); Serial.print(" Ki="); Serial.print(Ki1f,3); Serial.print(" Kd="); Serial.println(Kd1f,3);
    Serial.print("  M2 fwd Kp="); Serial.print(Kp2f,3); Serial.print(" Ki="); Serial.print(Ki2f,3); Serial.print(" Kd="); Serial.println(Kd2f,3);
    Serial.print("  M1 rev Kp="); Serial.print(Kp1r,3); Serial.print(" Ki="); Serial.print(Ki1r,3); Serial.print(" Kd="); Serial.println(Kd1r,3);
    Serial.print("  M2 rev Kp="); Serial.print(Kp2r,3); Serial.print(" Ki="); Serial.print(Ki2r,3); Serial.print(" Kd="); Serial.println(Kd2r,3);
  }
  else if (cmd == "ping") { Serial.println(">> PONG"); }
  else if (cmd == "help") { printHelp(); }
  else { Serial.print(">> Desconocido: "); Serial.println(cmd); }
}

// =====================================================
//  ACK PAYLOAD — telemetría de vuelta a la ESP32
//  Se precargar ANTES de que llegue el siguiente paquete
//  Formato compacto (máx 32 bytes): T1,R1,T2,R2
// =====================================================
void preloadAck() {
  char ack[32] = {0};
  snprintf(ack, sizeof(ack), "%.0f,%.0f,%.0f,%.0f",
           target1, rpm1, target2, rpm2);
  radio.writeAckPayload(1, ack, sizeof(ack));
}

// =====================================================
//  AYUDA
// =====================================================
void printHelp() {
  Serial.println("\n+-----------------------------------------------+");
  Serial.println("|  Arduino Mega — PID Dual  TB6612FNG + RF24   |");
  Serial.println("|  Baud: 115200  |  Receptor NRF24L01          |");
  Serial.println("+-----------------------------------------------+");
  Serial.println("  m1=80  m1=-80  m2=80  m12=80   stop");
  Serial.println("  kp1f=  ki1f=  kd1f=  (PID avance  M1)");
  Serial.println("  kp2f=  ki2f=  kd2f=  (PID avance  M2)");
  Serial.println("  kp1r=  ki1r=  kd1r=  (PID retroceso M1)");
  Serial.println("  kp2r=  ki2r=  kd2r=  (PID retroceso M2)");
  Serial.println("  sync=1/0  kpsync=  fixsign=1/0");
  Serial.println("  status  pid  diag  reset  ping  help");
  Serial.println("  [Comandos aceptados por Serial USB y RF24]");
  Serial.println("------------------------------------------------\n");
}