// =====================================================
//   ARDUINO MEGA 2560 — Receptor nRF24L01+ PA/LNA
//   PID x4 motores / 2 orugas  +  2x TB6612FNG
//
//   Librería requerida: RF24 by TMRh20
//
//   PINOUT nRF24L01+ → Arduino Mega
//   ─────────────────────────────────────────────
//   VCC  → 3.3V   (NUNCA 5V)
//   GND  → GND
//   CE   → Pin 48
//   CSN  → Pin 49
//   SCK  → Pin 52  (SPI bus Mega)
//   MOSI → Pin 51  (SPI bus Mega)
//   MISO → Pin 50  (SPI bus Mega)
//   IRQ  → sin conectar
//
//   CONDENSADOR 100uF entre VCC y GND del módulo.
//
//   PINOUT TB6612FNG, encoders y motores:
//   ver mega_pid_4motores.ino — sin cambios
//
//   Compilar como: Arduino Mega or Mega 2560
// =====================================================

#include <SPI.h>
#include <RF24.h>

#define PIN_CE   48
#define PIN_CSN  49

RF24 radio(PIN_CE, PIN_CSN);

const uint64_t PIPE_ADDR = 0xE8E8F0F0E1LL;   // misma que ESP32

// ── Estructura del paquete ─────────────────────────
struct Packet {
  char    cmd;
  float   left;
  float   right;
  uint8_t seq;
};

// =====================================================
//  PINES TB6612FNG
// =====================================================
#define PWMA_H1  2
#define AIN1_H1 22
#define AIN2_H1 23
#define PWMB_H1  3
#define BIN1_H1 24
#define BIN2_H1 25
#define STBY_H1 26

#define PWMA_H2  4
#define AIN1_H2 27
#define AIN2_H2 28
#define PWMB_H2  5
#define BIN1_H2 29
#define BIN2_H2 30
#define STBY_H2 31

// ── Encoders ──
#define ENC1A 18
#define ENC1B 32
#define ENC2A 19
#define ENC2B 33
#define ENC3A 20
#define ENC3B 34
#define ENC4A 21
#define ENC4B 35

const int PPR = 234;
#define INTERVAL_MS 50

// =====================================================
//  VARIABLES ENCODER / RPM / PID
// =====================================================
volatile long encPos1=0, encPos2=0, encPos3=0, encPos4=0;
long lastPos1=0, lastPos2=0, lastPos3=0, lastPos4=0;
int  motorDir1=1, motorDir2=1, motorDir3=1, motorDir4=1;

float targetL=0, targetR=0;

float rpm1=0,rpmF1=0, rpm2=0,rpmF2=0;
float rpm3=0,rpmF3=0, rpm4=0,rpmF4=0;

float Kp1f=15.35,Ki1f=1.500,Kd1f=0.038;
float Kp2f=15.35,Ki2f=1.545,Kd2f=0.028;
float Kp3f=15.35,Ki3f=1.500,Kd3f=0.038;
float Kp4f=15.35,Ki4f=1.545,Kd4f=0.028;
float Kp1r=15.35,Ki1r=1.500,Kd1r=0.038;
float Kp2r=15.35,Ki2r=1.545,Kd2r=0.028;
float Kp3r=15.35,Ki3r=1.500,Kd3r=0.038;
float Kp4r=15.35,Ki4r=1.545,Kd4r=0.028;

float err1=0,lastErr1=0,integ1=0,out1=0;
float err2=0,lastErr2=0,integ2=0,out2=0;
float err3=0,lastErr3=0,integ3=0,out3=0;
float err4=0,lastErr4=0,integ4=0,out4=0;

bool fixSign = true;
unsigned long lastTime = 0;

// ── Watchdog RF ────────────────────────────────────
// Sin paquetes RF en WATCHDOG_RF_MS → STOP seguridad
const unsigned long WATCHDOG_RF_MS = 3000;
unsigned long lastPktTime = 0;

// ── Estado para ACK payload ────────────────────────
uint8_t ackStatus = 0;   // 0=running 1=stopped

// =====================================================
//  SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  // TB6612FNG H1
  pinMode(AIN1_H1,OUTPUT); pinMode(AIN2_H1,OUTPUT);
  pinMode(BIN1_H1,OUTPUT); pinMode(BIN2_H1,OUTPUT);
  pinMode(STBY_H1,OUTPUT); digitalWrite(STBY_H1,HIGH);
  // TB6612FNG H2
  pinMode(AIN1_H2,OUTPUT); pinMode(AIN2_H2,OUTPUT);
  pinMode(BIN1_H2,OUTPUT); pinMode(BIN2_H2,OUTPUT);
  pinMode(STBY_H2,OUTPUT); digitalWrite(STBY_H2,HIGH);
  stopAll();

  // Encoders
  pinMode(ENC1A,INPUT_PULLUP); pinMode(ENC1B,INPUT_PULLUP);
  pinMode(ENC2A,INPUT_PULLUP); pinMode(ENC2B,INPUT_PULLUP);
  pinMode(ENC3A,INPUT_PULLUP); pinMode(ENC3B,INPUT_PULLUP);
  pinMode(ENC4A,INPUT_PULLUP); pinMode(ENC4B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1A), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A), isr2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC3A), isr3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC4A), isr4, RISING);

  // nRF24
  if (!radio.begin()) {
    Serial.println("ERR: nRF24 no detectado");
    while (1);
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(5, 15);
  radio.enableAckPayload();
  radio.setPayloadSize(sizeof(Packet));
  radio.openReadingPipe(1, PIPE_ADDR);
  radio.startListening();

  // Cargar primer ACK payload (estado inicial)
  radio.writeAckPayload(1, &ackStatus, 1);

  lastTime    = millis();
  lastPktTime = millis();

  Serial.println("OK: Mega receptor nRF24 listo");
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  // ── Recibir paquete RF ──────────────────────────
  if (radio.available()) {
    Packet pkt;
    radio.read(&pkt, sizeof(pkt));
    lastPktTime = now;
    processPacket(pkt);

    // Actualizar ACK payload para el siguiente paquete
    ackStatus = (targetL == 0 && targetR == 0) ? 1 : 0;
    radio.writeAckPayload(1, &ackStatus, 1);
  }

  // ── Watchdog RF ─────────────────────────────────
  if (now - lastPktTime > WATCHDOG_RF_MS) {
    if (targetL != 0 || targetR != 0) {
      targetL = 0; targetR = 0;
      stopAll();
      Serial.println("WD: watchdog RF → STOP");
    }
  }

  // ── Ciclo PID ───────────────────────────────────
  if (now - lastTime >= INTERVAL_MS) {
    calcRPM();
    pidControl();
  }
}

// =====================================================
//  PROCESAR PAQUETE
// =====================================================
void processPacket(const Packet& pkt) {
  switch (pkt.cmd) {
    case 'S':
      targetL = 0; targetR = 0;
      stopAll();
      break;
    case 'B':
      targetL = constrain(pkt.left,  -150.0f, 150.0f);
      targetR = constrain(pkt.right, -150.0f, 150.0f);
      resetPID_L(); resetPID_R();
      break;
    case 'L':
      targetL = constrain(pkt.left, -150.0f, 150.0f);
      resetPID_L();
      break;
    case 'R':
      targetR = constrain(pkt.right, -150.0f, 150.0f);
      resetPID_R();
      break;
  }
}

// =====================================================
//  CALCULAR RPM
// =====================================================
void calcRPM() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0) return;

  long p1,p2,p3,p4;
  noInterrupts();
  p1=encPos1; p2=encPos2; p3=encPos3; p4=encPos4;
  interrupts();

  long d1=p1-lastPos1, d2=p2-lastPos2;
  long d3=p3-lastPos3, d4=p4-lastPos4;
  lastPos1=p1; lastPos2=p2; lastPos3=p3; lastPos4=p4;

  float r1=(d1*60.0f)/(PPR*dt), r2=(d2*60.0f)/(PPR*dt);
  float r3=(d3*60.0f)/(PPR*dt), r4=(d4*60.0f)/(PPR*dt);

  rpm1 = fixSign ? r1*motorDir1 : r1;
  rpm2 = fixSign ? r2*motorDir2 : r2;
  rpm3 = fixSign ? r3*motorDir3 : r3;
  rpm4 = fixSign ? r4*motorDir4 : r4;

  rpmF1=0.7f*rpmF1+0.3f*rpm1; rpm1=rpmF1;
  rpmF2=0.7f*rpmF2+0.3f*rpm2; rpm2=rpmF2;
  rpmF3=0.7f*rpmF3+0.3f*rpm3; rpm3=rpmF3;
  rpmF4=0.7f*rpmF4+0.3f*rpm4; rpm4=rpmF4;

  lastTime = now;
}

// =====================================================
//  PID CONTROL
// =====================================================
void pidControl() {
  float dt = INTERVAL_MS / 1000.0f;

  // M1 — izq adelante
  float kp=targetL>=0?Kp1f:Kp1r, ki=targetL>=0?Ki1f:Ki1r, kd=targetL>=0?Kd1f:Kd1r;
  err1=targetL-rpm1; integ1=constrain(integ1+err1*dt,-400,400);
  out1=constrain(kp*err1+ki*integ1+kd*(err1-lastErr1)/dt,-255,255);
  targetL==0 ? stopMotor1() : setMotor1(out1); lastErr1=err1;

  // M2 — der adelante
  kp=targetR>=0?Kp2f:Kp2r; ki=targetR>=0?Ki2f:Ki2r; kd=targetR>=0?Kd2f:Kd2r;
  err2=targetR-rpm2; integ2=constrain(integ2+err2*dt,-400,400);
  out2=constrain(kp*err2+ki*integ2+kd*(err2-lastErr2)/dt,-255,255);
  targetR==0 ? stopMotor2() : setMotor2(out2); lastErr2=err2;

  // M3 — izq atrás
  kp=targetL>=0?Kp3f:Kp3r; ki=targetL>=0?Ki3f:Ki3r; kd=targetL>=0?Kd3f:Kd3r;
  err3=targetL-rpm3; integ3=constrain(integ3+err3*dt,-400,400);
  out3=constrain(kp*err3+ki*integ3+kd*(err3-lastErr3)/dt,-255,255);
  targetL==0 ? stopMotor3() : setMotor3(out3); lastErr3=err3;

  // M4 — der atrás
  kp=targetR>=0?Kp4f:Kp4r; ki=targetR>=0?Ki4f:Ki4r; kd=targetR>=0?Kd4f:Kd4r;
  err4=targetR-rpm4; integ4=constrain(integ4+err4*dt,-400,400);
  out4=constrain(kp*err4+ki*integ4+kd*(err4-lastErr4)/dt,-255,255);
  targetR==0 ? stopMotor4() : setMotor4(out4); lastErr4=err4;
}

// =====================================================
//  MOTORES
// =====================================================
void setMotor1(float s){
  if(s>0){motorDir1=1;digitalWrite(AIN1_H1,HIGH);digitalWrite(AIN2_H1,LOW);}
  else{motorDir1=-1;digitalWrite(AIN1_H1,LOW);digitalWrite(AIN2_H1,HIGH);}
  analogWrite(PWMA_H1,(int)abs(s));
}
void stopMotor1(){digitalWrite(AIN1_H1,LOW);digitalWrite(AIN2_H1,LOW);analogWrite(PWMA_H1,0);integ1=0;motorDir1=1;}

void setMotor2(float s){
  if(s>0){motorDir2=1;digitalWrite(BIN1_H1,HIGH);digitalWrite(BIN2_H1,LOW);}
  else{motorDir2=-1;digitalWrite(BIN1_H1,LOW);digitalWrite(BIN2_H1,HIGH);}
  analogWrite(PWMB_H1,(int)abs(s));
}
void stopMotor2(){digitalWrite(BIN1_H1,LOW);digitalWrite(BIN2_H1,LOW);analogWrite(PWMB_H1,0);integ2=0;motorDir2=1;}

void setMotor3(float s){
  if(s>0){motorDir3=1;digitalWrite(AIN1_H2,HIGH);digitalWrite(AIN2_H2,LOW);}
  else{motorDir3=-1;digitalWrite(AIN1_H2,LOW);digitalWrite(AIN2_H2,HIGH);}
  analogWrite(PWMA_H2,(int)abs(s));
}
void stopMotor3(){digitalWrite(AIN1_H2,LOW);digitalWrite(AIN2_H2,LOW);analogWrite(PWMA_H2,0);integ3=0;motorDir3=1;}

void setMotor4(float s){
  if(s>0){motorDir4=1;digitalWrite(BIN1_H2,HIGH);digitalWrite(BIN2_H2,LOW);}
  else{motorDir4=-1;digitalWrite(BIN1_H2,LOW);digitalWrite(BIN2_H2,HIGH);}
  analogWrite(PWMB_H2,(int)abs(s));
}
void stopMotor4(){digitalWrite(BIN1_H2,LOW);digitalWrite(BIN2_H2,LOW);analogWrite(PWMB_H2,0);integ4=0;motorDir4=1;}

void stopAll(){stopMotor1();stopMotor2();stopMotor3();stopMotor4();}

// =====================================================
//  ISR ENCODERS
// =====================================================
void isr1(){encPos1+=(digitalRead(ENC1B)==HIGH)?1:-1;}
void isr2(){encPos2+=(digitalRead(ENC2B)==HIGH)?1:-1;}
void isr3(){encPos3+=(digitalRead(ENC3B)==HIGH)?1:-1;}
void isr4(){encPos4+=(digitalRead(ENC4B)==HIGH)?1:-1;}

// ── Helpers ──
void resetPID_L(){integ1=0;lastErr1=0;integ3=0;lastErr3=0;}
void resetPID_R(){integ2=0;lastErr2=0;integ4=0;lastErr4=0;}
