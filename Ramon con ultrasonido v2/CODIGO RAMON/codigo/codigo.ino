// ESP32 + L298N + 3x HC-SR04 (Izq–Frente–Der)
// --------------------------------------------
// Fase 1:
//   Avanza (M_FWD) si dF > 7.5 cm (ya NO depende de dL, dR).
//
// Fase 2 (una sola vez):
//   Si dF <= 8.5 -> giro IZQUIERDA 1.411 s -> STOP 1000 ms.
//
// Fase 3 (una sola vez):
//   Solo después de Fase 2 (giroFase2Hecho == true).
//   Cuando dF <= 10.13 cm (solo condición frontal):
//        -> STOP suave
//        -> Giro IZQUIERDA 1.411 s
//        -> STOP suave
//        -> Luego vuelve a la lógica normal (Fase 1, nunca repite Fase 3).
//
// Mapeo motores:
// RIGHT = IN1/IN2 (ENA)
// LEFT  = IN3/IN4 (ENB)

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
#error "Selecciona ESP32 Dev Module en Herramientas > Placa."
#endif

// ---------- HC-SR04 ----------
#define TRIG_R  4
#define ECHO_R  18   // usar divisor 5V->3.3V
#define TRIG_L  16
#define ECHO_L  19   // usar divisor
#define TRIG_F  23
#define ECHO_F  25   // usar divisor

// ---------- L298N ----------
#define ENA_PIN 14   // RIGHT/DERECHA = IN1/IN2 (PWM)
#define IN1_PIN 13
#define IN2_PIN 12   // si da problemas de boot, cámbialo por otro GPIO (ej. 5)
#define ENB_PIN 26   // LEFT = IN3/IN4 (PWM)
#define IN3_PIN 27
#define IN4_PIN 33

// ---------- Parámetros ----------
const float TH_OPEN = 7.5f;                // umbral "apertura" general para avanzar
#define FILTER_SAMPLES   5                 // mediana de 5
const float SOUND_CM_PER_US    = 0.0343f;  // cm/us
const uint32_t ECHO_TIMEOUT_US = 25000UL;  // 25 ms

// FASE 2: umbral frontal para iniciar giro por tiempo
const float TH_TURN_F = 8.5f;              // cuando dF <= 8.5 cm
#define PHASE2_TURN_MS      1411           // ~1.411 s de giro a la IZQUIERDA
#define POST_TURN_STOP_MS   1000           // 1 s de stop suave después del giro

// FASE 3: condición frontal y tiempo de giro
const float TH_PHASE3_F   = 10.13f;        // condición dF <= 10.13 cm
#define PHASE3_TURN_MS     1411            // otro giro IZQUIERDA de 1.411 s

// ---------- Rangos geométricos del pasillo (ya no usados en Fase 1, pero los dejamos) ----------
const float DL_MIN_LEFT   = 4.3f;
const float DL_MAX_RIGHT  = 9.0f;
const float DR_MIN_RIGHT  = 3.3f;
const float DR_MAX_LEFT   = 8.0f;

// PWM
#define DUTY_FWD       200
#define DUTY_TURN      200
#define RAMP_STEP_DUTY   6
#define RAMP_STEP_MS     8

// === PARO ANTES DE GIRAR ===
#define STOP_BEFORE_TURN_MS 200

// (Opcional) Autotest inicial
//#define DO_SELFTEST
#define SELFTEST_FWD_MS   1200
#define SELFTEST_LEFT_MS   700
#define SELFTEST_RIGHT_MS  700

// Modos
enum Mode { M_FWD, M_LEFT, M_RIGHT, M_STOP };
Mode modo = M_STOP;

int dutyRight = 0;
int dutyLeft  = 0;

// FASE 2
bool giroFase2Hecho  = false;      // para que solo se ejecute Fase 2 una vez
bool fase2Activa     = false;
unsigned long fase2StartMs = 0;

// FASE 3
bool fase3Hecha      = false;      // para que solo se ejecute Fase 3 una vez

// ---------- Helpers PWM ----------
inline void writeDuty(uint8_t pin, int duty){
  duty = constrain(duty, 0, 255);
  analogWrite(pin, duty);
}

void rampToPin(uint8_t pin, int &currDuty, int targetDuty){
  targetDuty = constrain(targetDuty, 0, 255);
  if (currDuty == targetDuty) { writeDuty(pin, targetDuty); return; }
  int step = (targetDuty > currDuty) ? RAMP_STEP_DUTY : -RAMP_STEP_DUTY;
  for (int v = currDuty; (step>0)? v <= targetDuty : v >= targetDuty; v += step){
    writeDuty(pin, v);
    currDuty = v;
    delay(RAMP_STEP_MS);
  }
}

// ---------- Motores ----------
void setRight(int duty, int dir){
  if (dir>0){ digitalWrite(IN1_PIN,HIGH); digitalWrite(IN2_PIN,LOW); }
  else if (dir<0){ digitalWrite(IN1_PIN,LOW);  digitalWrite(IN2_PIN,HIGH); }
  else { digitalWrite(IN1_PIN,LOW); digitalWrite(IN2_PIN,LOW); }
  writeDuty(ENA_PIN, duty);
  dutyRight = constrain(duty, 0, 255);
}

void setLeft(int duty, int dir){
  if (dir>0){ digitalWrite(IN3_PIN,HIGH); digitalWrite(IN4_PIN,LOW); }
  else if (dir<0){ digitalWrite(IN3_PIN,LOW);  digitalWrite(IN4_PIN,HIGH); }
  else { digitalWrite(IN3_PIN,LOW); digitalWrite(IN4_PIN,LOW); }
  writeDuty(ENB_PIN, duty);
  dutyLeft = constrain(duty, 0, 255);
}

void softStop(){
  rampToPin(ENA_PIN, dutyRight, 0);
  rampToPin(ENB_PIN, dutyLeft,  0);
  digitalWrite(IN1_PIN,LOW); digitalWrite(IN2_PIN,LOW);
  digitalWrite(IN3_PIN,LOW); digitalWrite(IN4_PIN,LOW);
  modo = M_STOP;
}

void applyMode(Mode m){
  switch(m){
    case M_FWD:
      digitalWrite(IN1_PIN,HIGH); digitalWrite(IN2_PIN,LOW);
      digitalWrite(IN3_PIN,LOW);  digitalWrite(IN4_PIN,HIGH);
      rampToPin(ENA_PIN, dutyRight, DUTY_FWD);
      rampToPin(ENB_PIN, dutyLeft,  DUTY_FWD);
      break;

    case M_LEFT:
      digitalWrite(IN1_PIN,HIGH); digitalWrite(IN2_PIN,LOW);
      digitalWrite(IN3_PIN,HIGH); digitalWrite(IN4_PIN,LOW);
      rampToPin(ENA_PIN, dutyRight, DUTY_TURN);
      rampToPin(ENB_PIN, dutyLeft,  DUTY_TURN);
      break;

    case M_RIGHT:
      digitalWrite(IN1_PIN,LOW);  digitalWrite(IN2_PIN,HIGH);
      digitalWrite(IN3_PIN,LOW);  digitalWrite(IN4_PIN,HIGH);
      rampToPin(ENA_PIN, dutyRight, DUTY_TURN);
      rampToPin(ENB_PIN, dutyLeft,  DUTY_TURN);
      break;

    default:
      softStop();
      break;
  }
  modo = m;
}

// ---------- Medición ----------
float pulseDistCM(uint8_t trig, uint8_t echo){
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  uint32_t dur = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (!dur) return NAN;
  return 0.5f * dur * SOUND_CM_PER_US;
}

float distMediana(uint8_t trig, uint8_t echo, uint8_t n=FILTER_SAMPLES){
  float v[9];
  n = constrain(n,(uint8_t)1,(uint8_t)9);
  uint8_t k=0;
  for(uint8_t i=0;i<n;i++){
    float d = pulseDistCM(trig, echo);
    if(!isnan(d)) v[k++] = d;
    delay(6);
  }
  if (!k) return NAN;

  for(uint8_t i=0;i<k;i++)
    for(uint8_t j=i+1;j<k;j++)
      if(v[j] < v[i]){ float t=v[i]; v[i]=v[j]; v[j]=t; }

  return v[k/2];
}

// ---------- Geometría (no usada en Fase 1, pero la dejamos por si acaso) ----------
bool dentroPasillo(float dL, float dR){
  if (isnan(dL) || isnan(dR)) return false;
  return (dL >= DL_MIN_LEFT && dL <= DL_MAX_RIGHT &&
          dR >= DR_MIN_RIGHT && dR <= DR_MAX_LEFT);
}

// ---------- Telemetría ----------
unsigned long lastPrint = 0;
const unsigned long PRINT_EVERY_MS = 200;

// ---------- SETUP ----------
void setup(){
  Serial.begin(115200);
  delay(200);

  pinMode(TRIG_R,OUTPUT); pinMode(ECHO_R,INPUT);
  pinMode(TRIG_L,OUTPUT); pinMode(ECHO_L,INPUT);
  pinMode(TRIG_F,OUTPUT); pinMode(ECHO_F,INPUT);

  pinMode(IN1_PIN,OUTPUT); pinMode(IN2_PIN,OUTPUT);
  pinMode(IN3_PIN,OUTPUT); pinMode(IN4_PIN,OUTPUT);

  pinMode(ENA_PIN,OUTPUT);
  pinMode(ENB_PIN,OUTPUT);

  applyMode(M_STOP);
}

// ---------- LOOP ----------
void loop(){

  float dR = distMediana(TRIG_R, ECHO_R);
  float dL = distMediana(TRIG_L, ECHO_L);
  float dF = distMediana(TRIG_F, ECHO_F);

  // ========= FASE 2: GIRO A LA IZQUIERDA UNA SOLA VEZ =========
  if(!fase2Activa && !giroFase2Hecho && !isnan(dF) && dF <= TH_TURN_F){
    Serial.println("FASE 2: dF <= 8.5cm -> giro IZQUIERDA 1.411s, STOP 1s");

    // Stop previo al giro
    softStop();
    delay(STOP_BEFORE_TURN_MS);

    // Arrancar giro a la izquierda
    applyMode(M_LEFT);
    fase2Activa  = true;
    fase2StartMs = millis();
  }

  if (fase2Activa){
    if (millis() - fase2StartMs >= PHASE2_TURN_MS){
      // Terminar giro → stop suave + pausa 1s
      softStop();
      delay(POST_TURN_STOP_MS);

      giroFase2Hecho = true;   // marcar que ya se hizo la fase 2
      fase2Activa    = false;
      // Luego ya puede entrar Fase 3 o seguir en Fase 1
    }

    // Mientras está girando, no ejecutamos lógica de navegación
    unsigned long now = millis();
    if (now - lastPrint >= PRINT_EVERY_MS){
      lastPrint = now;
      Serial.print("FASE2 girando LEFT... dF=");
      Serial.println(isnan(dF)? -1 : dF);
    }
    delay(20);
    return;
  }

  // ========= FASE 3: GIRO IZQUIERDA 1.411s CUANDO dF <= 10.13 (una sola vez) =========
  // Solo después de que Fase 2 ya sucedió.
  if (giroFase2Hecho && !fase3Hecha && !isnan(dF) && dF <= TH_PHASE3_F){
    Serial.println("FASE 3: dF <= 10.13cm -> giro IZQUIERDA 1.411s");

    // Stop suave antes de girar
    softStop();
    delay(STOP_BEFORE_TURN_MS);

    // Giro a la izquierda durante PHASE3_TURN_MS
    applyMode(M_LEFT);
    delay(PHASE3_TURN_MS);

    // Stop suave al terminar el giro
    softStop();

    // Marcar que Fase 3 ya se hizo (no se repite)
    fase3Hecha = true;
    // Después de esto, el robot quedará en STOP y en el siguiente ciclo aplicará Fase 1.
  }

  // ========= FASE 1: Avance normal (SOLO depende de dF) =========
  Mode target = M_STOP;

  if(!isnan(dF) && dF > TH_OPEN){
    target = M_FWD;
  }

  if(target != modo){
    applyMode(target);
  }

  unsigned long now = millis();
  if (now - lastPrint >= PRINT_EVERY_MS){
    lastPrint = now;
    Serial.print("dL="); Serial.print(isnan(dL)? -1 : dL);
    Serial.print("  dF="); Serial.print(isnan(dF)? -1 : dF);
    Serial.print("  dR="); Serial.print(isnan(dR)? -1 : dR);
    Serial.print("  | modo=");
    const char* n[]={"FWD","LEFT","RIGHT","STOP"};
    Serial.println(n[modo]);
  }

  delay(20);
}
