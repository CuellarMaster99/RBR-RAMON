#include <Arduino.h>

// ---------------------------------------------------------------
// CONFIGURACIÓN DE PINES (TUS PINES REALES)
// ---------------------------------------------------------------

// HC-SR04
#define TRIG_R  4
#define ECHO_R  18
#define TRIG_L  16
#define ECHO_L  19
#define TRIG_F  23
#define ECHO_F  25

// L298N
#define ENA_PIN 14   // Motor derecho (PWM)
#define IN1_PIN 13
#define IN2_PIN 12
#define ENB_PIN 26   // Motor izquierdo (PWM)
#define IN3_PIN 27
#define IN4_PIN 33

// ---------------------------------------------------------------
// PARÁMETROS
// ---------------------------------------------------------------
#define SOUND_CM_PER_US 0.0343f
#define ECHO_TIMEOUT_US 25000UL
#define FILTER_SAMPLES 5

// Umbrales del laberinto
const float TH_FRONT_BLOCK = 10.0f;   // obstáculo al frente
const float WALL_LEFT      = 5.0f;    // muy cerca a izquierda
const float WALL_RIGHT     = 5.0f;    // muy cerca a derecha

// Velocidades
#define DUTY_FWD   180
#define DUTY_TURN  180

// ---------------------------------------------------------------
// ENUMERACIONES
// ---------------------------------------------------------------

enum Mode { ACT_STOP, ACT_FWD, ACT_LEFT, ACT_RIGHT };
enum State { S_INIT, S_NAV, S_ERROR };

Mode prevAction = ACT_STOP;
Mode currentAction = ACT_STOP;

State state = S_INIT;

// ---------------------------------------------------------------
// UTILIDADES DE MEDICIÓN
// ---------------------------------------------------------------
float pulseDistCM(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  uint32_t dur = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (!dur) return NAN;

  return 0.5f * dur * SOUND_CM_PER_US;
}

float distMediana(uint8_t trig, uint8_t echo) {
  float d[FILTER_SAMPLES];
  int k = 0;

  for (int i = 0; i < FILTER_SAMPLES; i++) {
    float x = pulseDistCM(trig, echo);
    if (!isnan(x)) d[k++] = x;
    delay(5);
  }

  if (k == 0) return NAN;

  // ordenar para obtener la mediana
  for (int i = 0; i < k; i++)
    for (int j = i + 1; j < k; j++)
      if (d[j] < d[i]) { float t = d[i]; d[i] = d[j]; d[j] = t; }

  return d[k / 2];
}

// ---------------------------------------------------------------
// MOTORES
// ---------------------------------------------------------------
void stopMotors() {
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
}

void forward() {
  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH);
  analogWrite(ENA_PIN, DUTY_FWD);
  analogWrite(ENB_PIN, DUTY_FWD);
}

void turnLeft() {
  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, DUTY_TURN);
  analogWrite(ENB_PIN, DUTY_TURN);
}

void turnRight() {
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, HIGH);
  analogWrite(ENA_PIN, DUTY_TURN);
  analogWrite(ENB_PIN, DUTY_TURN);
}

// ---------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);

  stopMotors();
  delay(1000);
  state = S_NAV;
}

// ---------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------
void loop() {

  // ==========================
  // LECTURA SENSORES
  // ==========================
  float dL = distMediana(TRIG_L, ECHO_L);
  float dF = distMediana(TRIG_F, ECHO_F);
  float dR = distMediana(TRIG_R, ECHO_R);

  /////////////////////////////////
  //     MAQUINA DE ESTADOS
  /////////////////////////////////
  switch (state) {

  // ------------------------------------------------------
  // ESTADO PRINCIPAL DE NAVEGACIÓN (S_NAV)
  // ------------------------------------------------------
  case S_NAV: {
    Mode decision = ACT_STOP;

    // --------------------------
    // 1. LÓGICA DE DECISIÓN PURO
    // --------------------------

    if (!isnan(dF) && dF <= TH_FRONT_BLOCK) {
      decision = ACT_LEFT; // obstáculo al frente → girar izquierda
    }
    else if (!isnan(dL) && dL < WALL_LEFT) {
      decision = ACT_RIGHT; // muy cerca izquierda → corregir derecha
    }
    else if (!isnan(dR) && dR < WALL_RIGHT) {
      decision = ACT_LEFT; // muy cerca derecha → corregir izquierda
    }
    else {
      decision = ACT_FWD;
    }

    // --------------------------------------------
    // 2. MEMORIA PARA EVITAR ACTUACIONES PÉSIMAS
    // --------------------------------------------

    // (A) evitar giro repetido dos veces seguidas
    if ((decision == ACT_LEFT && prevAction == ACT_LEFT) ||
        (decision == ACT_RIGHT && prevAction == ACT_RIGHT)) {

      decision = ACT_FWD;  // estabilizar avanzando
    }

    // (B) evitar zig-zag LEFT ↔ RIGHT
    if ((decision == ACT_LEFT && prevAction == ACT_RIGHT) ||
        (decision == ACT_RIGHT && prevAction == ACT_LEFT)) {

      decision = ACT_FWD;
    }

    // --------------------------
    // 3. APLICAR ACCIÓN FINAL
    // --------------------------
    switch (decision) {
      case ACT_FWD:  forward();    break;
      case ACT_LEFT: turnLeft();   break;
      case ACT_RIGHT: turnRight(); break;
      default:       stopMotors(); break;
    }

    // Guardar memoria
    prevAction = currentAction;
    currentAction = decision;

    // debug
    Serial.print("dL="); Serial.print(dL);
    Serial.print(" dF="); Serial.print(dF);
    Serial.print(" dR="); Serial.print(dR);
    Serial.print(" | Action=");
    Serial.println(currentAction);

    break;
  }

  // ------------------------------------------------------
  // ESTADO DE ERROR (por si se quieren agregar fallos)
  // ------------------------------------------------------
  case S_ERROR:
    stopMotors();
    break;

  } // fin switch state

  delay(20);
}