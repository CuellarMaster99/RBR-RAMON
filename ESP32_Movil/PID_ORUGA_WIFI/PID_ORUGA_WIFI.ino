// =====================================================
//   ESP32 CONTROL DOBLE MOTOR + PID + WiFi WebSocket
//   Protocolo JSON bidireccional
//   v4.0 - WiFi WebSocket, sin cable USB
// =====================================================
//
// DEPENDENCIAS (instalar en Arduino IDE):
//   - WebSocketsServer  by Markus Sattler  (v2.x)
//   - ArduinoJson       by Benoit Blanchon (v7.x)
//
// CONEXION:
//   1. Sube este sketch al ESP32
//   2. Abre Serial Monitor a 115200
//   3. Verás la IP asignada por el router
//   4. Copia esa IP al script Python (ESP32_IP)
// =====================================================

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// =====================================================
//  *** CAMBIA ESTOS DATOS AL DE TU RED WiFi ***
// =====================================================
const char* WIFI_SSID = "FAMILIA MENDEZ_2";
const char* WIFI_PASS = "1144101738";

// =====================================================
// PROTOTIPOS ISR
// =====================================================
void IRAM_ATTR encoder1ISR();
void IRAM_ATTR encoder2ISR();

// =====================================================
// SERVIDOR WEBSOCKET  (puerto 81)
// =====================================================
WebSocketsServer wsServer(81);

// =====================================================
// PINES MOTOR 1
// =====================================================
#define ENA        25
#define IN1        26
#define IN2        27
#define ENCODER1_A 34
#define ENCODER1_B 35

// =====================================================
// PINES MOTOR 2
// =====================================================
#define ENB        33
#define IN3        14
#define IN4        12
#define ENCODER2_A 32
#define ENCODER2_B 13

// =====================================================
// CONFIGURACION PWM
// =====================================================
#define PWM_FREQ       31000
#define PWM_RESOLUTION 8

// =====================================================
// ENCODERS
// =====================================================
volatile long encoderPos1 = 0;
volatile long encoderPos2 = 0;
long lastEncoderPos1 = 0;
long lastEncoderPos2 = 0;
const int PPR = 234;

int motorDir1 = 1;
int motorDir2 = 1;

// =====================================================
// TIEMPO
// =====================================================
unsigned long lastTime      = 0;
unsigned long lastBroadcast = 0;
const unsigned long BROADCAST_INTERVAL = 20; // ms entre envios JSON

// =====================================================
// VELOCIDADES
// =====================================================
float rpm1 = 0, rpmFiltrada1 = 0, targetRPM1 = 0;
float rpm2 = 0, rpmFiltrada2 = 0, targetRPM2 = 0;

// =====================================================
// GANANCIAS PID
// =====================================================
float Kp1_fwd = 1.35, Ki1_fwd = 0.500, Kd1_fwd = 0.038;
float Kp1_rev = 1.35, Ki1_rev = 0.545, Kd1_rev = 0.038;
float Kp2_fwd = 1.35, Ki2_fwd = 0.545, Kd2_fwd = 0.028;
float Kp2_rev = 1.35, Ki2_rev = 0.545, Kd2_rev = 0.028;

// =====================================================
// ESTADO PID
// =====================================================
float error1 = 0, lastError1 = 0, integral1 = 0, output1 = 0;
float error2 = 0, lastError2 = 0, integral2 = 0, output2 = 0;

// =====================================================
// SYNC / OTROS
// =====================================================
bool  syncMode  = false;
float Kp_sync   = 0.5;
bool  fixSign   = true;
int   connectedClient = -1;   // cliente WS actual

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Pines puente H
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Encoders
  pinMode(ENCODER1_A, INPUT); pinMode(ENCODER1_B, INPUT);
  pinMode(ENCODER2_A, INPUT); pinMode(ENCODER2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2ISR, RISING);

  // PWM
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  stopMotor1(); stopMotor2();

  // ── Conectar WiFi ──
  Serial.printf("\nConectando a WiFi: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400); Serial.print(".");
  }
  Serial.printf("\n✓ WiFi conectado\n");
  Serial.printf("  IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("  WebSocket: ws://%s:81\n\n", WiFi.localIP().toString().c_str());

  // ── WebSocket ──
  wsServer.begin();
  wsServer.onEvent(wsEvent);

  lastTime = millis();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  wsServer.loop();   // CRÍTICO: procesar eventos WS

  unsigned long now = millis();

  if (now - lastTime >= 20) {     // 50 Hz control
    calcularRPM();
    pidControl();
    lastTime = now;
  }

  if (now - lastBroadcast >= BROADCAST_INTERVAL) {
    broadcastStatus();
    lastBroadcast = now;
  }
}

// =====================================================
// EVENTO WEBSOCKET
// =====================================================
void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED: {
      IPAddress ip = wsServer.remoteIP(num);
      Serial.printf("[WS] Cliente #%u conectado  IP: %s\n", num, ip.toString().c_str());
      connectedClient = num;

      // Enviar config actual al nuevo cliente
      sendConfig(num);
      break;
    }

    case WStype_DISCONNECTED:
      Serial.printf("[WS] Cliente #%u desconectado\n", num);
      if (connectedClient == num) {
        connectedClient = -1;
        // Seguridad: detener motores si se desconecta el cliente
        targetRPM1 = 0; targetRPM2 = 0;
        stopMotor1(); stopMotor2();
      }
      break;

    case WStype_TEXT: {
      // Parsear JSON recibido
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, payload, length);
      if (err) {
        Serial.printf("[WS] JSON inválido: %s\n", err.c_str());
        sendError(num, "JSON inválido");
        return;
      }
      processJSON(num, doc);
      break;
    }

    default:
      break;
  }
}

// =====================================================
// PROCESAR COMANDO JSON ENTRANTE
//
// Formato esperado (ejemplos):
//
//   Setpoint individual:
//     {"cmd":"set","m1":80}
//     {"cmd":"set","m2":-50}
//     {"cmd":"set","m1":80,"m2":80}
//
//   Stop:
//     {"cmd":"stop"}
//     {"cmd":"stop","motor":1}   <- solo motor 1
//
//   PID avance:
//     {"cmd":"pid","dir":"fwd","motor":1,"kp":1.35,"ki":0.5,"kd":0.038}
//
//   PID retroceso:
//     {"cmd":"pid","dir":"rev","motor":2,"kp":1.35,"ki":0.545,"kd":0.028}
//
//   Sincronización:
//     {"cmd":"sync","enable":true}
//     {"cmd":"sync","enable":false}
//     {"cmd":"sync","kp":0.5}
//
//   Fix sign:
//     {"cmd":"fixsign","enable":true}
//
//   Consulta de estado:
//     {"cmd":"status"}
//
//   Reset encoders:
//     {"cmd":"reset"}
// =====================================================
void processJSON(uint8_t num, JsonDocument& doc) {

  const char* cmd = doc["cmd"] | "";

  // ── SETPOINT ──────────────────────────────────────
  if (strcmp(cmd, "set") == 0) {
    if (doc.containsKey("m1")) {
      targetRPM1 = constrain((float)doc["m1"], -150.0f, 150.0f);
    }
    if (doc.containsKey("m2")) {
      targetRPM2 = constrain((float)doc["m2"], -150.0f, 150.0f);
    }
    Serial.printf("[CMD] set  M1=%.1f  M2=%.1f\n", targetRPM1, targetRPM2);
    sendAck(num, "set");
  }

  // ── STOP ──────────────────────────────────────────
  else if (strcmp(cmd, "stop") == 0) {
    int motor = doc["motor"] | 0;   // 0 = ambos, 1 o 2 = individual
    if (motor == 0 || motor == 1) { targetRPM1 = 0; stopMotor1(); }
    if (motor == 0 || motor == 2) { targetRPM2 = 0; stopMotor2(); }
    Serial.printf("[CMD] stop  motor=%d\n", motor);
    sendAck(num, "stop");
  }

  // ── PID ───────────────────────────────────────────
  else if (strcmp(cmd, "pid") == 0) {
    int   motor = doc["motor"] | 0;
    const char* dir = doc["dir"] | "fwd";   // "fwd" o "rev"
    bool isFwd = (strcmp(dir, "fwd") == 0);

    if (doc.containsKey("kp") || doc.containsKey("ki") || doc.containsKey("kd")) {
      if (motor == 1 || motor == 0) {
        float& Kp = isFwd ? Kp1_fwd : Kp1_rev;
        float& Ki = isFwd ? Ki1_fwd : Ki1_rev;
        float& Kd = isFwd ? Kd1_fwd : Kd1_rev;
        if (doc.containsKey("kp")) Kp = (float)doc["kp"];
        if (doc.containsKey("ki")) Ki = (float)doc["ki"];
        if (doc.containsKey("kd")) Kd = (float)doc["kd"];
        Serial.printf("[CMD] PID M1 %s  Kp=%.4f Ki=%.4f Kd=%.4f\n",
                      dir, Kp, Ki, Kd);
      }
      if (motor == 2 || motor == 0) {
        float& Kp = isFwd ? Kp2_fwd : Kp2_rev;
        float& Ki = isFwd ? Ki2_fwd : Ki2_rev;
        float& Kd = isFwd ? Kd2_fwd : Kd2_rev;
        if (doc.containsKey("kp")) Kp = (float)doc["kp"];
        if (doc.containsKey("ki")) Ki = (float)doc["ki"];
        if (doc.containsKey("kd")) Kd = (float)doc["kd"];
        Serial.printf("[CMD] PID M2 %s  Kp=%.4f Ki=%.4f Kd=%.4f\n",
                      dir, Kp, Ki, Kd);
      }
    }
    sendAck(num, "pid");
  }

  // ── SYNC ──────────────────────────────────────────
  else if (strcmp(cmd, "sync") == 0) {
    if (doc.containsKey("enable")) {
      syncMode = (bool)doc["enable"];
      Serial.printf("[CMD] sync  enable=%d\n", syncMode);
    }
    if (doc.containsKey("kp")) {
      Kp_sync = (float)doc["kp"];
      Serial.printf("[CMD] sync  kp=%.4f\n", Kp_sync);
    }
    sendAck(num, "sync");
  }

  // ── FIXSIGN ───────────────────────────────────────
  else if (strcmp(cmd, "fixsign") == 0) {
    fixSign = (bool)doc["enable"];
    Serial.printf("[CMD] fixsign  enable=%d\n", fixSign);
    sendAck(num, "fixsign");
  }

  // ── STATUS ────────────────────────────────────────
  else if (strcmp(cmd, "status") == 0) {
    sendConfig(num);
  }

  // ── RESET ENCODERS ────────────────────────────────
  else if (strcmp(cmd, "reset") == 0) {
    noInterrupts();
    encoderPos1 = encoderPos2 = 0;
    lastEncoderPos1 = lastEncoderPos2 = 0;
    interrupts();
    Serial.println("[CMD] reset encoders");
    sendAck(num, "reset");
  }

  // ── PING ──────────────────────────────────────────
  else if (strcmp(cmd, "ping") == 0) {
    sendAck(num, "pong");
  }

  else {
    Serial.printf("[CMD] desconocido: %s\n", cmd);
    sendError(num, "Comando desconocido");
  }
}

// =====================================================
// ENVIAR JSON DE ESTADO EN TIEMPO REAL (broadcast)
//
// Formato:
// {
//   "type": "telemetry",
//   "rpm1": 78.3,    "target1": 80.0,
//   "err1": 1.7,     "out1": 124.5,
//   "rpm2": 77.9,    "target2": 80.0,
//   "err2": 2.1,     "out2": 126.1,
//   "sync": false
// }
// =====================================================
void broadcastStatus() {
  if (connectedClient < 0) return;   // sin cliente, no enviar

  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"telemetry\","
    "\"rpm1\":%.2f,\"target1\":%.1f,\"err1\":%.2f,\"out1\":%.1f,"
    "\"rpm2\":%.2f,\"target2\":%.1f,\"err2\":%.2f,\"out2\":%.1f,"
    "\"sync\":%s}",
    rpm1, targetRPM1, error1, output1,
    rpm2, targetRPM2, error2, output2,
    syncMode ? "true" : "false"
  );
  wsServer.sendTXT(connectedClient, buf);
}

// =====================================================
// ENVIAR ACK
// { "type":"ack", "cmd":"stop" }
// =====================================================
void sendAck(uint8_t num, const char* cmdName) {
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"type\":\"ack\",\"cmd\":\"%s\"}", cmdName);
  wsServer.sendTXT(num, buf);
}

// =====================================================
// ENVIAR ERROR
// { "type":"error", "msg":"..." }
// =====================================================
void sendError(uint8_t num, const char* msg) {
  char buf[128];
  snprintf(buf, sizeof(buf), "{\"type\":\"error\",\"msg\":\"%s\"}", msg);
  wsServer.sendTXT(num, buf);
}

// =====================================================
// ENVIAR CONFIGURACION COMPLETA
// { "type":"config", "pid":{ ... }, "sync":... }
// =====================================================
void sendConfig(uint8_t num) {
  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"config\","
    "\"pid\":{"
      "\"m1fwd\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},"
      "\"m1rev\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},"
      "\"m2fwd\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},"
      "\"m2rev\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f}},"
    "\"sync\":%s,\"kpsync\":%.4f,\"fixsign\":%s}",
    Kp1_fwd, Ki1_fwd, Kd1_fwd,
    Kp1_rev, Ki1_rev, Kd1_rev,
    Kp2_fwd, Ki2_fwd, Kd2_fwd,
    Kp2_rev, Ki2_rev, Kd2_rev,
    syncMode ? "true" : "false", Kp_sync,
    fixSign  ? "true" : "false"
  );
  wsServer.sendTXT(num, buf);
}

// =====================================================
// CALCULAR RPM
// =====================================================
void calcularRPM() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0f;
  if (deltaTime <= 0) return;

  long pos1, pos2;
  noInterrupts(); pos1 = encoderPos1; pos2 = encoderPos2; interrupts();

  long delta1 = pos1 - lastEncoderPos1;
  long delta2 = pos2 - lastEncoderPos2;
  lastEncoderPos1 = pos1; lastEncoderPos2 = pos2;

  float rawRpm1 = (delta1 * 60.0f) / (PPR * deltaTime);
  float rawRpm2 = (delta2 * 60.0f) / (PPR * deltaTime);

  rpm1 = fixSign ? rawRpm1 * motorDir1 : rawRpm1;
  rpm2 = fixSign ? rawRpm2 * motorDir2 : rawRpm2;

  rpmFiltrada1 = 0.7f * rpmFiltrada1 + 0.3f * rpm1;
  rpmFiltrada2 = 0.7f * rpmFiltrada2 + 0.3f * rpm2;
  rpm1 = rpmFiltrada1; rpm2 = rpmFiltrada2;
}

// =====================================================
// GANANCIAS PID SEGUN DIRECCION
// =====================================================
void getGains1(float& Kp, float& Ki, float& Kd) {
  Kp = (targetRPM1 >= 0) ? Kp1_fwd : Kp1_rev;
  Ki = (targetRPM1 >= 0) ? Ki1_fwd : Ki1_rev;
  Kd = (targetRPM1 >= 0) ? Kd1_fwd : Kd1_rev;
}

void getGains2(float& Kp, float& Ki, float& Kd) {
  Kp = (targetRPM2 >= 0) ? Kp2_fwd : Kp2_rev;
  Ki = (targetRPM2 >= 0) ? Ki2_fwd : Ki2_rev;
  Kd = (targetRPM2 >= 0) ? Kd2_fwd : Kd2_rev;
}

// =====================================================
// CONTROL PID
// =====================================================
void pidControl() {
  const float dt = 0.02f;

  // Motor 1
  float Kp1, Ki1, Kd1; getGains1(Kp1, Ki1, Kd1);
  error1     = targetRPM1 - rpm1;
  integral1  = constrain(integral1 + error1 * dt, -400, 400);
  output1    = constrain(Kp1 * error1 + Ki1 * integral1 + Kd1 * (error1 - lastError1) / dt, -255, 255);
  if (targetRPM1 == 0) stopMotor1(); else setMotor1(output1);
  lastError1 = error1;

  // Motor 2
  float Kp2, Ki2, Kd2; getGains2(Kp2, Ki2, Kd2);
  error2     = targetRPM2 - rpm2;
  integral2  = constrain(integral2 + error2 * dt, -400, 400);
  output2    = constrain(Kp2 * error2 + Ki2 * integral2 + Kd2 * (error2 - lastError2) / dt, -255, 255);

  if (syncMode && targetRPM2 != 0) {
    float sc  = Kp_sync * (abs(rpm1) - abs(rpm2));
    output2   = constrain(output2 + (output2 >= 0 ? sc : -sc), -255, 255);
  }
  if (targetRPM2 == 0) stopMotor2(); else setMotor2(output2);
  lastError2 = error2;
}

// =====================================================
// CONTROL FISICO MOTORES
// =====================================================
void setMotor1(float speed) {
  if (speed > 0)       { motorDir1=1; digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  ledcWrite(ENA,(int)fabs(speed)); }
  else if (speed < 0)  { motorDir1=-1;digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); ledcWrite(ENA,(int)fabs(speed)); }
  else stopMotor1();
}
void stopMotor1() { digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); ledcWrite(ENA,0); integral1=0; motorDir1=1; }

void setMotor2(float speed) {
  if (speed > 0)       { motorDir2=1; digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  ledcWrite(ENB,(int)fabs(speed)); }
  else if (speed < 0)  { motorDir2=-1;digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); ledcWrite(ENB,(int)fabs(speed)); }
  else stopMotor2();
}
void stopMotor2() { digitalWrite(IN3,LOW); digitalWrite(IN4,LOW); ledcWrite(ENB,0); integral2=0; motorDir2=1; }

// =====================================================
// ISR ENCODERS
// =====================================================
void IRAM_ATTR encoder1ISR() { if (digitalRead(ENCODER1_B)) encoderPos1++; else encoderPos1--; }
void IRAM_ATTR encoder2ISR() { if (digitalRead(ENCODER2_B)) encoderPos2++; else encoderPos2--; }

