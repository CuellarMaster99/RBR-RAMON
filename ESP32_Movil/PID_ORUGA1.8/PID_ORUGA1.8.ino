// =====================================================
//   ARDUINO MEGA WIFI — ATmega2560
//   PID DUAL + TB6612FNG + WEBSOCKET via AT commands
//   Compilar como: "Arduino Mega 2560"
//   NO requiere ningún núcleo extra
// =====================================================
//
//  ARQUITECTURA
//  ─────────────────────────────────────────────────
//  ATmega2560  →  PID + encoders + motores
//  Serial3 (pines 14/15 TX3/RX3)  →  ESP8266 (AT)
//  El ESP8266 actúa como módulo WiFi transparente.
//  Se comunica con el ESP8266 usando el firmware AT
//  que viene de fábrica en el Arduino Mega WiFi.
//
//  PINOUT TB6612FNG
//  ─────────────────────────────────────────────────
//  PWMA  → Pin 6   (PWM Timer4)
//  AIN1  → Pin 22
//  AIN2  → Pin 23
//  PWMB  → Pin 7   (PWM Timer4)
//  BIN1  → Pin 24
//  BIN2  → Pin 25
//  STBY  → Pin 26
//  VCC   → 5V lógica
//  VM    → 7.4–12V fuente externa
//  GND   → GND común con fuente
//
//  PINOUT ENCODERS
//  ─────────────────────────────────────────────────
//  Motor 1  Canal A → Pin 2  (INT0)
//  Motor 1  Canal B → Pin 3  (INT1 — lectura dir)
//  Motor 2  Canal A → Pin 18 (INT3)
//  Motor 2  Canal B → Pin 19 (INT4 — lectura dir)
//
//  SWITCH HARDWARE del Arduino Mega WiFi
//  ─────────────────────────────────────────────────
//  Posición  3-4  →  ATmega TX → ESP8266 RX  (Serial3)
//  Posición  5-6  →  ESP8266 TX → ATmega RX  (Serial3)
//  (consulta el diagrama de tu placa específica)
//
//  LIBRERÍAS
//  ─────────────────────────────────────────────────
//  Ninguna extra — solo Arduino estándar
//
//  PASOS PARA COMPILAR
//  ─────────────────────────────────────────────────
//  1. Tools → Board → "Arduino Mega or Mega 2560"
//  2. Tools → Processor → "ATmega2560"
//  3. Verificar y subir  (sin instalar nada extra)
// =====================================================

// ── Pines TB6612FNG ──
#define PWMA  6
#define AIN1  22
#define AIN2  23
#define PWMB  7
#define BIN1  24
#define BIN2  25
#define STBY  26

// ── Pines encoders ──
#define ENC1A  2    // INT0
#define ENC1B  3
#define ENC2A  18   // INT3
#define ENC2B  19

// ── WiFi ──
const char* SSID     = "TU_RED_WIFI";
const char* PASSWORD = "TU_CONTRASENA";
const int   WS_PORT  = 81;

// ── Encoder ──
const int PPR = 234;   // Pulsos por revolución — ajusta según tu encoder

// ── Temporización ──
#define INTERVAL_MS  20    // Ciclo PID (ms) — 50 Hz

// =====================================================
//  VARIABLES ENCODER
// =====================================================
volatile long encoderPos1 = 0;
volatile long encoderPos2 = 0;
long lastPos1 = 0, lastPos2 = 0;

int motorDir1 = 1, motorDir2 = 1;

// =====================================================
//  VELOCIDADES Y PID
// =====================================================
float rpm1 = 0, rpmF1 = 0, target1 = 0;
float rpm2 = 0, rpmF2 = 0, target2 = 0;

// PID avance
float Kp1f=1.35, Ki1f=0.500, Kd1f=0.038;
float Kp2f=1.35, Ki2f=0.545, Kd2f=0.028;
// PID retroceso
float Kp1r=1.35, Ki1r=0.545, Kd1r=0.038;
float Kp2r=1.35, Ki2r=0.545, Kd2r=0.028;

float err1=0, lastErr1=0, integ1=0, out1=0;
float err2=0, lastErr2=0, integ2=0, out2=0;

bool  syncMode = false;
float Kp_sync  = 0.5;
bool  fixSign  = true;

unsigned long lastTime = 0;

// =====================================================
//  WEBSOCKET — estado de la conexión AT
// =====================================================
// El ESP8266 en modo servidor TCP actúa como WebSocket
// "transparente": reenvía los bytes recibidos al Mega.
// Para el dashboard Python usamos un servidor TCP simple
// ya que WebSocket puro via AT es complejo.
// Protocolo: texto plano, mismo formato de comandos/datos.

bool  wifiOK    = false;
bool  serverOK  = false;
bool  clientOK  = false;   // hay un cliente TCP conectado
int   clientID  = 0;       // ID de conexión ESP8266 (0–4)

String atBuf = "";          // buffer de respuestas AT
String rxBuf = "";          // buffer de datos entrantes del cliente

// =====================================================
//  SETUP
// =====================================================
void setup() {
  // Serial USB — debug y Serial Plotter
  Serial.begin(115200);
  delay(1000);

  // Serial3 — comunicación con ESP8266
  Serial3.begin(115200);
  delay(500);

  // TB6612FNG
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);
  pinMode(STBY,OUTPUT); digitalWrite(STBY, HIGH);
  stopMotor1(); stopMotor2();

  // Encoders
  pinMode(ENC1A,INPUT_PULLUP); pinMode(ENC1B,INPUT_PULLUP);
  pinMode(ENC2A,INPUT_PULLUP); pinMode(ENC2B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1A), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A), isr2, RISING);

  // Conectar WiFi y arrancar servidor TCP
  initWiFi();

  lastTime = millis();
  printHelp();
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  // Leer respuestas / datos del ESP8266
  readESP();

  // Ciclo PID cada INTERVAL_MS
  if (millis() - lastTime >= INTERVAL_MS) {
    calcRPM();
    pidControl();
    sendTelemetry();
  }

  // Comandos por Serial USB (tuning local)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}

// =====================================================
//  WIFI — INICIALIZACIÓN (comandos AT)
// =====================================================
void initWiFi() {
  Serial.println("Iniciando ESP8266...");

  sendAT("AT",               "OK",  2000);
  sendAT("AT+RST",           "ready", 5000);
  sendAT("AT+CWMODE=1",      "OK",  2000);   // modo station

  // Conectar a la red
  String cwjap = "AT+CWJAP=\"";
  cwjap += SSID; cwjap += "\",\""; cwjap += PASSWORD; cwjap += "\"";
  bool joined = sendAT(cwjap.c_str(), "WIFI GOT IP", 15000);

  if (joined) {
    wifiOK = true;
    // Obtener y mostrar la IP
    Serial3.println("AT+CIFSR");
    delay(1000);
    while (Serial3.available()) {
      String line = Serial3.readStringUntil('\n');
      line.trim();
      if (line.indexOf("STAIP") >= 0) {
        Serial.print("IP: "); Serial.println(line);
      }
    }
    // Servidor TCP multi-conexión
    sendAT("AT+CIPMUX=1",              "OK", 2000);
    String srv = "AT+CIPSERVER=1,";
    srv += WS_PORT;
    bool ok = sendAT(srv.c_str(),      "OK", 2000);
    serverOK = ok;
    if (ok) {
      Serial.print("Servidor TCP en puerto ");
      Serial.println(WS_PORT);
      Serial.println("Dashboard Python: CONNECTION_MODE = \"wifi\"");
    }
  } else {
    Serial.println("WiFi FALLÓ — usando solo Serial USB");
  }
}

// ── Enviar comando AT y esperar respuesta ──
bool sendAT(const char* cmd, const char* expected, unsigned long timeout) {
  // Limpiar buffer
  while (Serial3.available()) Serial3.read();

  Serial3.println(cmd);
  Serial.print("[AT] "); Serial.println(cmd);

  String resp = "";
  unsigned long t = millis();
  while (millis() - t < timeout) {
    while (Serial3.available()) {
      char c = Serial3.read();
      resp += c;
    }
    if (resp.indexOf(expected) >= 0) {
      Serial.print("[OK] "); Serial.println(expected);
      return true;
    }
  }
  Serial.print("[TIMEOUT] esperando: "); Serial.println(expected);
  return false;
}

// =====================================================
//  LEER ESP8266 — datos entrantes y eventos
// =====================================================
void readESP() {
  while (Serial3.available()) {
    char c = Serial3.read();
    atBuf += c;

    // Procesar cuando llega fin de línea
    if (c == '\n') {
      atBuf.trim();

      // Cliente conectado: "+IPD,0,5:m1=50"
      if (atBuf.startsWith("+IPD,")) {
        parseIPD(atBuf);
      }
      // Cliente nuevo conectado
      else if (atBuf.indexOf(",CONNECT") >= 0) {
        // Extraer ID (primer carácter)
        clientID = atBuf.charAt(0) - '0';
        clientOK = true;
        Serial.print("[WS] Cliente conectado ID=");
        Serial.println(clientID);
      }
      // Cliente desconectado
      else if (atBuf.indexOf(",CLOSED") >= 0) {
        clientOK = false;
        Serial.println("[WS] Cliente desconectado — STOP");
        target1 = 0; target2 = 0;
        stopMotor1(); stopMotor2();
      }

      atBuf = "";
    }

    // Evitar desbordamiento del buffer
    if (atBuf.length() > 256) atBuf = "";
  }
}

// ── Parsear mensaje IPD del ESP8266 ──
// Formato: "+IPD,<id>,<len>:<datos>"
void parseIPD(const String& line) {
  int colon = line.indexOf(':');
  if (colon < 0) return;
  String data = line.substring(colon + 1);
  data.trim();
  if (data.length() > 0) {
    processCommand(data);
  }
}

// ── Enviar datos al cliente TCP (dashboard Python) ──
void tcpSend(const String& msg) {
  if (!clientOK || !wifiOK) return;
  String cmd = "AT+CIPSEND=";
  cmd += clientID; cmd += ","; cmd += (msg.length() + 2);
  Serial3.println(cmd);
  delay(20);   // esperar prompt ">"
  Serial3.print(msg); Serial3.print("\r\n");
}

// =====================================================
//  CALCULAR RPM
// =====================================================
void calcRPM() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) return;

  long p1, p2;
  noInterrupts(); p1 = encoderPos1; p2 = encoderPos2; interrupts();

  long d1 = p1 - lastPos1;
  long d2 = p2 - lastPos2;
  lastPos1 = p1; lastPos2 = p2;

  float r1 = (d1 * 60.0) / (PPR * dt);
  float r2 = (d2 * 60.0) / (PPR * dt);

  if (fixSign) { rpm1 = r1 * motorDir1; rpm2 = r2 * motorDir2; }
  else         { rpm1 = r1;             rpm2 = r2; }

  rpmF1 = 0.7 * rpmF1 + 0.3 * rpm1;
  rpmF2 = 0.7 * rpmF2 + 0.3 * rpm2;
  rpm1  = rpmF1; rpm2 = rpmF2;

  lastTime = now;
}

// =====================================================
//  PID CONTROL
// =====================================================
void pidControl() {
  float dt = INTERVAL_MS / 1000.0;

  // Seleccionar ganancias según dirección del setpoint
  float Kp1 = (target1 >= 0) ? Kp1f : Kp1r;
  float Ki1 = (target1 >= 0) ? Ki1f : Ki1r;
  float Kd1 = (target1 >= 0) ? Kd1f : Kd1r;
  float Kp2 = (target2 >= 0) ? Kp2f : Kp2r;
  float Ki2 = (target2 >= 0) ? Ki2f : Ki2r;
  float Kd2 = (target2 >= 0) ? Kd2f : Kd2r;

  // Motor 1
  err1    = target1 - rpm1;
  integ1 += err1 * dt;
  integ1  = constrain(integ1, -400.0, 400.0);
  out1    = Kp1*err1 + Ki1*integ1 + Kd1*(err1-lastErr1)/dt;
  out1    = constrain(out1, -255.0, 255.0);
  if (target1 == 0) stopMotor1(); else setMotor1(out1);
  lastErr1 = err1;

  // Motor 2
  err2    = target2 - rpm2;
  integ2 += err2 * dt;
  integ2  = constrain(integ2, -400.0, 400.0);
  out2    = Kp2*err2 + Ki2*integ2 + Kd2*(err2-lastErr2)/dt;
  out2    = constrain(out2, -255.0, 255.0);

  // Sincronización lineal
  if (syncMode && target2 != 0) {
    float corr = Kp_sync * (abs(rpm1) - abs(rpm2));
    out2 = (out2 >= 0) ? out2 + corr : out2 - corr;
    out2 = constrain(out2, -255.0, 255.0);
  }

  if (target2 == 0) stopMotor2(); else setMotor2(out2);
  lastErr2 = err2;
}

// =====================================================
//  TELEMETRÍA — Serial USB + TCP WiFi
// =====================================================
void sendTelemetry() {
  // Mismo formato que el ESP32 original
  // Compatible con Serial Plotter y dashboard Python
  Serial.print("Target1:"); Serial.print(target1,1); Serial.print("\t");
  Serial.print("RPM1:");    Serial.print(rpm1,1);    Serial.print("\t");
  Serial.print("Err1:");    Serial.print(err1,1);    Serial.print("\t");
  Serial.print("Out1:");    Serial.print(out1,1);    Serial.print("\t");
  Serial.print("Target2:"); Serial.print(target2,1); Serial.print("\t");
  Serial.print("RPM2:");    Serial.print(rpm2,1);    Serial.print("\t");
  Serial.print("Err2:");    Serial.print(err2,1);    Serial.print("\t");
  Serial.print("Out2:");    Serial.print(out2,1);    Serial.print("\t");
  Serial.print("Sync:");    Serial.println(syncMode ? 1 : 0);

  // TCP (dashboard Python en modo WiFi)
  if (clientOK) {
    char buf[160];
    snprintf(buf, sizeof(buf),
      "RPM1:%.1f\tTarget1:%.1f\tErr1:%.1f\tOut1:%.1f\t"
      "RPM2:%.1f\tTarget2:%.1f\tErr2:%.1f\tOut2:%.1f",
      rpm1, target1, err1, out1,
      rpm2, target2, err2, out2);
    tcpSend(String(buf));
  }
}

// =====================================================
//  CONTROL FÍSICO MOTORES — TB6612FNG
// =====================================================
void setMotor1(float spd) {
  if (spd > 0) {
    motorDir1=1; digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW);
    analogWrite(PWMA,(int)abs(spd));
  } else if (spd < 0) {
    motorDir1=-1; digitalWrite(AIN1,LOW); digitalWrite(AIN2,HIGH);
    analogWrite(PWMA,(int)abs(spd));
  } else { stopMotor1(); }
}
void stopMotor1() {
  digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW); analogWrite(PWMA,0);
  integ1=0; motorDir1=1;
}
void setMotor2(float spd) {
  if (spd > 0) {
    motorDir2=1; digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW);
    analogWrite(PWMB,(int)abs(spd));
  } else if (spd < 0) {
    motorDir2=-1; digitalWrite(BIN1,LOW); digitalWrite(BIN2,HIGH);
    analogWrite(PWMB,(int)abs(spd));
  } else { stopMotor2(); }
}
void stopMotor2() {
  digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW); analogWrite(PWMB,0);
  integ2=0; motorDir2=1;
}

// =====================================================
//  ISR ENCODERS
// =====================================================
void isr1() { if (digitalRead(ENC1B)) encoderPos1++; else encoderPos1--; }
void isr2() { if (digitalRead(ENC2B)) encoderPos2++; else encoderPos2--; }

// =====================================================
//  PROCESAMIENTO DE COMANDOS
//  Acepta comandos desde Serial USB y desde TCP/WiFi
// =====================================================
void processCommand(String cmd) {
  cmd.trim(); cmd.toLowerCase();

  if      (cmd.startsWith("m1="))  { target1=constrain(cmd.substring(3).toFloat(),-150,150); integ1=0; lastErr1=0; Serial.print(">> M1: "); Serial.println(target1); }
  else if (cmd.startsWith("m2="))  { target2=constrain(cmd.substring(3).toFloat(),-150,150); integ2=0; lastErr2=0; Serial.print(">> M2: "); Serial.println(target2); }
  else if (cmd.startsWith("m12=")) { float v=constrain(cmd.substring(4).toFloat(),-150,150); target1=v; target2=v; integ1=0; integ2=0; Serial.print(">> M1+M2: "); Serial.println(v); }

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

  else if (cmd.startsWith("kpsync=")) { Kp_sync=cmd.substring(7).toFloat(); Serial.print(">> Kp_sync="); Serial.println(Kp_sync,4); }
  else if (cmd == "sync=1") { syncMode=true;  Serial.println(">> Sync ON"); }
  else if (cmd == "sync=0") { syncMode=false; Serial.println(">> Sync OFF"); }

  else if (cmd == "stop")  { target1=0; target2=0; stopMotor1(); stopMotor2(); Serial.println(">> STOP"); }
  else if (cmd == "stop1") { target1=0; stopMotor1(); Serial.println(">> STOP M1"); }
  else if (cmd == "stop2") { target2=0; stopMotor2(); Serial.println(">> STOP M2"); }

  else if (cmd == "reset") {
    noInterrupts(); encoderPos1=0; encoderPos2=0; lastPos1=0; lastPos2=0; interrupts();
    Serial.println(">> Encoders reseteados");
  }
  else if (cmd == "fixsign=1") { fixSign=true;  Serial.println(">> fixSign ON"); }
  else if (cmd == "fixsign=0") { fixSign=false; Serial.println(">> fixSign OFF"); }

  else if (cmd == "diag")   { printDiag(); }
  else if (cmd == "status") { printStatus(); }
  else if (cmd == "pid")    { printPID(); }
  else if (cmd == "ping")   { Serial.println(">> Arduino Mega WiFi OK"); }
  else if (cmd == "help")   { printHelp(); }
  else if (cmd == "wifi")   { printWiFiStatus(); }
  else { Serial.print(">> Desconocido: "); Serial.println(cmd); }
}

// =====================================================
//  DIAGNÓSTICO Y AYUDA
// =====================================================
void printHelp() {
  Serial.println("\n+----------------------------------------------------+");
  Serial.println("|  ARDUINO MEGA WIFI — PID DUAL  TB6612FNG           |");
  Serial.println("|  Compilado para ATmega2560  —  WiFi via AT         |");
  Serial.println("+----------------------------------------------------+");
  Serial.println("  m1=80 / m2=80 / m12=80  (neg = retroceso)");
  Serial.println("  kp1f= ki1f= kd1f=  (PID avance  M1)");
  Serial.println("  kp2f= ki2f= kd2f=  (PID avance  M2)");
  Serial.println("  kp1r= ki1r= kd1r=  (PID retroceso M1)");
  Serial.println("  kp2r= ki2r= kd2r=  (PID retroceso M2)");
  Serial.println("  sync=1/0  kpsync=  stop  reset");
  Serial.println("  fixsign=1/0  diag  status  pid  wifi  ping  help");
  Serial.println("----------------------------------------------------\n");
}

void printDiag() {
  Serial.println("\n-- DIAGNÓSTICO ENCODER ---");
  Serial.print("  M1 Dir:"); Serial.print(motorDir1==1?"AVANCE":"RETROCESO");
  Serial.print("  RPM:"); Serial.println(rpm1,2);
  Serial.print("  M2 Dir:"); Serial.print(motorDir2==1?"AVANCE":"RETROCESO");
  Serial.print("  RPM:"); Serial.println(rpm2,2);
  Serial.print("  fixSign: "); Serial.println(fixSign?"ON":"OFF");
  Serial.println("  Si retrocedes y RPM>0 → fixsign=1\n");
}

void printStatus() {
  Serial.println("\n-- STATUS ---");
  Serial.print("  M1 RPM:"); Serial.print(rpm1,2); Serial.print(" Tgt:"); Serial.print(target1,1); Serial.print(" Err:"); Serial.print(err1,2); Serial.print(" Out:"); Serial.println(out1,2);
  Serial.print("  M2 RPM:"); Serial.print(rpm2,2); Serial.print(" Tgt:"); Serial.print(target2,1); Serial.print(" Err:"); Serial.print(err2,2); Serial.print(" Out:"); Serial.println(out2,2);
  Serial.print("  Sync:"); Serial.print(syncMode?"ON":"OFF"); Serial.print("  fixSign:"); Serial.println(fixSign?"ON":"OFF");
}

void printPID() {
  Serial.println("\n-- PID AVANCE ---");
  Serial.print("  M1 Kp:"); Serial.print(Kp1f,4); Serial.print(" Ki:"); Serial.print(Ki1f,4); Serial.print(" Kd:"); Serial.println(Kd1f,4);
  Serial.print("  M2 Kp:"); Serial.print(Kp2f,4); Serial.print(" Ki:"); Serial.print(Ki2f,4); Serial.print(" Kd:"); Serial.println(Kd2f,4);
  Serial.println("-- PID RETROCESO ---");
  Serial.print("  M1 Kp:"); Serial.print(Kp1r,4); Serial.print(" Ki:"); Serial.print(Ki1r,4); Serial.print(" Kd:"); Serial.println(Kd1r,4);
  Serial.print("  M2 Kp:"); Serial.print(Kp2r,4); Serial.print(" Ki:"); Serial.print(Ki2r,4); Serial.print(" Kd:"); Serial.println(Kd2r,4);
}

void printWiFiStatus() {
  Serial.print("  WiFi: ");    Serial.println(wifiOK   ? "CONECTADO" : "SIN CONEXIÓN");
  Serial.print("  Servidor: ");Serial.println(serverOK ? "OK" : "FALLO");
  Serial.print("  Cliente: "); Serial.println(clientOK ? "CONECTADO" : "ESPERANDO");
  // Pedir IP al ESP8266
  Serial3.println("AT+CIFSR");
  delay(800);
  while (Serial3.available()) {
    String l = Serial3.readStringUntil('\n'); l.trim();
    if (l.indexOf("STAIP") >= 0) { Serial.print("  IP: "); Serial.println(l); }
  }
}
