// =====================================================
//   ESP32 CONTROL DOBLE MOTOR + PID AVANCE/RETROCESO
//   v4.0 WiFi — Control remoto via UDP
//
//   Cambios respecto a v3.0:
//   - Agrega WiFi + UDP para control inalambrico
//   - Comandos llegan por UDP (puerto 4210)
//   - Telemetria sale por UDP (puerto 4211)
//   - Watchdog: sin comandos en 2s → STOP de seguridad
//   - Serial USB sigue funcionando para depurar con cable
//
//   ANTES DE SUBIR: cambia WIFI_SSID y WIFI_PASS
// =====================================================

#include <WiFi.h>
#include <WiFiUdp.h>

// =====================================================
// CONFIGURACION WIFI — pon los datos de tu hotspot
// =====================================================
const char* WIFI_SSID = "12345";
const char* WIFI_PASS = "12345678";

// Puerto donde el ESP32 escucha comandos de Python
const int UDP_PORT_RX = 4210;

// Puerto donde el ESP32 envia telemetria a Python
const int UDP_PORT_TX = 4211;

// =====================================================
// OBJETOS WIFI Y UDP
// =====================================================
WiFiUDP udp;
IPAddress pythonIP;
bool pythonKnown = false;

// =====================================================
// WATCHDOG DE SEGURIDAD
// Si no llega ningun comando en WATCHDOG_MS ms
// se detienen los motores automaticamente
// =====================================================
const unsigned long WATCHDOG_MS = 2000;
unsigned long lastCmdTime = 0;

// Prototipos ISR (necesarios antes del setup)
void IRAM_ATTR encoder1ISR();
void IRAM_ATTR encoder2ISR();

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
// VARIABLES DE ENCODERS
// =====================================================
volatile long encoderPos1 = 0;
volatile long encoderPos2 = 0;
long lastEncoderPos1 = 0;
long lastEncoderPos2 = 0;
const int PPR = 234;   // Pulsos por revolucion

// =====================================================
// DIRECCION REAL DEL MOTOR
//   1  = el motor gira hacia adelante (avance)
//  -1  = el motor gira hacia atras   (retroceso)
// =====================================================
int motorDir1 = 1;
int motorDir2 = 1;

// =====================================================
// VARIABLES DE TIEMPO
// =====================================================
unsigned long lastTime = 0;

// =====================================================
// VELOCIDADES
// =====================================================
float rpm1 = 0, rpmFiltrada1 = 0, targetRPM1 = 0;
float rpm2 = 0, rpmFiltrada2 = 0, targetRPM2 = 0;

// =====================================================
// GANANCIAS PID MOTOR 1 - AVANCE  (target > 0)
// =====================================================
float Kp1_fwd = 1.35, Ki1_fwd = 0.500, Kd1_fwd = 0.038;

// =====================================================
// GANANCIAS PID MOTOR 1 - RETROCESO  (target < 0)
// =====================================================
float Kp1_rev = 1.35, Ki1_rev = 0.545, Kd1_rev = 0.038;

// =====================================================
// GANANCIAS PID MOTOR 2 - AVANCE
// =====================================================
float Kp2_fwd = 1.35, Ki2_fwd = 0.545, Kd2_fwd = 0.028;

// =====================================================
// GANANCIAS PID MOTOR 2 - RETROCESO
// =====================================================
float Kp2_rev = 1.35, Ki2_rev = 0.545, Kd2_rev = 0.028;

// =====================================================
// ESTADO INTERNO DEL PID
// =====================================================
float error1 = 0, lastError1 = 0, integral1 = 0, output1 = 0;
float error2 = 0, lastError2 = 0, integral2 = 0, output2 = 0;

// =====================================================
// SINCRONIZACION
// =====================================================
bool syncMode = false;
float Kp_sync = 0.5;

// =====================================================
// MODO DE SALIDA SERIAL
// =====================================================
bool plotterMode = false;

// =====================================================
// CORRECCION DE SIGNO POR HARDWARE
// =====================================================
bool fixSign = true;


// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Configurar pines de direccion del puente H
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configurar pines de encoders como entrada
  pinMode(ENCODER1_A, INPUT);
  pinMode(ENCODER1_B, INPUT);
  pinMode(ENCODER2_A, INPUT);
  pinMode(ENCODER2_B, INPUT);

  // Interrupciones en flanco de subida del canal A
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2ISR, RISING);

  // Canal PWM para cada motor
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  stopMotor1();
  stopMotor2();

  // Conectar al hotspot
  Serial.println("\nConectando a: " + String(WIFI_SSID));
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    intentos++;
    if (intentos > 40) {
      Serial.println("\nNo se pudo conectar. Verifica SSID y contrasena.");
      Serial.println("Reiniciando en 3 segundos...");
      delay(3000);
      ESP.restart();
    }
  }

  Serial.println("\n\nWiFi conectado!");
  Serial.print("IP del ESP32: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
  Serial.println(">>> Copia esta linea en xbox_wifi.py:");
  Serial.print(">>> ESP32_IP = \"");
  Serial.print(WiFi.localIP());
  Serial.println("\"");
  Serial.println("");

  udp.begin(UDP_PORT_RX);

  lastTime    = millis();
  lastCmdTime = millis();

  printHelp();
}


// =====================================================
// LOOP PRINCIPAL
// =====================================================
void loop() {

  // Leer paquetes UDP entrantes
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char buf[128];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';

    // Guardar IP de Python la primera vez que manda algo
    if (!pythonKnown) {
      pythonIP    = udp.remoteIP();
      pythonKnown = true;
      Serial.print(">> Python conectado desde: ");
      Serial.println(pythonIP);
    }

    lastCmdTime = millis();   // resetear watchdog
    processCommand(String(buf));
  }

  // Watchdog — sin comandos por WATCHDOG_MS → STOP de seguridad
  if (millis() - lastCmdTime > WATCHDOG_MS) {
    if (targetRPM1 != 0 || targetRPM2 != 0) {
      Serial.println(">> Watchdog: sin comandos, STOP de seguridad");
      targetRPM1 = 0; targetRPM2 = 0;
      stopMotor1(); stopMotor2();
    }
  }

  // Control PID cada 20 ms (50 Hz)
  if (millis() - lastTime >= 20) {
    calcularRPM();
    pidControl();
    printData();
    if (pythonKnown) {
      sendTelemetry();
    }
  }

  // Leer comandos desde Serial USB (para depurar con cable)
  if (Serial.available()) {
    processCommand(Serial.readStringUntil('\n'));
  }
}


// =====================================================
// ENVIAR TELEMETRIA A PYTHON POR UDP
// Mismo formato que printData() para que Python
// lo parsee de la misma manera
// =====================================================
void sendTelemetry() {
  char buf[220];
  snprintf(buf, sizeof(buf),
    "Target1:%.1f\tRPM1:%.1f\tErr1:%.1f\tOut1:%.1f\t"
    "Target2:%.1f\tRPM2:%.1f\tErr2:%.1f\tOut2:%.1f\tSync:%d",
    targetRPM1, rpm1, error1, output1,
    targetRPM2, rpm2, error2, output2,
    syncMode ? 1 : 0
  );
  udp.beginPacket(pythonIP, UDP_PORT_TX);
  udp.print(buf);
  udp.endPacket();
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

  float rawRpm1 = (delta1 * 60.0) / (PPR * deltaTime);
  float rawRpm2 = (delta2 * 60.0) / (PPR * deltaTime);

  if (fixSign) {
    rpm1 = rawRpm1 * motorDir1;
    rpm2 = rawRpm2 * motorDir2;
  } else {
    rpm1 = rawRpm1;
    rpm2 = rawRpm2;
  }

  rpmFiltrada1 = 0.7 * rpmFiltrada1 + 0.3 * rpm1;
  rpmFiltrada2 = 0.7 * rpmFiltrada2 + 0.3 * rpm2;

  rpm1 = rpmFiltrada1;
  rpm2 = rpmFiltrada2;

  lastTime = currentTime;
}


// =====================================================
// SELECCION DE GANANCIAS SEGUN DIRECCION
// =====================================================
void getGains1(float &Kp, float &Ki, float &Kd) {
  if (targetRPM1 >= 0) {
    Kp = Kp1_fwd; Ki = Ki1_fwd; Kd = Kd1_fwd;
  } else {
    Kp = Kp1_rev; Ki = Ki1_rev; Kd = Kd1_rev;
  }
}

void getGains2(float &Kp, float &Ki, float &Kd) {
  if (targetRPM2 >= 0) {
    Kp = Kp2_fwd; Ki = Ki2_fwd; Kd = Kd2_fwd;
  } else {
    Kp = Kp2_rev; Ki = Ki2_rev; Kd = Kd2_rev;
  }
}


// =====================================================
// PID CONTROL
// =====================================================
void pidControl() {

  float dt = 0.02;

  // Motor 1
  float Kp1, Ki1, Kd1;
  getGains1(Kp1, Ki1, Kd1);

  error1     = targetRPM1 - rpm1;
  integral1 += error1 * dt;
  integral1  = constrain(integral1, -400, 400);
  float derivative1 = (error1 - lastError1) / dt;

  output1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;
  output1 = constrain(output1, -255, 255);

  if (targetRPM1 == 0) stopMotor1(); else setMotor1(output1);
  lastError1 = error1;

  // Motor 2
  float Kp2, Ki2, Kd2;
  getGains2(Kp2, Ki2, Kd2);

  error2     = targetRPM2 - rpm2;
  integral2 += error2 * dt;
  integral2  = constrain(integral2, -400, 400);
  float derivative2 = (error2 - lastError2) / dt;

  output2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;
  output2 = constrain(output2, -255, 255);

  // Sincronizacion lineal
  if (syncMode && targetRPM2 != 0) {
    float syncError      = abs(rpm1) - abs(rpm2);
    float syncCorrection = Kp_sync * syncError;
    if (output2 >= 0) output2 += syncCorrection;
    else              output2 -= syncCorrection;
    output2 = constrain(output2, -255, 255);
  }

  if (targetRPM2 == 0) stopMotor2(); else setMotor2(output2);
  lastError2 = error2;
}


// =====================================================
// IMPRIMIR DATOS POR SERIAL USB
// =====================================================
void printData() {
  Serial.print("Target1:"); Serial.print(targetRPM1, 1); Serial.print("\t");
  Serial.print("RPM1:");    Serial.print(rpm1, 1);       Serial.print("\t");
  Serial.print("Err1:");    Serial.print(error1, 1);     Serial.print("\t");
  Serial.print("Out1:");    Serial.print(output1, 1);    Serial.print("\t");
  Serial.print("Target2:"); Serial.print(targetRPM2, 1); Serial.print("\t");
  Serial.print("RPM2:");    Serial.print(rpm2, 1);       Serial.print("\t");
  Serial.print("Err2:");    Serial.print(error2, 1);     Serial.print("\t");
  Serial.print("Out2:");    Serial.print(output2, 1);    Serial.print("\t");
  Serial.print("Sync:");    Serial.println(syncMode ? 1 : 0);
}


// =====================================================
// CONTROL FISICO DE MOTORES
// =====================================================
void setMotor1(float speed) {
  if (speed > 0) {
    motorDir1 = 1;
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    ledcWrite(ENA, (int)abs(speed));
  } else if (speed < 0) {
    motorDir1 = -1;
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    ledcWrite(ENA, (int)abs(speed));
  } else { stopMotor1(); }
}

void stopMotor1() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  ledcWrite(ENA, 0);
  integral1 = 0;
  motorDir1 = 1;
}

void setMotor2(float speed) {
  if (speed > 0) {
    motorDir2 = 1;
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    ledcWrite(ENB, (int)abs(speed));
  } else if (speed < 0) {
    motorDir2 = -1;
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    ledcWrite(ENB, (int)abs(speed));
  } else { stopMotor2(); }
}

void stopMotor2() {
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENB, 0);
  integral2 = 0;
  motorDir2 = 1;
}


// =====================================================
// ISR ENCODERS
// =====================================================
void IRAM_ATTR encoder1ISR() {
  if (digitalRead(ENCODER1_B)) encoderPos1++;
  else                         encoderPos1--;
}

void IRAM_ATTR encoder2ISR() {
  if (digitalRead(ENCODER2_B)) encoderPos2++;
  else                         encoderPos2--;
}


// =====================================================
// DIAGNOSTICO DE SIGNO DE ENCODER
// =====================================================
void printDiag() {
  Serial.println("\n-- DIAGNOSTICO ENCODER --------------------------------");
  Serial.print("  Motor 1 -> Dir HW: ");
  Serial.print(motorDir1 == 1 ? "AVANCE" : "RETROCESO");
  Serial.print("  |  RPM medida: ");
  Serial.println(rpm1, 2);
  Serial.print("  Motor 2 -> Dir HW: ");
  Serial.print(motorDir2 == 1 ? "AVANCE" : "RETROCESO");
  Serial.print("  |  RPM medida: ");
  Serial.println(rpm2, 2);
  Serial.println();
  Serial.println("  Si retrocedes y RPM es POSITIVA -> usa fixsign=1");
  Serial.println("  Si retrocedes y RPM es NEGATIVA -> fixsign=0 esta bien");
  Serial.print("  fixSign actual: ");
  Serial.println(fixSign ? "1 (correccion por HW activa)" : "0 (cuadratura)");
  Serial.println("------------------------------------------------------\n");
}


// =====================================================
// MENU DE AYUDA
// =====================================================
void printHelp() {
  Serial.println("\n+--------------------------------------------------+");
  Serial.println("|   ESP32 PID DUAL WiFi  v4.0                      |");
  Serial.println("+--------------------------------------------------+");
  Serial.println("\n-- SETPOINTS ------------------------------------------");
  Serial.println("  m1=80      -> Motor 1 a +80 RPM (avance)");
  Serial.println("  m1=-80     -> Motor 1 a -80 RPM (retroceso)");
  Serial.println("  m2=80      -> Motor 2 a +80 RPM");
  Serial.println("  m12=80     -> Ambos motores a +80 RPM");
  Serial.println("\n-- PID AVANCE -----------------------------------------");
  Serial.println("  kp1f=  ki1f=  kd1f=  (Motor 1 avance)");
  Serial.println("  kp2f=  ki2f=  kd2f=  (Motor 2 avance)");
  Serial.println("\n-- PID RETROCESO --------------------------------------");
  Serial.println("  kp1r=  ki1r=  kd1r=  (Motor 1 retroceso)");
  Serial.println("  kp2r=  ki2r=  kd2r=  (Motor 2 retroceso)");
  Serial.println("\n-- SINCRONIZACION -------------------------------------");
  Serial.println("  sync=1/0   kpsync=0.5");
  Serial.println("\n-- CONTROL --------------------------------------------");
  Serial.println("  stop / stop1 / stop2 / reset");
  Serial.println("\n-- DIAGNOSTICO ----------------------------------------");
  Serial.println("  status  pid  diag  fixsign=1/0  ping  help");
  Serial.println("------------------------------------------------------\n");
}


// =====================================================
// PROCESAR COMANDO (desde UDP o Serial USB)
// Mismos comandos que la version v3.0 por Serial
// =====================================================
void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("m1=")) {
    targetRPM1 = constrain(cmd.substring(3).toFloat(), -150, 150);
    Serial.print(">> M1 setpoint: "); Serial.println(targetRPM1);
  }
  else if (cmd.startsWith("m2=")) {
    targetRPM2 = constrain(cmd.substring(3).toFloat(), -150, 150);
    Serial.print(">> M2 setpoint: "); Serial.println(targetRPM2);
  }
  else if (cmd.startsWith("m12=")) {
    float v = constrain(cmd.substring(4).toFloat(), -150, 150);
    targetRPM1 = targetRPM2 = v;
    Serial.print(">> M1+M2 setpoint: "); Serial.println(v);
  }
  else if (cmd.startsWith("kp1f=")) { Kp1_fwd=cmd.substring(5).toFloat(); Serial.print(">> Kp1_fwd: "); Serial.println(Kp1_fwd,4); }
  else if (cmd.startsWith("ki1f=")) { Ki1_fwd=cmd.substring(5).toFloat(); Serial.print(">> Ki1_fwd: "); Serial.println(Ki1_fwd,4); }
  else if (cmd.startsWith("kd1f=")) { Kd1_fwd=cmd.substring(5).toFloat(); Serial.print(">> Kd1_fwd: "); Serial.println(Kd1_fwd,4); }
  else if (cmd.startsWith("kp1r=")) { Kp1_rev=cmd.substring(5).toFloat(); Serial.print(">> Kp1_rev: "); Serial.println(Kp1_rev,4); }
  else if (cmd.startsWith("ki1r=")) { Ki1_rev=cmd.substring(5).toFloat(); Serial.print(">> Ki1_rev: "); Serial.println(Ki1_rev,4); }
  else if (cmd.startsWith("kd1r=")) { Kd1_rev=cmd.substring(5).toFloat(); Serial.print(">> Kd1_rev: "); Serial.println(Kd1_rev,4); }
  else if (cmd.startsWith("kp2f=")) { Kp2_fwd=cmd.substring(5).toFloat(); Serial.print(">> Kp2_fwd: "); Serial.println(Kp2_fwd,4); }
  else if (cmd.startsWith("ki2f=")) { Ki2_fwd=cmd.substring(5).toFloat(); Serial.print(">> Ki2_fwd: "); Serial.println(Ki2_fwd,4); }
  else if (cmd.startsWith("kd2f=")) { Kd2_fwd=cmd.substring(5).toFloat(); Serial.print(">> Kd2_fwd: "); Serial.println(Kd2_fwd,4); }
  else if (cmd.startsWith("kp2r=")) { Kp2_rev=cmd.substring(5).toFloat(); Serial.print(">> Kp2_rev: "); Serial.println(Kp2_rev,4); }
  else if (cmd.startsWith("ki2r=")) { Ki2_rev=cmd.substring(5).toFloat(); Serial.print(">> Ki2_rev: "); Serial.println(Ki2_rev,4); }
  else if (cmd.startsWith("kd2r=")) { Kd2_rev=cmd.substring(5).toFloat(); Serial.print(">> Kd2_rev: "); Serial.println(Kd2_rev,4); }
  else if (cmd.startsWith("kpsync=")) { Kp_sync=cmd.substring(7).toFloat(); Serial.print(">> Kp_sync: "); Serial.println(Kp_sync,4); }
  else if (cmd == "sync=1") { syncMode=true;  Serial.println(">> Sync ON");  }
  else if (cmd == "sync=0") { syncMode=false; Serial.println(">> Sync OFF"); }
  else if (cmd == "stop") {
    targetRPM1=0; targetRPM2=0; stopMotor1(); stopMotor2();
    Serial.println(">> STOP todos");
  }
  else if (cmd == "stop1") { targetRPM1=0; stopMotor1(); Serial.println(">> STOP M1"); }
  else if (cmd == "stop2") { targetRPM2=0; stopMotor2(); Serial.println(">> STOP M2"); }
  else if (cmd == "reset") {
    noInterrupts();
    encoderPos1=encoderPos2=lastEncoderPos1=lastEncoderPos2=0;
    interrupts();
    Serial.println(">> Encoders reseteados");
  }
  else if (cmd == "fixsign=1") { fixSign=true;  Serial.println(">> fixSign ON");  }
  else if (cmd == "fixsign=0") { fixSign=false; Serial.println(">> fixSign OFF"); }
  else if (cmd == "plotter=1") { plotterMode=true;  Serial.println(">> Modo PLOTTER"); }
  else if (cmd == "plotter=0") { plotterMode=false; Serial.println(">> Modo MONITOR"); }
  else if (cmd == "diag")   { printDiag(); }
  else if (cmd == "help")   { printHelp(); }
  else if (cmd == "ping")   {
    Serial.println(">> PONG");
    if (pythonKnown) {
      udp.beginPacket(pythonIP, UDP_PORT_TX);
      udp.print("PONG");
      udp.endPacket();
    }
  }
  else if (cmd == "status") {
    Serial.println("\n-- STATUS ---------------------------------------------");
    Serial.print("  WiFi IP: "); Serial.println(WiFi.localIP());
    Serial.print("  M1 -> RPM: "); Serial.print(rpm1,2);
    Serial.print("  Target: ");    Serial.print(targetRPM1,1);
    Serial.print("  Error: ");     Serial.print(error1,2);
    Serial.print("  Output: ");    Serial.println(output1,2);
    Serial.print("  M2 -> RPM: "); Serial.print(rpm2,2);
    Serial.print("  Target: ");    Serial.print(targetRPM2,1);
    Serial.print("  Error: ");     Serial.print(error2,2);
    Serial.print("  Output: ");    Serial.println(output2,2);
    Serial.print("  Sync: ");      Serial.print(syncMode?"ON":"OFF");
    Serial.print("  Kp_sync: ");   Serial.print(Kp_sync,3);
    Serial.print("  fixSign: ");   Serial.println(fixSign?"ON":"OFF");
    Serial.println("------------------------------------------------------\n");
  }
  else if (cmd == "pid") {
    Serial.println("\n-- PID AVANCE -----------------------------------------");
    Serial.print("  M1 -> Kp:"); Serial.print(Kp1_fwd,4); Serial.print(" Ki:"); Serial.print(Ki1_fwd,4); Serial.print(" Kd:"); Serial.println(Kd1_fwd,4);
    Serial.print("  M2 -> Kp:"); Serial.print(Kp2_fwd,4); Serial.print(" Ki:"); Serial.print(Ki2_fwd,4); Serial.print(" Kd:"); Serial.println(Kd2_fwd,4);
    Serial.println("-- PID RETROCESO --------------------------------------");
    Serial.print("  M1 -> Kp:"); Serial.print(Kp1_rev,4); Serial.print(" Ki:"); Serial.print(Ki1_rev,4); Serial.print(" Kd:"); Serial.println(Kd1_rev,4);
    Serial.print("  M2 -> Kp:"); Serial.print(Kp2_rev,4); Serial.print(" Ki:"); Serial.print(Ki2_rev,4); Serial.print(" Kd:"); Serial.println(Kd2_rev,4);
    Serial.println("------------------------------------------------------\n");
  }
  else {
    Serial.print(">> Comando desconocido: "); Serial.println(cmd);
    Serial.println("   Escribe 'help' para ver los comandos disponibles");
  }
}
