/*
  Control de 2 motores DC con L298N + Arduino UNO
  - Motor1: mantener 'A' = horario, mantener 'B' = antihorario
  - Motor2: mantener 'C' = horario, mantener 'D' = antihorario
  - Si se suelta la tecla (no se recibe de nuevo), el motor se detiene
  - Velocidad: escribir un numero por Serial Monitor (RPM objetivo), max 6 RPM.
    Se convierte linealmente a PWM (0-255).

  IMPORTANTE SOBRE "tecla mantenida":
  El Serial Monitor normal NO envia "tecla presionada" continuamente.
  Para simular mantener una tecla, tu app (o un script) debe enviar la letra
  repetidamente. Este codigo usa timeout: si deja de llegar la letra, para.
  Parada instantanea de ambos motores: enviar "S" o "STOP" (una linea).
*/

// -------------------- Pines L298N --------------------
// Motor 1
const uint8_t ENA = 5;   // PWM
const uint8_t IN1 = 8;
const uint8_t IN2 = 9;

// Motor 2
const uint8_t ENB = 6;   // PWM
const uint8_t IN3 = 10;
const uint8_t IN4 = 11;

// -------------------- Configuracion --------------------
const float RPM_MAX = 6.0f;                // limite tecnico pedido
const unsigned long CMD_TIMEOUT_MS = 250;  // tiempo max sin renovar comando

// Estado de direccion por motor
enum Direction : uint8_t {
  STOPPED = 0,
  CW,
  CCW
};

Direction dirMotor1 = STOPPED;
Direction dirMotor2 = STOPPED;

unsigned long lastCmdM1 = 0;
unsigned long lastCmdM2 = 0;

float rpmObjetivo = 3.0f;  // valor inicial
uint8_t pwmSalida = 127;   // equivalente aproximado a 3/6

String serialBuffer = "";

// -------------------- Utilidades --------------------
uint8_t rpmToPwm(float rpm) {
  if (rpm <= 0.0f) return 0;
  if (rpm >= RPM_MAX) return 255;
  return (uint8_t)((rpm / RPM_MAX) * 255.0f);
}

void detenerMotores() {
  // Motor 1
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  // Motor 2
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);

  dirMotor1 = STOPPED;
  dirMotor2 = STOPPED;
  lastCmdM1 = 0;
  lastCmdM2 = 0;
}

void aplicarMotor1(Direction d) {
  switch (d) {
    case CW:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, pwmSalida);
      break;
    case CCW:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, pwmSalida);
      break;
    default:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      break;
  }
}

void aplicarMotor2(Direction d) {
  switch (d) {
    case CW:
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, pwmSalida);
      break;
    case CCW:
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, pwmSalida);
      break;
    default:
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 0);
      break;
  }
}

void procesarLineaSerial(String linea) {
  linea.trim();
  if (linea.length() == 0) return;

  {
    String up = linea;
    up.toUpperCase();
    if (up == "STOP") {
      detenerMotores();
      Serial.println(F("Motores DETENIDO (STOP)"));
      return;
    }
  }

  // Letras de control de giro
  if (linea.length() == 1) {
    char c = linea.charAt(0);
    c = toupper(c);

    switch (c) {
      case 'A':
        dirMotor1 = CW;
        lastCmdM1 = millis();
        Serial.println(F("M1 -> Horario"));
        return;
      case 'B':
        dirMotor1 = CCW;
        lastCmdM1 = millis();
        Serial.println(F("M1 -> Antihorario"));
        return;
      case 'C':
        dirMotor2 = CW;
        lastCmdM2 = millis();
        Serial.println(F("M2 -> Horario"));
        return;
      case 'D':
        dirMotor2 = CCW;
        lastCmdM2 = millis();
        Serial.println(F("M2 -> Antihorario"));
        return;
      case 'S':
        detenerMotores();
        Serial.println(F("Motores DETENIDO (S)"));
        return;
      default:
        break;
    }
  }

  // Si no es letra, intentar leer RPM
  float nuevaRpm = linea.toFloat();
  if (nuevaRpm < 0.0f) nuevaRpm = 0.0f;
  if (nuevaRpm > RPM_MAX) nuevaRpm = RPM_MAX;

  rpmObjetivo = nuevaRpm;
  pwmSalida = rpmToPwm(rpmObjetivo);

  Serial.print(F("RPM objetivo = "));
  Serial.print(rpmObjetivo, 2);
  Serial.print(F(" | PWM = "));
  Serial.println(pwmSalida);
}

void leerSerialNoBloqueante() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();

    // Fin de linea en \n o \r
    if (ch == '\n' || ch == '\r') {
      if (serialBuffer.length() > 0) {
        procesarLineaSerial(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += ch;
      // Limite simple para evitar crecimiento excesivo
      if (serialBuffer.length() > 20) {
        serialBuffer = "";
      }
    }
  }
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
  detenerMotores();

  Serial.println(F("=== Control 2 motores L298N ==="));
  Serial.println(F("Comandos: A/B para Motor1, C/D para Motor2"));
  Serial.println(F("Parada ambos motores al instante: S o STOP"));
  Serial.println(F("Enviar numero (0 a 6) para RPM objetivo"));
}

void loop() {
  leerSerialNoBloqueante();

  unsigned long ahora = millis();

  // Logica "si se suelta, se detiene": timeout de comando
  if ((ahora - lastCmdM1) > CMD_TIMEOUT_MS) {
    dirMotor1 = STOPPED;
  }
  if ((ahora - lastCmdM2) > CMD_TIMEOUT_MS) {
    dirMotor2 = STOPPED;
  }

  aplicarMotor1(dirMotor1);
  aplicarMotor2(dirMotor2);

  delay(5);
}
