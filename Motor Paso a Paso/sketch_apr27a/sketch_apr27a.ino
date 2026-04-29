// Definición de pines
const int dirPin = 4;    // Pin para la dirección
const int stepPin = 5;   // Pin para los pulsos (pasos)

// Pasos por revolución (el NEMA 17 suele tener 1.8 grados por paso)
const int stepsPerRev = 200; 

void setup() {
  // Configurar los pines como salida
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  // Definir sentido horario (HIGH o LOW según la conexión de tus cables)
  digitalWrite(dirPin, HIGH); 

  // Generar los pasos
  for(int x = 0; x < stepsPerRev; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000); // Controla la velocidad (menor número = más rápido)
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }

  delay(1000); // Esperar un segundo antes de repetir
}
