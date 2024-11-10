// Pines de los Sensores Ultrasonicos
#define TRIG_PIN_1 25
#define ECHO_PIN_1 34

#define TRIG_PIN_2 27
#define ECHO_PIN_2 35

#define TRIG_PIN_3 2
#define ECHO_PIN_3 15

// Función para medir la distancia con un sensor específico
float medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH);
  float distancia = duracion * 0.034 / 2;  // Conversión a cm
  return distancia;
}

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);

  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);
}

void loop() {
  float distancia1 = medirDistancia(TRIG_PIN_1, ECHO_PIN_1);
  float distancia2 = medirDistancia(TRIG_PIN_2, ECHO_PIN_2);
  float distancia3 = medirDistancia(TRIG_PIN_3, ECHO_PIN_3);

  Serial.print("Distancia Sensor 1: ");
  Serial.print(distancia1);
  Serial.println(" cm");

  Serial.print("Distancia Sensor 2: ");
  Serial.print(distancia2);
  Serial.println(" cm");

  Serial.print("Distancia Sensor 3: ");
  Serial.print(distancia3);
  Serial.println(" cm");

  Serial.println("--------------------------");
  delay(500);  // Pequeño retraso antes de la siguiente lectura
}
