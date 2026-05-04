#define U1_TRIG 14
#define U1_ECHO 33

#define U2_TRIG 27
#define U2_ECHO 32

#define U3_TRIG 26
#define U3_ECHO 35

long medirDistancia(int trigPin, int echoPin) {
  // Limpia trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Pulso de 10us
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Lee el tiempo del echo (timeout 30ms ≈ 5m)
  long duracion = pulseIn(echoPin, HIGH, 30000);

  // Si no detecta nada
  if (duracion == 0) return -1;

  // Convertir a cm
  long distancia = duracion * 0.034 / 2;

  return distancia;
}

void setup() {
  Serial.begin(115200);

  pinMode(U1_TRIG, OUTPUT);
  pinMode(U2_TRIG, OUTPUT);
  pinMode(U3_TRIG, OUTPUT);

  pinMode(U1_ECHO, INPUT);
  pinMode(U2_ECHO, INPUT);
  pinMode(U3_ECHO, INPUT);

  Serial.println("Ultrasonicos listos...");
}

void loop() {
  long d1 = medirDistancia(U1_TRIG, U1_ECHO);
  delay(20); // evita interferencia

  long d2 = medirDistancia(U2_TRIG, U2_ECHO);
  delay(20);

  long d3 = medirDistancia(U3_TRIG, U3_ECHO);
  delay(20);

  Serial.print("U1: ");
  Serial.print(d1);
  Serial.print(" cm | U2: ");
  Serial.print(d2);
  Serial.print(" cm | U3: ");
  Serial.print(d3);
  Serial.println(" cm");

  delay(10);
}