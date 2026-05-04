#define PWMA 23
#define A1 19
#define A2 18

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);

  // Dirección: adelante
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
}

void loop() {
  // Velocidad (0-255)
  analogWrite(PWMA, 200);

  delay(3000);

  // Apagar motor
  analogWrite(PWMA, 0);

  delay(3000);
}