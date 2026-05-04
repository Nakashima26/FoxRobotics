#include <Wire.h>
#include <MPU6050_tockn.h>

// ===== MPU6050 =====
MPU6050 mpu(Wire);

// ===== Ultrasonicos =====
#define TRIG_L 27
#define ECHO_L 32
#define TRIG_R 26
#define ECHO_R 35

// ===== Motor DC =====
#define PWMA 23
#define A1 18
#define A2 19

// ===== SERVO (SIN LIBRERÍA) =====
#define SERVO_PIN 13

// ===== PWM CONFIG =====
const int freqServo = 50;
const int resServo = 16;

const int freqMotor = 1000;
const int resMotor = 8;

// ===== PID =====
float Kp = 1.5;
float Ki = 0.01;
float Kd = 0.3;

float error, prevError = 0;
float integral = 0;
float derivada;
float outputPID;

// ===== Tiempo =====
unsigned long lastTime = 0;

// ===== Control =====
int velocidadMotor = 180;
bool girando = false;
bool direccionIzquierda = true;
float anguloInicial = 0;

// ===== Centro servo =====
int centroServo = 90;

// ===== FUNCIÓN SERVO =====
void escribirServo(int angulo) {
  if (angulo > 180) angulo = 180;
  if (angulo < 0) angulo = 0;

  // 500–2500 µs
  int pulso = map(angulo, 0, 180, 500, 2500);

  // periodo = 20 ms → 20000 µs
  int duty = (pulso * ((1 << resServo) - 1)) / 20000;

  ledcWrite(SERVO_PIN, duty);
}

// ===== Lectura filtrada =====
long leerDistanciaFiltrada(int trig, int echo) {
  long suma = 0;
  int muestras = 5;

  for (int i = 0; i < muestras; i++) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long duracion = pulseIn(echo, HIGH, 30000);
    long distancia = duracion * 0.034 / 2;

    if (distancia == 0 || distancia > 200) distancia = 200;

    suma += distancia;
    delay(5);
  }

  return suma / muestras;
}

void setup() {
  Serial.begin(115200);

  // MPU
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);

  // Ultrasonicos
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  // Motor
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);

  // ===== PWM MOTOR =====
  ledcAttach(PWMA, freqMotor, resMotor);

  // ===== PWM SERVO =====
  ledcAttach(SERVO_PIN, freqServo, resServo);

  // Inicializar servo
  
  escribirServo(centroServo);
  delay(1000);

  lastTime = millis();

  Serial.println("Sistema listo");
}

void loop() {
  mpu.update();

  long distL = leerDistanciaFiltrada(TRIG_L, ECHO_L);
  long distR = leerDistanciaFiltrada(TRIG_R, ECHO_R);

  // ===== DETECCIÓN DE GIRO =====
  if (!girando && (distL > 80 || distR > 80)) {
    girando = true;
    anguloInicial = mpu.getAngleZ();
    direccionIzquierda = (distL > distR);

    Serial.println("Iniciando giro");
  }

  // ===== MODO GIRO =====
  if (girando) {
    ledcWrite(PWMA, velocidadMotor);

    if (direccionIzquierda) {
      escribirServo(0);
    } else {
      escribirServo(180);
    }

    float anguloActual = mpu.getAngleZ();
    float delta = abs(anguloActual - anguloInicial);

    if (delta > 180) delta = 360 - delta;

    if (delta >= 90) {
      girando = false;
      escribirServo(centroServo);
      Serial.println("Giro completado");
    }


    return;
  }

  // ===== PID =====
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if (dt < 0.01) dt = 0.01;

  lastTime = currentTime;

  error = distL - distR;

  if (error > 50) error = 50;
  if (error < -50) error = -50;

  integral += error * dt;

  if (integral > 50) integral = 50;
  if (integral < -50) integral = -50;

  derivada = (error - prevError) / dt;

  outputPID = Kp * error + Ki * integral + Kd * derivada;

  prevError = error;

  if (outputPID > 40) outputPID = 40;
  if (outputPID < -40) outputPID = -40;

  int anguloServo = centroServo + outputPID;

  escribirServo(anguloServo);

  // Motor
  ledcWrite(PWMA, velocidadMotor);

  // Debug
  Serial.print("Servo: ");
  Serial.println(anguloServo);

  delay(30);
}