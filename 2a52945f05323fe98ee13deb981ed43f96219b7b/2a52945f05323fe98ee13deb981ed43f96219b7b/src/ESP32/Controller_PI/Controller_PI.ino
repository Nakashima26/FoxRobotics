#include <Wire.h>
#include <MPU6050_tockn.h>


// ===== MPU6050 =====
MPU6050 mpu(Wire);

// ===== Ultrasonicos =====
// PINES
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
// PWM
const int freqServo = 50;
const int resServo = 16;

const int freqMotor = 1000;
const int resMotor = 8;

// ===== PID =====
float Kp = 1.5;
float Ki = 0.01;
float Kd = 0.3;
// PID PAREDES
float KpWall = 1.0;
float KiWall = 0.0;
float KdWall = 1.2;

float errorWall = 0;
float prevErrorWall = 0;
float integralWall = 0;

// PID GYRO
float KpGyro = 2.0;
float KiGyro = 0.0;
float KdGyro = 0.5;

float error, prevError = 0;
float integral = 0;
float derivada;
float outputPID;
float errorGyro = 0;
float prevErrorGyro = 0;
float integralGyro = 0;

// ===== Tiempo =====
unsigned long lastTime = 0;
// TIEMPO
unsigned long lastPIDTime = 0;
unsigned long lastGyroTime = 0;

// ===== Control =====
// GIROSCOPIO
float anguloGyro = 0;
float anguloObjetivo = 0;

// CONTROL
int velocidadMotor = 180;
bool girando = false;
int centroServo = 80;

// ESTADOS
enum Estado {
  SIGUIENDO,
  GIRANDO
};

Estado estado = SIGUIENDO;

// GIROS
bool direccionIzquierda = true;
float anguloInicial = 0;
bool primerGiro = false;

// ===== Centro servo =====
int centroServo = 90;
int AngGiro = 82;

// ===== FUNCIÓN SERVO =====
void escribirServo(int angulo) {
  if (angulo > 180) angulo = 180;
  if (angulo < 0) angulo = 0;
unsigned long lastTurnTime = 0;
const int cooldownGiro = 2000;

  // 500–2500 µs
  int pulso = map(angulo, 0, 180, 500, 2500);
// DETECCION
int contadorEsquina = 0;
const int umbralPared = 100;

  // periodo = 20 ms → 20000 µs
  int duty = (pulso * ((1 << resServo) - 1)) / 20000;
// FILTRO
float alpha = 0.75;

float distL_filtrada = 0;
float distR_filtrada = 0;

// FUNCIONES
void escribirServo(int angulo) {
  angulo = constrain(angulo, 0, 180);
  int pulso = map(angulo, 0, 180, 500, 2500);
  int duty = (pulso * ((1 << resServo) - 1)) / 20000;
  ledcWrite(SERVO_PIN, duty);
}

// ===== Lectura filtrada =====
long leerDistanciaFiltrada(int trig, int echo) {
  long suma = 0;
  int muestras = 5;
void setMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  ledcWrite(PWMA, velocidad);
}

  for (int i = 0; i < muestras; i++) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
void printDual(String txt) {
  Serial.print(txt);
}

    long duracion = pulseIn(echo, HIGH, 30000);
    long distancia = duracion * 0.034 / 2;
long leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

    if (distancia == 0 || distancia > 200) distancia = 200;
  long duracion = pulseIn(echo, HIGH, 8000);
  long distancia = duracion * 0.034 / 2;

    suma += distancia;
    delay(5);
  if (distancia == 0 || distancia > 200){
    distancia = 200;
  }

  return suma / muestras;
  return distancia;
}

float filtroEMA(float nueva, float anterior) {
  return alpha * nueva + (1.0 - alpha) * anterior;
}

bool detectarEsquina(long distL, long distR) {
  bool apertura = (distL > umbralPared) || (distR > umbralPared);
  if (apertura){
    contadorEsquina++;
  }else{
    contadorEsquina = 0;
  }
  return contadorEsquina >= 1;
}

void actualizarGyro() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0;
  lastGyroTime = now;
  float gyroZ = mpu.getGyroZ();

  // eliminar ruido
  if (abs(gyroZ) < 1.0){
    gyroZ = 0;
  }
  anguloGyro += gyroZ * dt;
}

// PID ULTRASONICOS + GYRO
void controlPID(long distL, long distR) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  if (dt < 0.01){
    dt = 0.01;
  }
    
  // PID ULTRASONICOS
  errorWall = distL - distR;
  errorWall = constrain(errorWall, -50, 50);
  integralWall += errorWall * dt;
  integralWall = constrain(integralWall, -40, 40);
  float derivWall = (errorWall - prevErrorWall) / dt;
  float outputWall = KpWall * errorWall + KiWall * integralWall + KdWall * derivWall;
  prevErrorWall = errorWall;

  // PID GYRO
  errorGyro = anguloObjetivo - anguloGyro;
  errorGyro = constrain(errorGyro, -20, 20);
  integralGyro += errorGyro * dt;
  integralGyro = constrain(integralGyro, -30, 30);
  float derivGyro = (errorGyro - prevErrorGyro) / dt;
  float outputGyro = KpGyro * errorGyro + KiGyro * integralGyro + KdGyro * derivGyro;
  prevErrorGyro = errorGyro;

  // sumar PID
  float outputFinal = outputWall + outputGyro;
  outputFinal = constrain(outputFinal, -25, 25);
  escribirServo(centroServo + outputFinal);
  setMotor(velocidadMotor);

  printDual(" | Wall:");
  printDual(String(outputWall));
  printDual(" | Gyro:");
  printDual(String(outputGyro));
  printDual(" | Servo:");
  printDual(String(centroServo + outputFinal));
}

void setup() {
  Serial.begin(115200);

  // MPU
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);

  // Ultrasonicos
  // ultrasonicos
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);

  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  // Motor
  // motor
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);

  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);

  // ===== PWM MOTOR =====
  // PWM
  ledcAttach(PWMA, freqMotor, resMotor);

  // ===== PWM SERVO =====
  ledcAttach(SERVO_PIN, freqServo, resServo);

  // Inicializar servo
  
  escribirServo(centroServo);
  delay(1000);

  lastTime = millis();

  // inicio
  distL_filtrada = leerDistancia(TRIG_L, ECHO_L);
  distR_filtrada = leerDistancia(TRIG_R, ECHO_R);
  lastPIDTime = millis();
  lastGyroTime = millis();
  anguloObjetivo = 0;
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
  actualizarGyro();

  // leer sensores
  long distL_raw = leerDistancia(TRIG_L, ECHO_L);
  long distR_raw = leerDistancia(TRIG_R, ECHO_R);

  distL_filtrada = filtroEMA(distL_raw, distL_filtrada);
  distR_filtrada = filtroEMA(distR_raw, distR_filtrada);

  long distL = distL_filtrada;
  long distR = distR_filtrada;

  switch (estado) {
    case SIGUIENDO:
      velocidadMotor = 180;
      controlPID(distL, distR);

      // detectar esquina
      if ((millis() - lastTurnTime > cooldownGiro) && detectarEsquina(distL, distR) && millis() > 9000) {

        estado = GIRANDO;
        anguloGyro = 0;

        // decidir direccion
        if (!primerGiro) {
          direccionIzquierda = distL > distR;
          primerGiro = true;
        }

        Serial.println(direccionIzquierda ? "Giro izquierda" : "Giro derecha");
      }
      break;

    case GIRANDO:
    {
      float delta = abs(anguloGyro);

      // desacelerar para giro
      if (delta < 45){
        velocidadMotor = 165;
      } else if (delta < 70){
        velocidadMotor = 145;
      } else{
        velocidadMotor = 120;
      }
        
      setMotor(velocidadMotor);

      // giro
      if (direccionIzquierda){
        escribirServo(150);
      }
      else{
        escribirServo(20);
      }
        
      // terminar giro
      if (delta >= AngGiro) {
        escribirServo(centroServo);
        velocidadMotor = 180;

        // reset PID paredes
        integralWall = 0;
        prevErrorWall = 0;

        // reset PID gyro
        integralGyro = 0;
        prevErrorGyro = 0;

        // recto
        anguloObjetivo = anguloGyro;
        lastTurnTime = millis();
        estado = SIGUIENDO;
        Serial.println("Giro completado");
      }

      break;
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
  // prints
  printDual(" | Estado:");
  printDual(String(estado));
  printDual(" | L:");
  printDual(String(distL));
  printDual(" | R:");
  printDual(String(distR));
  printDual(" | Ang:");
  printDual(String(anguloGyro));
  printDual(" | Obj:");
  printDual(String(anguloObjetivo));
  Serial.println(" ");
}