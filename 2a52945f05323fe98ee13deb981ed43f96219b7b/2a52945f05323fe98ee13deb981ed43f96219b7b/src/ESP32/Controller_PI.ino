#include <Wire.h>
#include <MPU6050_tockn.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
MPU6050 mpu(Wire);

// PINES
#define TRIG_L 27
#define ECHO_L 32
#define TRIG_R 26
#define ECHO_R 35

#define PWMA 23
#define A1 18
#define A2 19
#define SERVO_PIN 13

// PWM
const int freqServo = 50;
const int resServo = 16;
const int freqMotor = 1000;
const int resMotor = 8;

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

float errorGyro = 0;
float prevErrorGyro = 0;
float integralGyro = 0;

// TIEMPO
unsigned long lastPIDTime = 0;
unsigned long lastGyroTime = 0;

// GIROSCOPIO
float anguloGyro = 0;
float anguloObjetivo = 0;

// CONTROL
int velocidadMotor = 180;
int centroServo = 80;

// INTEGRACION PI -> ESP32
float obsBiasNorm = 0.0;     // [-1, 1]
int turnHint = 0;            // -1 derecha, 0 ninguno, +1 izquierda
unsigned long lastPiMsgMs = 0;
const unsigned long piTimeoutMs = 250;
bool piPriority = false;
int piMemoryFrames = 0;
bool piReady = false;
unsigned long bootStartMs = 0;

const bool WAIT_PI_AT_BOOT = true;
const bool ALLOW_FALLBACK_AFTER_WAIT = true;
const unsigned long BOOT_WAIT_PI_MS = 12000;

float visionSteerGain = 18.0;
float turnHintGain = 7.0;

// ESTADOS
enum Estado {
  SIGUIENDO,
  GIRANDO
};

Estado estado = SIGUIENDO;

// GIROS
bool direccionIzquierda = true;
bool primerGiro = false;
int AngGiro = 82;
unsigned long lastTurnTime = 0;
const int cooldownGiro = 2000;

// DETECCION
int contadorEsquina = 0;
const int umbralPared = 100;

// CARRERA
int turnsCompleted = 0;
bool raceFinished = false;
const int TURNS_PER_RACE = 12;

// FILTRO
float alpha = 0.75;
float distL_filtrada = 0;
float distR_filtrada = 0;

void printDual(String txt) {
  Serial.print(txt);
  SerialBT.print(txt);
}

void escribirServo(int angulo) {
  angulo = constrain(angulo, 0, 180);
  int pulso = map(angulo, 0, 180, 500, 2500);
  int duty = (pulso * ((1 << resServo) - 1)) / 20000;
  ledcWrite(SERVO_PIN, duty);
}

void setMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  ledcWrite(PWMA, velocidad);
}

long leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duracion = pulseIn(echo, HIGH, 8000);
  long distancia = duracion * 0.034 / 2;

  if (distancia == 0 || distancia > 200) {
    distancia = 200;
  }

  return distancia;
}

float filtroEMA(float nueva, float anterior) {
  return alpha * nueva + (1.0 - alpha) * anterior;
}

bool detectarEsquina(long distL, long distR) {
  bool apertura = (distL > umbralPared) || (distR > umbralPared);
  if (apertura) {
    contadorEsquina++;
  } else {
    contadorEsquina = 0;
  }
  return contadorEsquina >= 3;
}

void actualizarGyro() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0;
  lastGyroTime = now;

  float gyroZ = mpu.getGyroZ();
  if (abs(gyroZ) < 1.0) {
    gyroZ = 0;
  }

  anguloGyro += gyroZ * dt;
}

void parsePiMessage(String line) {
  line.trim();

  // Handshake de arranque: la Pi envia READY cuando termino de levantar.
  if (line.startsWith("READY")) {
    piReady = true;
    lastPiMsgMs = millis();
    Serial.println("ACK:READY");
    return;
  }

  // Nuevo formato recomendado:
  // V1,obs=+0.123,turn=0,state=avoid_red
  if (line.startsWith("V1,")) {
    int obsIdx = line.indexOf("obs=");
    int turnIdx = line.indexOf("turn=");
    int prioIdx = line.indexOf("prio=");
    int memIdx = line.indexOf("mem=");

    if (obsIdx >= 0) {
      int obsEnd = line.indexOf(',', obsIdx);
      String obsStr = (obsEnd >= 0) ? line.substring(obsIdx + 4, obsEnd) : line.substring(obsIdx + 4);
      obsBiasNorm = obsStr.toFloat();
      obsBiasNorm = constrain(obsBiasNorm, -1.0, 1.0);
    }

    if (turnIdx >= 0) {
      int turnEnd = line.indexOf(',', turnIdx);
      String turnStr = (turnEnd >= 0) ? line.substring(turnIdx + 5, turnEnd) : line.substring(turnIdx + 5);
      int turnVal = turnStr.toInt();
      if (turnVal > 0) {
        turnHint = 1;
      } else if (turnVal < 0) {
        turnHint = -1;
      } else {
        turnHint = 0;
      }
    }

    if (prioIdx >= 0) {
      int prioEnd = line.indexOf(',', prioIdx);
      String prioStr = (prioEnd >= 0) ? line.substring(prioIdx + 5, prioEnd) : line.substring(prioIdx + 5);
      piPriority = (prioStr.toInt() != 0);
    }

    if (memIdx >= 0) {
      int memEnd = line.indexOf(',', memIdx);
      String memStr = (memEnd >= 0) ? line.substring(memIdx + 4, memEnd) : line.substring(memIdx + 4);
      piMemoryFrames = memStr.toInt();
      if (piMemoryFrames < 0) {
        piMemoryFrames = 0;
      }
    }

    piReady = true;
    lastPiMsgMs = millis();
    Serial.println("ACK:V1");
    return;
  }

  // Modo legado: solo pixel X (como controlPI.py original)
  bool numeric = true;
  for (unsigned int i = 0; i < line.length(); i++) {
    char c = line.charAt(i);
    if (!(c >= '0' && c <= '9')) {
      numeric = false;
      break;
    }
  }

  if (numeric && line.length() > 0) {
    int x = line.toInt();
    // Convierte 0..640 a -1..1. Si llega 700 (neutral), cae cerca de +1, asi que saturamos por rango.
    if (x >= 0 && x <= 640) {
      obsBiasNorm = (float(x) - 320.0) / 320.0;
      obsBiasNorm = constrain(obsBiasNorm, -1.0, 1.0);
      piReady = true;
      lastPiMsgMs = millis();
      Serial.println("ACK:X");
    }
  }
}

void readPiSerial() {
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    if (line.length() > 0) {
      parsePiMessage(line);
    }
  }
}

void controlPID(long distL, long distR) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  if (dt < 0.01) {
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

  float outputFinal = outputWall + outputGyro;

  bool piAlive = (millis() - lastPiMsgMs) <= piTimeoutMs;
  float outputVision = 0;

  if (piAlive) {
    float localVisionGain = visionSteerGain;
    if (piPriority) {
      localVisionGain *= 1.20;
    }
    outputVision = (obsBiasNorm * localVisionGain) + (float(turnHint) * turnHintGain);
  } else {
    piPriority = false;
    piMemoryFrames = 0;
    turnHint = 0;
    obsBiasNorm = 0.0;
  }

  outputFinal += outputVision;
  outputFinal = constrain(outputFinal, -25, 25);

  escribirServo(centroServo + outputFinal);
  setMotor(velocidadMotor);

  printDual(" | Wall:");
  printDual(String(outputWall));
  printDual(" | Gyro:");
  printDual(String(outputGyro));
  printDual(" | Vis:");
  printDual(String(outputVision));
  printDual(" | Servo:");
  printDual(String(centroServo + outputFinal));
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot");

  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);

  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);

  ledcAttach(PWMA, freqMotor, resMotor);
  ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServo(centroServo);
  delay(1000);

  distL_filtrada = leerDistancia(TRIG_L, ECHO_L);
  distR_filtrada = leerDistancia(TRIG_R, ECHO_R);

  lastPIDTime = millis();
  lastGyroTime = millis();
  lastPiMsgMs = 0;
  bootStartMs = millis();

  anguloObjetivo = 0;
  Serial.println("Sistema listo (Controller_PI)");
}

void loop() {
  readPiSerial();

  bool bootWaitActive = false;
  if (WAIT_PI_AT_BOOT && !piReady) {
    if (ALLOW_FALLBACK_AFTER_WAIT) {
      bootWaitActive = (millis() - bootStartMs) < BOOT_WAIT_PI_MS;
    } else {
      bootWaitActive = true;
    }
  }

  if (bootWaitActive) {
    escribirServo(centroServo);
    setMotor(0);
    printDual(" | Estado:WAIT_PI");
    printDual(" | t=");
    printDual(String((millis() - bootStartMs) / 1000.0, 1));
    Serial.println(" ");
    SerialBT.println(" ");
    delay(20);
    return;
  }

  if (raceFinished) {
    setMotor(0);
    escribirServo(centroServo);
    printDual(" | TERMINADO giros=12/12");
    Serial.println(" ");
    SerialBT.println(" ");
    delay(20);
    return;
  }

  mpu.update();
  actualizarGyro();

  long distL_raw = leerDistancia(TRIG_L, ECHO_L);
  long distR_raw = leerDistancia(TRIG_R, ECHO_R);

  distL_filtrada = filtroEMA(distL_raw, distL_filtrada);
  distR_filtrada = filtroEMA(distR_raw, distR_filtrada);

  long distL = distL_filtrada;
  long distR = distR_filtrada;

  switch (estado) {
    case SIGUIENDO: {
      velocidadMotor = 180;
      controlPID(distL, distR);

      bool bloqueadoPorObstaculo = piPriority || (piMemoryFrames > 0);
      if ((millis() - lastTurnTime > cooldownGiro) && !bloqueadoPorObstaculo && detectarEsquina(distL, distR) && millis() > 9000) {
        estado = GIRANDO;
        anguloGyro = 0;

        if (!primerGiro) {
          direccionIzquierda = distL > distR;
          primerGiro = true;
        }

        Serial.println(direccionIzquierda ? "Giro izquierda" : "Giro derecha");
      }
      break;
    }

    case GIRANDO: {
      float delta = abs(anguloGyro);

      if (delta < 45) {
        velocidadMotor = 165;
      } else if (delta < 70) {
        velocidadMotor = 145;
      } else {
        velocidadMotor = 120;
      }

      setMotor(velocidadMotor);

      if (direccionIzquierda) {
        escribirServo(150);
      } else {
        escribirServo(20);
      }

      if (delta >= AngGiro) {
        escribirServo(centroServo);
        velocidadMotor = 180;

        integralWall = 0;
        prevErrorWall = 0;

        integralGyro = 0;
        prevErrorGyro = 0;

        anguloObjetivo = anguloGyro;
        lastTurnTime = millis();
        estado = SIGUIENDO;
        turnsCompleted++;
        if (turnsCompleted >= TURNS_PER_RACE) {
          raceFinished = true;
        }
        Serial.print("Giro completado ");
        Serial.print(turnsCompleted);
        Serial.print("/");
        Serial.println(TURNS_PER_RACE);
      }

      break;
    }
  }

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
  printDual(" | obsNorm:");
  printDual(String(obsBiasNorm, 3));
  printDual(" | turn:");
  printDual(String(turnHint));
  printDual(" | prio:");
  printDual(String(piPriority ? 1 : 0));
  printDual(" | mem:");
  printDual(String(piMemoryFrames));
  printDual(" | giros:");
  printDual(String(turnsCompleted));
  printDual("/");
  printDual(String(TURNS_PER_RACE));
  Serial.println(" ");
  SerialBT.println(" ");
}
