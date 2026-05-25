#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL

  mpu.begin();
  mpu.calcGyroOffsets(true); // Calibración 

  Serial.println("MPU6050 listo");
}

void loop() {
  mpu.update();

  Serial.print(" | Z: ");
  Serial.println(mpu.getAngleZ());

  delay(200);
}