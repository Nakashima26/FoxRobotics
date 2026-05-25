#include <ESP32Servo.h>

Servo miServo;

void setup() {
  Serial.begin(115200);
  miServo.attach(13, 500, 2400); 
}

void loop() {
  miServo.write(0);
  Serial.println(0);
  delay(2000);

  miServo.write(90);
  Serial.println(90);
  delay(2000);

  miServo.write(180);
  Serial.println(180);
  delay(2000);
}