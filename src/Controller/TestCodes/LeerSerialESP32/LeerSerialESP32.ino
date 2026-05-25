#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);      // Monitor serial (PC)
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // UART con Raspberry Pi

  Serial.println("Listo para recibir datos...");
}

void loop() {
  // Si hay datos desde la Raspberry Pi
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.print(c);  // Lo mostramos en el monitor serial
    Serial2.write(c);
  }

}