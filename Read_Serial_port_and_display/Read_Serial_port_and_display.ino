#include <HardwareSerial.h>

HardwareSerial SerialPort(2);

void setup() {
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  Serial.begin(115200);
}

void loop() {
  Serial.println("Reading output...");
  if (SerialPort.available()) {
    Serial.println(SerialPort.readString());
  }
  delay(1000);
}
