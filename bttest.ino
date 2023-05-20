#include <SoftwareSerial.h>

const byte rxPin = 3;
const byte txPin = 2;

SoftwareSerial mySerial (rxPin, txPin);

void setup() {
  mySerial.begin(9600);
  
}

void loop() {
  Serial.println("AT");
  delay(500);
  if (Serial.available() > 0) {
    String response = Serial.readString();
    Serial.println("Отговор от HC-05: " + response);
  }

  delay(1000);
}