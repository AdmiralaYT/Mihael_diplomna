#include <SoftwareSerial.h>

const byte rxPin = 3;
const byte txPin = 2;

SoftwareSerial mySerial (rxPin, txPin);

void setup() {
  mySerial.begin(9600);
  
}

void loop() {
  mySerial.println("AT");
  delay(500);
  if (mySerial.available() > 0) {
    String response = mySerial.readString();
    mySerial.println("Отговор от HC-05: " + response);
  }

  delay(1000);
}