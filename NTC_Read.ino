const int thermistorPin = A0;
const int B = 3950; 
const int R0 = 10000; 

void setup() {
  Serial.begin(9600);
}

void loop() {
  int reading = analogRead(thermistorPin); 
  float resistance = (1023.0 / reading - 1.0) * R0; 
  float temperature = 1.0 / (log(resistance / R0) / B + 1 / 298.15) - 273.15; 

  Serial.print("Температура: ");
  Serial.print(temperature);
  Serial.println("°C");

  delay(1000);
}