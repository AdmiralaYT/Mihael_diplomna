const int heaterPin = 9;
const int temperatureSensorPin = A0;
const int setPoint = 25;
const int Kp = 2, Ki = 0.01, Kd = 5;

float temperature; 
float error; 
float integral = 0; 
float derivative; 
float lastError = 0;
float heatingPower;

void setup() {
  pinMode(heaterPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  temperature = readTemperature();
  error = setPoint - temperature; 
  integral = integral + error * 0.1;
  derivative = (error - lastError) / 0.1;
  heatingPower = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  analogWrite(heaterPin, heatingPower);
  Serial.println(temperature);

  delay(100);
}

float readTemperature() {
  int reading = analogRead(temperatureSensorPin);
  return reading / 10.24;
}

float calculateIntegral(float error, float deltaTime) {
  integral += error * deltaTime;
  return integral;
}

float calculateDerivative(float error, float deltaTime) {
  derivative = (error - lastError) / deltaTime;
  return derivative;
}