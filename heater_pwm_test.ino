#include <PID_v1_bc.h>

#include <SoftwareSerial.h>

const int pwmPin = 4; // Изходен пин за ШИМ сигнала
const int fanPin = 5; // Изходен пин за нагревателя


double Kp = 2;
double Ki = 5;
double Kd = 1;

// Променливи за PID регулатора
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const int analogPin = A0; // Аналогов пин, където е свързан NTC сензорът.

const int numReadings = 10; 
int readings[numReadings];
int index = 0;      
int total = 0;        

SoftwareSerial BTSerial(2, 3); // Пинове, свързани с модула за Bluetooth.

void setup() {
  Serial.begin(9600); 
  BTSerial.begin(9600); 
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;  
      pinMode(pwmPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  Setpoint = 50; 
  Input = getTemperature(); // Текуща температура
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); 
  }
}

void loop() {
getTemperature();
PIDRegulator();
}
double getTemperature() {
  total = total - readings[index]; 
  readings[index] = analogRead(analogPin); 
  total = total + readings[index]; 
  index = (index + 1) % numReadings;
  int average = total / numReadings; 
  float v = average * (70.0 / 1023.0); 
  float temperature = map(v, 0.2, 70, -30, 40); 
//  Serial.print("Temperature: ");
//  Serial.print(temperature);
//  Serial.println(" C");
  return temperature;
}

void PIDRegulator() {
  Input = getTemperature();
  if (Input < 15) {
    digitalWrite(fanPin, HIGH);
  }
  else {
    myPID.Compute();
    analogWrite(pwmPin, Output);
    digitalWrite(fanPin, LOW);
  }
  
  Serial.print("Temperature: ");
  Serial.print(Input);
  Serial.print(" | PWM: ");
  Serial.println(Output);
  delay(100);
  }
