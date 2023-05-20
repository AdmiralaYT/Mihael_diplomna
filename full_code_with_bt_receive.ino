#include <PID_v1_bc.h>
#include <SoftwareSerial.h>
double currentTemperature = 0.0;
double wantedTemperature = 25.0;
int fanspeed = 100;

int asd = 1;

SoftwareSerial mySerial(0, 1);

const int ADC_PIN = A0;
const int HEATER_PIN = 9;
const int HEATER_PIN1 = 11;
const int FAN_PIN = 5;
const int PWM_RANGE = 255;


double kp = 2.0;   
double ki = 5.0;   
double kd = 1.0;    

PID myPID(&currentTemperature, &wantedTemperature, &currentTemperature, kp, ki, kd, DIRECT);

const int MIN_TEMPERATURE = -30;
const int MAX_TEMPERATURE = 40;

const float MIN_ADC_VALUE = 0.0;
const float MAX_ADC_VALUE = 1023.0;
const float MIN_VOLTAGE = 0.0;
const float MAX_VOLTAGE = 0.5;

void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(HEATER_PIN1, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  analogWrite(FAN_PIN, 0); 
  Serial.begin(9600);
  mySerial.begin(9600); 
  myPID.SetMode(MANUAL);
  myPID.SetOutputLimits(0, 0);
}

void loop() {
  AdcReadCyclic();
  PidFuncCyclic();
  BT();
}

void AdcReadCyclic() {
  int adcValue = analogRead(ADC_PIN);
  float voltage = mapFloat(adcValue, MIN_ADC_VALUE, MAX_ADC_VALUE, MIN_VOLTAGE, MAX_VOLTAGE);
  currentTemperature = mapFloat(voltage, MIN_VOLTAGE, MAX_VOLTAGE, MIN_TEMPERATURE, MAX_TEMPERATURE);
  delay(100);
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void PidFuncCyclic(){
  myPID.Compute();

  double temperatureDifference = wantedTemperature - currentTemperature;

  if (temperatureDifference > 0) {
    digitalWrite(HEATER_PIN, HIGH);
    digitalWrite(HEATER_PIN1, HIGH);
    analogWrite(FAN_PIN, 0);
  } else if (temperatureDifference < 0) {
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(HEATER_PIN1, LOW);
    analogWrite(FAN_PIN, fanspeed);
  } else {
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(HEATER_PIN1, LOW);
    analogWrite(FAN_PIN, 0);
  }

  //Serial.print("Current Temperature: ");
  //Serial.print(currentTemperature);
  //Serial.print(" degrees Celsius\t");
  //Serial.print("Wanted Temperature: ");
  //Serial.print(wantedTemperature);
  //Serial.println(" degrees Celsius");

  delay(100);
}

void BT(){
    if (mySerial.available() > 0) {
    String response = mySerial.readString();
    Serial.println(response);
    delay(100);
  }

Serial.println(asd);
delay(100);
}