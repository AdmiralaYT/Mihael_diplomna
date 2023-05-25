#include <PID_v1_bc.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <OneWire.h>

double currentTemperature = 0.0;
double wantedTemperature = 25.0;
double newWantedTemperature = 0.0;
double temperatureDifference = 0.0;
double btrec = 0.0;
int fanspeed = 50;
bool isValueReceived = false;
SoftwareSerial mySerial(3, 2);
int apptemp = 0;

const int ACT_LED_PIN = 12;
const int ADC_PIN = A0;
const int HEATER_PIN = 9;
const int HEATER_PIN1 = 11;
const int FAN_PIN = 5;
const int PWM_RANGE = 255;

double Kp = 2.0;
double Ki = 5.0;
double Kd = 1.0;
PID myPID(&currentTemperature, &wantedTemperature, &currentTemperature, Kp, Ki, Kd, DIRECT);

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
  SystemInit();
  TimeSliceSyst();
}

void SystemInit() {
  AdcReadCyclic();
  PidFuncCyclic();
  BT();
  float apptemp = currentTemperature;
  mySerial.print(currentTemperature);
  mySerial.println(" C\r\n");
  delay(100);
}

void TimeSliceSyst() {
  const unsigned long ADC_TIME_SLICE = 10;
  const unsigned long PID_TIME_SLICE = 10;
  const unsigned long PWM_TIME_SLICE = 10;
  const unsigned long DEBUG_TIME_SLICE = 10;
  const unsigned long LED_TIME_SLICE = 10;
  const unsigned long COM_TIME_SLICE = 10;
  const unsigned long IO_TIME_SLICE = 10;
  static unsigned long adcLastTime = 0;
  static unsigned long pidLastTime = 0;
  static unsigned long pwmLastTime = 0;
  static unsigned long debugLastTime = 0;
  static unsigned long ledLastTime = 0;
  static unsigned long comLastTime = 0;
  static unsigned long ioLastTime = 0;

  unsigned long currentMillis = millis();

  if ((currentMillis - adcLastTime) >= ADC_TIME_SLICE) {
    AdcReadCyclic();
    adcLastTime = currentMillis;
  }

  if ((currentMillis - pidLastTime) >= PID_TIME_SLICE) {
    PidFuncCyclic();
    pidLastTime = currentMillis;
  }

  if ((currentMillis - pwmLastTime) >= PWM_TIME_SLICE) {
    PwmDataCyclic();
    pwmLastTime = currentMillis;
  }

  if ((currentMillis - debugLastTime) >= DEBUG_TIME_SLICE) {
    DebugDataCyclic();
    debugLastTime = currentMillis;
  }

  if ((currentMillis - ledLastTime) >= LED_TIME_SLICE) {
    ActLedCyclic();
    ledLastTime = currentMillis;
  }

  if ((currentMillis - comLastTime) >= COM_TIME_SLICE) {
    ComInfoSendCyclic();
    comLastTime = currentMillis;
  }

  if ((currentMillis - ioLastTime) >= IO_TIME_SLICE) {
    IoDriverCyclic();
    ioLastTime = currentMillis;
  }
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

void PidFuncCyclic() {
  myPID.Compute();

  double temperatureDifference = wantedTemperature - currentTemperature;
  delay(100);
}

void BT() {
  if (mySerial.available() > 0) {
    btrec = mySerial.parseInt();
    if (btrec == 1567) {
      fanspeed = 50;
    } else if (btrec == 2567) {
      fanspeed = 128;
    } else if (btrec == 3567) {
      fanspeed = 255;
    } else {
      newWantedTemperature = btrec;
    }

    isValueReceived = true;
    delay(100);
  }
}

void PwmDataCyclic() {
  int pwmValue = currentTemperature * PWM_RANGE / 100;
  analogWrite(FAN_PIN, pwmValue);
}

void DebugDataCyclic() {
  Serial.print("Current Temperature: ");
  Serial.println(currentTemperature);
  Serial.print(" degrees Celsius\t");
  Serial.print("Wanted Temperature: ");
  Serial.print(wantedTemperature);
  Serial.println(" degrees Celsius");
}

void ActLedCyclic() {
  static unsigned long lastToggleTime = 0;
  static bool ledState = false;

  unsigned long currentTime = millis();
  if ((currentTime - lastToggleTime) >= 500) {
    lastToggleTime = currentTime;
    ledState = !ledState;
    digitalWrite(ACT_LED_PIN, ledState);
  }
}

void ComInfoSendCyclic() {
  if (Serial.available()) {
    mySerial.println(currentTemperature);
    wantedTemperature = Serial.read();
  }
}

void IoDriverCyclic() {
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

  if (isValueReceived) {
    wantedTemperature = newWantedTemperature;
    isValueReceived = false;
  }
}
