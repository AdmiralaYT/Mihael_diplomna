#include <PID_v1_bc.h>
#include <SoftwareSerial.h>
#include <InterpolationLib.h>

double currentTemperature;
double wantedTemperature = 25.0;
double newWantedTemperature;
double temperatureDifference;
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

double integral = 0.0; 
double derivative; 
double lastError = 0;
double heatingPower;

double NtcRead = 0.0;

double Voltage = 0.0;

const double Kp = 0.5;  // Пропорционален коефициент
const double Ki = 0.2;  // Интегрален коефициент
const double Kd = 0.1;  // Диференциален коефициент


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
}

void loop() {
  SystemInit();
  TimeSliceSyst();
}

void SystemInit() {
  AdcReadCyclic();
  NtcDataCyclic();
  PidFuncCyclic();
  BT();
  delay(100);
}

void TimeSliceSyst() {
const unsigned long ADC_TIME_SLICE = 1000;
const unsigned long NTC_TIME_SLICE = 1000;
const unsigned long PID_TIME_SLICE = 1000;
const unsigned long PWM_TIME_SLICE = 1000;
const unsigned long DEBUG_TIME_SLICE = 2000;
const unsigned long LED_TIME_SLICE = 100;
const unsigned long COM_TIME_SLICE = 100;
const unsigned long IO_TIME_SLICE = 100;
static unsigned long adcLastTime = 0;
static unsigned long ntcLastTime = 0;
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

if ((currentMillis - ntcLastTime) >= NTC_TIME_SLICE) {
NtcDataCyclic();
ntcLastTime = currentMillis;
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
  NtcRead = analogRead(ADC_PIN);
  Voltage = (NtcRead / 1023) * 5.0;
  delay(100);
}

void NtcDataCyclic(){
  const unsigned char numValues = 13;
double xValues_Voltage[13] = { 0.0, 0.384615385, 
                               0.769230769, 1.153846154,
                               1.538461538, 1.923076923,
                               2.307692308, 2.692307692,
                               3.076923077, 3.461538462, 
                               3.846153846, 4.230769231, 
                               4.615384615 };
                                 
  double yValues_Celsius[13] = {-40, -30,
                                -20, -10,
                                 0, 10,
                                20, 30, 
                                40, 50, 
                                60, 70, 
                                80 };

  currentTemperature = Interpolation::Linear(xValues_Voltage, yValues_Celsius, numValues, Voltage, true);
  apptemp = currentTemperature;
  mySerial.print(currentTemperature);
  mySerial.println(" C\r\n");
  delay(100);

 }

void PidFuncCyclic() {
  temperatureDifference = wantedTemperature - currentTemperature;
  integral += temperatureDifference * 0.1;
  derivative = (temperatureDifference - lastError) / 0.1;
  heatingPower = Kp * temperatureDifference + Ki * integral + Kd * derivative;
  lastError = temperatureDifference;
  
  //Serial.print("dif: ");
  //Serial.println(temperatureDifference);
  //Serial.print("  int: ");
  //Serial.println(integral);
  //Serial.print("  derivative: ");
  //Serial.print(derivative);
  //Serial.print("     ");
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
  analogWrite(HEATER_PIN, heatingPower);
  analogWrite(HEATER_PIN1, heatingPower);
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