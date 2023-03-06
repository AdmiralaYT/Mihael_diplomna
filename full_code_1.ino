#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1_bc.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define ADC_PIN A0
#define RELAY_PIN 2
#define PWM_PIN 9
#define ACT_LED_PIN 13

int currentTemperature = 0;
int targetTemperature = 0;

double pidSetpoint = 0;
double pidInput = 0;
double pidOutput = 0;
PID pid(&pidInput, &pidOutput, &pidSetpoint, 5, 1, 1, DIRECT);

void SystemInit() {
pinMode(RELAY_PIN, OUTPUT);
pinMode(PWM_PIN, OUTPUT);
pinMode(ACT_LED_PIN, OUTPUT);
Serial.begin(9600);
pid.SetMode(AUTOMATIC);
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT); 
  pinMode(PWM_PIN, OUTPUT); 
  pinMode(ACT_LED_PIN, OUTPUT);
  Serial.begin(9600);
  pid.SetMode(AUTOMATIC);
}

void loop() {
  AdcReadCyclic();
  NtcDataCyclic();
  PidFuncCyclic();
  PwmDataCyclic();
  DebugDataCyclic();
  ActLedCyclic();
  TimeSliceSyst();
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
int adcValue = analogRead(ADC_PIN);
float voltage = (adcValue / 1023.0) * 5.0;
currentTemperature = voltage / 0.01;
}

void NtcDataCyclic() {
if (currentTemperature < targetTemperature) {
digitalWrite(RELAY_PIN, HIGH);
} else {
digitalWrite(RELAY_PIN, LOW);
}
}

void PidFuncCyclic() {
pidSetpoint = targetTemperature;
pidInput = currentTemperature;
pid.Compute();
Serial.print("PID Output: ");
Serial.println(pidOutput);
}

void PwmDataCyclic() {
int pwmValue = pidOutput * 255 / 100;
analogWrite(PWM_PIN, pwmValue);
}

void DebugDataCyclic() {
Serial.print("Current Temperature: ");
Serial.print(currentTemperature);
Serial.print(", Target Temperature: ");
Serial.println(targetTemperature);
}

void ActLedCyclic() {
static bool ledState = LOW;
ledState = !ledState;
digitalWrite(ACT_LED_PIN, ledState);
}

void ComInfoSendCyclic() {
Serial.print("T:");
Serial.print(currentTemperature);
Serial.print(",Tg:");
Serial.println(targetTemperature);
}

void IoDriverCyclic() {
if (pidOutput > 50 && currentTemperature < targetTemperature) {
digitalWrite(RELAY_PIN, HIGH);
} else {
digitalWrite(RELAY_PIN, LOW);
}
}
