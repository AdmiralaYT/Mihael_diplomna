#include <PID_v1_bc.h>

#include <SoftwareSerial.h>

const int pwmPin = 4; // Изходен пин за ШИМ сигнала
const int fanPin = 5; // Изходен пин за нагревателя

// Константи за настройка на PID регулатора
double Kp = 2;
double Ki = 5;
double Kd = 1;

// Променливи за PID регулатора
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const int analogPin = A0; // Аналогов пин, където е свързан NTC сензорът.

const int numReadings = 10; // Брой измервания за интерполацията.
int readings[numReadings];  // Масив за съхранение на измерванията.
int index = 0;              // Индекс за измерванията.
int total = 0;              // Обща сума на измерванията.

SoftwareSerial BTSerial(2, 3); // Пинове, свързани с модула за Bluetooth.

void setup() {
  Serial.begin(9600); // Стартирайте Serial Monitor на скорост 9600 бода.
  BTSerial.begin(9600); // Стартирайте SoftwareSerial на скорост 9600 бода.
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;  // Запълнете масива с нули.
      pinMode(pwmPin, OUTPUT);
  pinMode(fanPin, OUTPUT);

  // Инициализация на PID регулатора
  Setpoint = 50; // Желана температура
  Input = getTemperature(); // Текуща температура
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // Ограничаване на стойностите на ШИМ сигнала
  }
}

void loop() {
getTemperature();
PIDRegulator();
}
double getTemperature() {
  total = total - readings[index]; // Извадете най-старото измерване от общата сума.
  readings[index] = analogRead(analogPin); // Прочетете стойността на аналоговия пин и я запишете в масива.
  total = total + readings[index]; // Добавете новото измерване към общата сума.
  index = (index + 1) % numReadings; // Увеличете индекса за следващото измерване.
  int average = total / numReadings; // Пресметнете средната стойност на измерванията.
  float v = average * (70.0 / 1023.0); // Преобразувайте стойността във волтове.
  float temperature = map(v, 0.2, 70, -30, 40); // Интерполирайте стойността в градуси.
//  Serial.print("Temperature: ");
//  Serial.print(temperature);
//  Serial.println(" C");
  return temperature;
}

void PIDRegulator() {
    // Прочитане на стойността на температурния сензор
  Input = getTemperature();

  // Ако температурата е по-ниска от 15 градуса, пускаме нагревателя
  if (Input < 15) {
    digitalWrite(fanPin, HIGH);
  }
  else {
    // Изчисляване на стойността на PID регулатора
    myPID.Compute();
    
    // Установяване на стойността на ШИМ сигнала на изходния пин
    analogWrite(pwmPin, Output);
    digitalWrite(fanPin, LOW);
  }
  
  // Извеждане на стойността на температурата и на стойността на ШИМ сигнала
  Serial.print("Temperature: ");
  Serial.print(Input);
  Serial.print(" | PWM: ");
  Serial.println(Output);
  delay(100);
  }
