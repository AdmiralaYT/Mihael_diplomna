const int thermistorPin = A0; // пин на аналогов вход на Arduino, където е подключен NTC термистора
const int B = 3950; // коефициент на термистора
const int R0 = 10000; // начална стойност на съпротивлението на термистора при 25°C

void setup() {
  Serial.begin(9600); // инициализиране на серийния порт
}

void loop() {
  int reading = analogRead(thermistorPin); // прочитане на аналоговата стойност от NTC термистора
  float resistance = (1023.0 / reading - 1.0) * R0; // изчисляване на съпротивлението на термистора
  float temperature = 1.0 / (log(resistance / R0) / B + 1 / 298.15) - 273.15; // изчисляване на температурата

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");

  delay(1000); // задържане на 1 секунда преди да се изпълни следващата итерация
}