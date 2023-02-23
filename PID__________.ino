const int NUM_CHANNELS = 2;
const int heaterPins[NUM_CHANNELS] = {9, 10};
const int temperatureSensorPins[NUM_CHANNELS] = {A0, A1};
const int setPoints[NUM_CHANNELS] = {25, 30};
const int Kp = 2, Ki = 0.01, Kd = 5;

struct ChannelData {
  float temperature;
  float error;
  float integral;
  float derivative;
  float lastError;
  float heatingPower;
  int heaterPin;
  int temperatureSensorPin;
  int setPoint;
};

ChannelData channels[NUM_CHANNELS];

void setup() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channels[i].heaterPin = heaterPins[i];
    channels[i].temperatureSensorPin = temperatureSensorPins[i];
    channels[i].setPoint = setPoints[i];
    channels[i].integral = 0;
    channels[i].lastError = 0;
    pinMode(channels[i].heaterPin, OUTPUT);
    Serial.begin(9600);
  }
}

void loop() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    ChannelData& channel = channels[i];
    channel.temperature = readTemperature(channel.temperatureSensorPin);
    channel.error = channel.setPoint - channel.temperature;
    channel.integral = calculateIntegral(channel.error, 0.1, channel.integral);
    channel.derivative = calculateDerivative(channel.error, 0.1, channel.lastError);
    channel.heatingPower = Kp * channel.error + Ki * channel.integral + Kd * channel.derivative;
    channel.lastError = channel.error;

    analogWrite(channel.heaterPin, channel.heatingPower);
    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(channel.temperature);
  }
  delay(100);
}

float readTemperature(int temperatureSensorPin) {
  int reading = analogRead(temperatureSensorPin);
  return reading / 10.24;
}

float calculateIntegral(float error, float deltaTime, float integral) {
  integral += error * deltaTime;
  return integral;
}

float calculateDerivative(float error, float deltaTime, float lastError) {
  return (error - lastError) / deltaTime;
}
