#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>

#define SCL 22
#define SDA 21

#define redPin 4
#define yellowPin 16
#define greenPin 17
#define buzzerPin 25

#define dhtPin 14
#define soundPin 33

const int sampleWindow = 200; // ms

DHT dht11(dhtPin, DHT11);
BH1750 lightMeter;

float readLuxSensor();
void readDHT11Sensor();
void readSoundSensor();
void activateAlarm();

void setup(){
  Serial.begin(115200);

  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(soundPin, INPUT);

  Wire.begin(SDA, SCL);
  lightMeter.begin();

  dht11.begin();

  analogReadResolution(12);          // 0..4095
  analogSetPinAttenuation(soundPin, ADC_11db); // tenta faixa maior
}

void loop() {
}

float readLuxSensor(){
  float lux = lightMeter.readLightLevel();
  return lux;
}

void readDHT11Sensor(){
  float humi  = dht11.readHumidity();
  float tempC = dht11.readTemperature();

  if (isnan(tempC) || isnan(humi)) {
    Serial.println("Failed to read from DHT11 sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");

    Serial.print("  |  ");

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C  ~  ");
  }
}

void readSoundSensor(){
  unsigned long end = millis() + sampleWindow;
  int minV = 4095, maxV = 0, v;
  unsigned long samples = 0;
  while (millis() < end) {
    v = analogRead(soundPin);
    if (v < minV) minV = v;
    if (v > maxV) maxV = v;
    samples++;
  }
  int p2p = maxV - minV;
  Serial.print("samples="); Serial.print(samples);
  Serial.print("  min="); Serial.print(minV);
  Serial.print("  max="); Serial.print(maxV);
  Serial.print("  p2p="); Serial.println(p2p);
}

void activateAlarm(){

}