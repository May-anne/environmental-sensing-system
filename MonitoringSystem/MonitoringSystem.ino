#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

#define SCL 22
#define SDA 21
#define ledPin 19
#define buzzerPin 18
#define dhtPin 

void setup(){
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  Wire.begin(SDA, SCL);
  lightMeter.begin();

  Serial.println(F("BH1750 Test begin"));
  digitalWrite(ledPin, HIGH);
}

void loop() {
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  delay(1000);
}