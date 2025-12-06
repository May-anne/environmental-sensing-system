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

const unsigned long intervalLUX = 2000;
const unsigned long darkAlertTime = 120000;
unsigned long lastLuxRead = 0;
unsigned long darkStartTime = 0;
bool darkPeriod = false; 
bool darkLightDetected = false;

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

  analogReadResolution(12);
  analogSetPinAttenuation(soundPin, ADC_11db);
}

void loop() {
  unsigned long currentMillis = millis();

  /*-------- Bloco 1: Análise de Luminosidade --------*/
  if (currentMillis - lastLuxRead >= intervalLUX) {
    lastLuxRead = currentMillis;

    float luxValue = lightMeter.readLightLevel();
    Serial.print("Lux: ");
    Serial.println(luxValue);

    // ---------- Período claro ----------
    if (!darkPeriod) {
      if (luxValue < 150 || luxValue > 300) { //Aprimorar depois
        Serial.println("Lux fora do ideal no período claro (150–300).");
        // coloque ação aqui (LED, buzzer, etc.)
      }
      darkLightDetected = false;
      darkStartTime = 0;
    }

    // ---------- Período escuro ----------
    else {
      if (luxValue > 0) {
        if (!darkLightDetected) { //Se detectar luz, inicie contagem
          darkStartTime = currentMillis;
          darkLightDetected = true;
        } else { //Luz ainda detectada detectada, meça o tempo
          if (currentMillis - darkStartTime >= darkAlertTime) {
            Serial.println("Luz detectada no período escuro por > 2 min.");
            // coloque ação aqui
          }
        }
      } else { //Lux não é zero (ou deixou de ser), resete flags
        darkLightDetected = false;
        darkStartTime = 0;
      }
    }
  }

  /*-------- Bloco 2: Análise de Umidade e Temperatura --------*/
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