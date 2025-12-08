//========= Importação de Bibliotecas =============
#include <Wire.h>
#include <BH1750.h>
#include <DHTesp.h>

//========= GY-30 =============
#define SCL 22
#define SDA 21

//========= DHT11 =============
#define dhtPin 19

//========= KY-037 =============
#define soundPin 33

//========= Alarme =============
#define redPin 4
#define yellowPin 16
#define greenPin 17
#define buzzerPin 26

//========= Constantes e Variáveis do GY-30 =============
const unsigned long intervalLUX = 2000;
const unsigned long darkAlertTime = 120000;
unsigned long lastLuxRead = 0;
unsigned long lastDHTRead = 0;
unsigned long lastBuzzerAlert = 0;
unsigned long darkStartTime = 0;
bool darkPeriod = false; 
bool darkLightDetected = false;

//========= Constantes e Variáveis do DHT =============
const unsigned long intervalDHT = 10000;
int rangeOutTempTimes = 0;
int rangeOutRHTimes = 0;

//========= Constantes e Variáveis do KY-037 =============
unsigned long window = 1000;   // 1 segundo
unsigned long windowStart = 0;

// Contagem de tempo em HIGH
unsigned long highTime = 0;

// Parâmetros do score
int peakThreshold = 60;        // ajuste conforme ruído do ambiente
unsigned long peakDuration = 0;

//========= Constantes e Variáveis do Alarme Sonoro =============
const unsigned long intervalBuzzer = 30000; // 30 segundos
const int buzzDuration = 300;               // 300 ms
unsigned long lastMillis = 0;

//========= Objetos =============
BH1750 lightMeter;
DHTesp dht;

//========= Declaração de Funções =============
float readLuxSensor();
void readDHT11Sensor();
void readSoundSensor();
void activateAlarm();

//========= Setup =============
void setup(){
  Serial.begin(115200);

  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(soundPin, INPUT);

  Wire.begin(SDA, SCL);
  lightMeter.begin();

  dht.setup(dhtPin, DHTesp::DHT11);

  analogReadResolution(12);
  analogSetPinAttenuation(soundPin, ADC_11db);
  digitalWrite(buzzerPin, LOW);  // mantém desligado
  windowStart = millis();
}

//========= Loop =============
void loop() {
  unsigned long currentMillis = millis();
  /*-------- Bloco 1: Análise de Luminosidade --------*/
  if (millis() - lastLuxRead >= intervalLUX) {
    lastLuxRead = millis();

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
  if(millis() - lastDHTRead >= intervalDHT){ //falta a média de 1 min
    lastDHTRead = millis();

    bool rangeOutTemp = false;
    bool rangeOutRH   = false;
    TempAndHumidity data = dht.getTempAndHumidity();

    if(isnan(data.temperature))
      Serial.println("Failed to read from DHT11 sensor!");
    else{
      if(data.temperature < 22 || data.temperature > 26){
        //Alerta: médio
        rangeOutTemp = true;
      }
      if(data.humidity < 40 || data.humidity > 60){
        //Alerta: médio
        rangeOutRH = true;
      }

      // ------- LED de alerta médio -------
      if (rangeOutTemp || rangeOutRH) {
        digitalWrite(yellowPin, HIGH);
      } else {
        digitalWrite(yellowPin, LOW);
      }

      // ------- Contagem de vezes -------
      if(rangeOutTemp){
        rangeOutTempTimes++;
        Serial.print("Temperature out of range. Counting times: ");
        Serial.println(rangeOutTempTimes);
      } else {
        rangeOutTempTimes = 0;
      }

      if(rangeOutRH){
        rangeOutRHTimes++;
        Serial.print("Humidity out of range. Counting times: ");
        Serial.println(rangeOutRHTimes);
      } else {
        rangeOutRHTimes = 0;
      }

      // --- Alerta após 3 leituras consecutivas ---
      if (rangeOutTempTimes >= 3) {
        Serial.println("Temperature out of range for 3 readings");
      }

      if (rangeOutRHTimes >= 3) {
        Serial.println("Humidity out of range for 3 readings");
      }
    }
  }

  /*-------- Bloco 3: Análise de Ruído --------*/
  unsigned long now = millis();

  // Se D0 está HIGH, calcula quanto tempo ficou HIGH desde o último loop
 static unsigned long lastCheck = now;
  unsigned long elapsed = now - lastCheck;
  lastCheck = now;

  if (digitalRead(soundPin) == HIGH) {
      highTime += elapsed;   // agora sim: soma tempo real em ms
  }

  // Quando a janela termina (1s)
  if (now - windowStart >= window) {

      // Calcula score baseado na fração de HIGH dentro da janela
      int score = (highTime * 100) / window;  // 0–100%

      Serial.print("Score: ");
      Serial.println(score);

      // --- Detecção de pico ---
      if (score > peakThreshold) {
        peakDuration += window;   // acumula ms
      } else {
        peakDuration = 0;
      }

      if (peakDuration >= 3000) { // pico por 3 segundos
        Serial.println("ALERTA: Pico longo detectado!");
      }

      // Reinicia janela
      highTime = 0;
      windowStart = now;
  }
}

void readDHT11Sensor(){
  TempAndHumidity data = dht.getTempAndHumidity();

  if (isnan(data.temperature)) {
    Serial.println("DHT11 falhou!");
  } else {
    Serial.print("Temp: ");
    Serial.println(data.temperature);
    Serial.print("Hum: ");
    Serial.println(data.humidity);
  }

  delay(2000);
}

void activateBuzzer(){
  unsigned long currentMillis = millis();
  unsigned long elapsed = currentMillis - lastMillis;

  if (elapsed < buzzDuration) {
    digitalWrite(buzzerPin, HIGH); // Buzzer ligado
  } else {
    digitalWrite(buzzerPin, LOW);  // Buzzer desligado
  }

  if (elapsed >= intervalBuzzer) {
    lastMillis = currentMillis; // Reinicia o contador a cada intervalo
  }
}