//========= Importação de Bibliotecas =============
#include <Wire.h>
#include <BH1750.h>
#include <DHTesp.h>
#include <LittleFS.h>
#include <ArduinoJson.h> 

//========= GY-30 ============
#define SCL 22
#define SDA 21

//========= DHT11 ============
#define dhtPin 19

//========= KY-037 ============
#define soundPin 33

//========= Alarme ============
#define redPin 4
#define yellowPin 16
#define greenPin 17
#define buzzerPin 26
#define buttonPin 27

//========= Constantes e Variáveis do GY-30 =============
const unsigned long intervalLUX = 2000;     // 2s
const unsigned long darkAlertTime = 120000; // 2 min para escuro
unsigned long lastLuxRead = 0;
unsigned long lastDHTRead = 0;
unsigned long lastBuzzerAlert = 0;
unsigned long darkStartTime = 0;
bool darkPeriod = false;
bool darkLightDetected = false;

//========= Constantes e Variáveis do DHT ============
const unsigned long intervalDHT = 10000; // 10s
int rangeOutTempTimes = 0; 
int rangeOutRHTimes = 0; 

//========= Constantes e Variáveis do KY-037 ============
unsigned long window = 1000;      // Tempo da Janela de 1 segundo
unsigned long windowStart = 0;    // Início da janela
unsigned long highTime = 0;       // Contagem de tempo em HIGH
// Parâmetros do score
int peakThreshold = 60;           // Ajuste conforme ruído do ambiente
unsigned long peakDuration = 0;   // Duração do pico

//========= Constantes e Variáveis do Alarme Sonoro ============
const unsigned long intervalBuzzer = 30000; // 30 segundos
const int buzzDuration = 300;               // 300 ms
unsigned long lastMillis = 0;

// ========= Constantes para médias/histerese/limites ==========
const float TEMP_OK_MIN = 22.0, TEMP_OK_MAX = 26.0;
const float TEMP_ATT_MIN = 20.0, TEMP_ATT_MAX = 28.0;

const float RH_OK_MIN   = 40.0, RH_OK_MAX   = 60.0;
const float RH_ATT_MIN  = 35.0, RH_ATT_MAX  = 65.0;

const float LUX_OK_MIN  = 150.0, LUX_OK_MAX = 300.0;
const float LUX_ATT_MIN = 100.0, LUX_ATT_MAX = 500.0;

const int NOISE_OK_MAX  = 40; const int NOISE_ATT_MAX = 60;

// Número de Amostras (Janelas)
const uint8_t LUX_WIN = 30;   // 2s * 30 = 60s
const uint8_t DHT_WIN = 6;    // 10s * 6 = 60s
const uint8_t NOISE_WIN = 60; // 1s * 60 = 60s

// Buffers circulares
float luxBuf[LUX_WIN];      uint8_t luxHead=0,  luxCount=0;
float tempBuf[DHT_WIN];     uint8_t tempHead=0, tempCount=0;
float humBuf[DHT_WIN];      uint8_t humHead=0,  humCount=0;
int   noiseBuf[NOISE_WIN];  uint8_t noiseHead=0, noiseCount=0;

// Médias móveis
float luxAvg1m=0.0, tempAvg1m=0.0, rhAvg1m=0.0;
float noiseAvg1m=0.0;

// Máx e Min
float luxMin = 32555.0;   float luxMax = 0.0;
float tempMin = 32555.0;  float tempMax = 0.0;
float rhMin= 100.0;       float rhMax = 0.0;
float noiseMin = 100.0;   float noiseMax = 0.0; float noisePeak = 0.0;

// Histerese / contagens
uint8_t outTempCount=0, inTempCount=0;
uint8_t outRHCount=0,   inRHCount=0;
uint8_t outLuxCount=0,  inLuxCount=0;
uint8_t outNoiseCount=0,inNoiseCount=0;

// Tempos em condição crítica (segundos acumulados)
uint16_t critTempSec=0, critRHSec=0, critLuxSec=0, critNoiseSec=0;
uint16_t multiOutSec=0;

// Flag para indicar alarme por luz no período escuro
bool darkAlarm = false;

const unsigned long RUN_24H = 24UL * 60UL * 60UL * 1000UL; // 24h em ms
const unsigned long RUN_1MIN = 1UL * 60UL * 1000UL; // 2 MIN em ms
unsigned long startMillis = 0;
bool finished24h = false;

float noiseMin24h = 100;
float noiseMax24h = 0;

//Contadores globais
uint32_t tempTotalSamples = 0; uint32_t rhTotalSamples = 0;
uint32_t tempOutSamples   = 0; uint32_t rhOutSamples   = 0;

uint32_t luxTotalSamples = 0; uint32_t noiseTotalSamples = 0;
uint32_t luxOutSamples   = 0; uint32_t noiseOutSamples   = 0;

uint16_t dayCounter = 0;

// ======= DETECÇÃO DIA/NOITE - 10 MINUTOS (Rápida) ========
const float LUX_DN_NIGHT = 40.0;   // Threshold detecção rápida
const float LUX_DN_DAY   = 120.0;
const uint32_t DN_TIME   = 10UL * 60UL * 1000UL; // 10 minutos

enum DayNightState {
  DN_UNKNOWN,
  DN_DAY,
  DN_NIGHT
};

DayNightState dnState = DN_UNKNOWN;  // Estado detecção rápida (10 min)

// ======= CICLO MACRO DIA/NOITE - 6 HORAS (Confirmação de longo prazo) ========
const float LUX_MACRO_NIGHT = 40.0;  // Threshold macro
const float LUX_MACRO_DAY   = 120.0;
const uint32_t MACRO_CONFIRM_TIME = 6UL * 60UL * 60UL * 1000UL; // 6 horas em ms

DayNightState macroState = DN_UNKNOWN;  // Estado macro (6h)

// ============ Variáveis do botão ============
volatile bool buttonPressed = false;
bool buzzerMuted = false;
unsigned long muteStartMillis = 0;
const unsigned long MUTE_TIME = 10UL * 60UL * 1000UL; // 10 minutos

// Estado global
enum SystemState { OK_STATE, ATENCAO_STATE, ALARME_STATE };
SystemState sysState = OK_STATE;

//========= Declaração de Funções ============
float readLuxSensor();
void readDHT11Sensor();
void readSoundSensor();
void activateBuzzer();
void updateDayNight10min(float luxAvg1m, unsigned long now);
void updateMacroCycle(float luxAvg1m, unsigned long now);
bool nightNow();
bool detectNightLight(float luxValue, unsigned long now);
void createCSV(const char *filename);
void createFile(const char *filename);
void printFile(const char *filename);
void loadDayCounter();
void saveDayCounter();
void resetMetrics();
void printDayNightStatus();

//========= Funções auxiliares ============
//Buffer Circular com valores da média
template<typename T>
void pushCirc(T* buf, uint8_t win, uint8_t &head, uint8_t &cnt, T v){
  buf[head] = v;
  head = (head + 1) % win;
  if (cnt < win) cnt++;
}

//Calcula média aritmética dos valores no buffer
template<typename T>
float avgBuf(T* buf, uint8_t cnt){
  if (cnt==0) return 0.0;
  long double s=0;
  for(uint8_t i=0;i<cnt;i++) s += buf[i];
  return (float)(s / cnt);
}

//========= Objetos ============
BH1750 lightMeter;
DHTesp dht;

// ============ Interrupção do Botão ============
void IRAM_ATTR handleButtonPress() {
  buttonPressed = true;
}

//========= Setup ============
void setup(){
  Serial.begin(115200);

  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  Serial.println("LittleFS mounted successfully");
  loadDayCounter(); 

  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(soundPin, INPUT);
  pinMode(buttonPin, INPUT);

  Wire.begin(SDA, SCL);
  lightMeter.begin();

  dht.setup(dhtPin, DHTesp::DHT11);

  analogReadResolution(12);
  analogSetPinAttenuation(soundPin, ADC_11db);
  digitalWrite(buzzerPin, LOW);
  windowStart = millis();

  // inicializa LEDs
  digitalWrite(greenPin, HIGH);
  digitalWrite(yellowPin, LOW);
  digitalWrite(redPin, LOW);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);

  startMillis = millis();
  Serial.println(">>> INICIO DA COLETA 24H");
}

//========= Loop ============
void loop() {
  // --------- BOTÃO DE SILENCIAMENTO ---------
  if (buttonPressed) {
    buttonPressed = false;

    if (sysState == ALARME_STATE) {
      buzzerMuted = true;
      muteStartMillis = millis();
      Serial.println(">>> ALARME SILENCIADO POR 10 MINUTOS");
    }
  }

  unsigned long currentMillis = millis();
  
  /*-------- Bloco 1: Análise de Luminosidade (cada 2s) --------*/
  if (currentMillis - lastLuxRead >= intervalLUX) {
    lastLuxRead = currentMillis;

    float luxValue = lightMeter.readLightLevel();
    
    //Máximo e Mínimo
    if(luxValue > luxMax)
      luxMax = luxValue;

    if(luxValue < luxMin)
      luxMin = luxValue;

    Serial.print("Lux (raw): ");
    Serial.println(luxValue);

    // empilha para média 1 min
    pushCirc(luxBuf, LUX_WIN, luxHead, luxCount, luxValue);
    luxAvg1m = avgBuf(luxBuf, luxCount);

    luxTotalSamples++;
    if (luxAvg1m < LUX_OK_MIN || luxAvg1m > LUX_OK_MAX) {
        luxOutSamples++;
    }

    // Atualiza ambas as detecções dia/noite
    updateDayNight10min(luxAvg1m, currentMillis);
    updateMacroCycle(luxAvg1m, currentMillis);

  // Detecta luz indevida durante a noite (usando luxAvg1m)
  bool nightLightAlarm = detectNightLight(luxAvg1m, currentMillis); // Usando luxAvg1m, média de 1 minuto

  if (nightLightAlarm) {
      darkAlarm = true;
      critLuxSec = 300; // Alarme se luz indevida for detectada por mais de 2 minutos
      Serial.println("*** LUZ INDEVIDA DETECTADA DURANTE A NOITE ***");
  }

  // ---------- Período claro ----------
  if (!nightNow()) {  // Período claro, dia
    bool insideOK = (luxAvg1m >= LUX_OK_MIN && luxAvg1m <= LUX_OK_MAX); // Verifica se a luz está dentro da faixa normal
    bool outside  = !insideOK; // Fora da faixa ideal de luz
    bool critical = (luxAvg1m < LUX_ATT_MIN || luxAvg1m > LUX_ATT_MAX); // Condição crítica de luminosidade

    if (outside) { 
      outLuxCount++; 
      inLuxCount = 0; 
    } else { 
      inLuxCount++; 
      outLuxCount = 0; 
    }

    if (critical) {
      critLuxSec += 2; // Incrementa 2 segundos no contador de tempo crítico
    } else {
      critLuxSec = 0;  // Reseta o contador se a luz estiver dentro do intervalo normal
    }
    
    darkLightDetected = false; // Reseta flag de luz detectada durante a noite
    darkStartTime = 0;  // Reseta o tempo de início da luz detectada
    }
  }

  /*-------- Bloco 2: Análise de Umidade e Temperatura (cada 10s) --------*/
  if(currentMillis - lastDHTRead >= intervalDHT){
    lastDHTRead = currentMillis;

    TempAndHumidity data = dht.getTempAndHumidity();

    if(isnan(data.temperature) || isnan(data.humidity)){
      Serial.println("Failed to read from DHT11 sensor!");
    } else {
      Serial.print("Temp (inst): "); Serial.println(data.temperature);
      Serial.print("Hum  (inst): "); Serial.println(data.humidity);

      //Máximo e Mínimo
      if(data.temperature > tempMax)
        tempMax = data.temperature;

      if(data.temperature < tempMin)
        tempMin = data.temperature;

      // empilha para média de 1 min
      pushCirc(tempBuf, DHT_WIN, tempHead, tempCount, data.temperature);
      pushCirc(humBuf,  DHT_WIN, humHead,  humCount,  data.humidity);
      tempAvg1m = avgBuf(tempBuf, tempCount);
      rhAvg1m   = avgBuf(humBuf,  humCount);

      tempTotalSamples++;
      if (tempAvg1m < TEMP_OK_MIN || tempAvg1m > TEMP_OK_MAX) {
          tempOutSamples++;
      }

      rhTotalSamples++;
      if (rhAvg1m < RH_OK_MIN || rhAvg1m > RH_OK_MAX) {
          rhOutSamples++;
      }

      // classificação com base nas médias
      bool tempOK  = (tempAvg1m >= TEMP_OK_MIN && tempAvg1m <= TEMP_OK_MAX);
      bool tempATT = (!tempOK) && (tempAvg1m >= TEMP_ATT_MIN && tempAvg1m <= TEMP_ATT_MAX);
      bool tempCRI = (!tempOK) && !tempATT;

      bool rhOK  = (rhAvg1m >= RH_OK_MIN && rhAvg1m <= RH_OK_MAX);
      bool rhATT = (!rhOK) && (rhAvg1m >= RH_ATT_MIN && rhAvg1m <= RH_ATT_MAX);
      bool rhCRI = (!rhOK) && !rhATT;

      // Histerese: 3 leituras consecutivas -> atenção (DHT a cada 10s)
      if (!tempOK){ outTempCount++; inTempCount=0; } else { inTempCount++; outTempCount=0; }
      if (!rhOK)  { outRHCount++;   inRHCount=0;   } else { inRHCount++;   outRHCount=0;   }

      // Duração crítico: acumula segundos se crítico
      critTempSec = tempCRI ? (critTempSec + 10) : 0;
      critRHSec   = rhCRI   ? (critRHSec   + 10) : 0;

      if (!tempOK) {
        rangeOutTempTimes++;
        Serial.print("Temperature out of range. Counting times: ");
        Serial.println(rangeOutTempTimes);
      } else {
        rangeOutTempTimes = 0;
      }

      if (!rhOK) {
        rangeOutRHTimes++;
        Serial.print("Humidity out of range. Counting times: ");
        Serial.println(rangeOutRHTimes);
      } else {
        rangeOutRHTimes = 0;
      }

      if (rangeOutTempTimes >= 3) {
        Serial.println("Temperature out of range for 3 readings");
      }
      if (rangeOutRHTimes >= 3) {
        Serial.println("Humidity out of range for 3 readings");
      }
    }
  }

  /*-------- Bloco 3: Análise de Ruído (janela 1s, score -> média 1 min) --------*/
  unsigned long now = millis();

  static unsigned long lastCheck = 0;
  if (lastCheck == 0) lastCheck = now;
  unsigned long elapsed = now - lastCheck;
  lastCheck = now;

  if (digitalRead(soundPin) == HIGH) {
      highTime += elapsed;
  }

  if (now - windowStart >= window) { // janela em 1s
      int score = 0;
      if (window > 0) score = (highTime * 100) / window;  // 0–100%

      Serial.print("Score (1s): ");
      Serial.println(score);

      // atualizar mínimo e máximo
      if (score < noiseMin) noiseMin = score;
      if (score > noiseMax) noiseMax = score;

      if (score < noiseMin24h) noiseMin24h = score;
      if (score > noiseMax24h) noiseMax24h = score;

      // --- Detecção de pico ---
      if (score > peakThreshold) {
        peakDuration += window;   // acumula ms
      } else {
        peakDuration = 0;
      }

      if (peakDuration >= 3000) { // Se tiver pico por 3 segundos
        Serial.println("ALERTA: Pico longo detectado!");
      }

      // guarda score no buffer de 1 min
      pushCirc(noiseBuf, NOISE_WIN, noiseHead, noiseCount, score);
      noiseAvg1m = avgBuf(noiseBuf, noiseCount);

      noiseTotalSamples++;
      if (noiseAvg1m > NOISE_OK_MAX) {
          noiseOutSamples++;
      }

      // classificação média 1 min
      bool noiseOK  = (noiseAvg1m <= NOISE_OK_MAX);
      bool noiseATT = (!noiseOK && noiseAvg1m <= NOISE_ATT_MAX);
      bool noiseCRI = (!noiseOK && noiseAvg1m > NOISE_ATT_MAX);

      if (!noiseOK){ outNoiseCount++; inNoiseCount=0; } else { inNoiseCount++; outNoiseCount=0; }
      critNoiseSec = noiseCRI ? (critNoiseSec + 1) : 0; // 1s por iteração de janela

      if (noiseCount >= NOISE_WIN) { // 60 amostras (1 por segundo)
          noiseCount = 0; // reinicia janela
          noiseMin = 100;
          noiseMax = 0;
      }

      // Reinicia janela
      highTime = 0;
      windowStart = now;
  }

  /*-------- Decisão global (sincronizada com leituras, executada após atualizações) --------*/
  //Conta quantas métricas estão fora do ideal
  bool tempOut  = (outTempCount >= 1);
  bool rhOut    = (outRHCount >= 1);
  bool luxOut   = (outLuxCount >= 1);
  bool noiseOut = (outNoiseCount >= 1);
  uint8_t outs = (tempOut?1:0) + (rhOut?1:0) + (luxOut?1:0) + (noiseOut?1:0);

  //Se 2 ou mais métricas estiverem ruins ao mesmo tempo, acumula 1 segundo
  //Caso contrário (menos de 2 ruins), zera o contador
  if (outs >= 2) multiOutSec += 1;
  else multiOutSec = 0;

  // Variável crítica fora por > 5 min ou múltiplas fora da faixa
  bool alarmByCriticalTime = (critTempSec >= 300) || (critRHSec >= 300) || (critLuxSec >= 300) || (critNoiseSec >= 300);
  bool alarmByMultiple    = (multiOutSec >= 60);

  //Histerese: Disparar alerta apenas após 3 leituras consecutivas fora da faixa
  bool tempAttention  = (outTempCount >= 3);
  bool rhAttention    = (outRHCount >= 3);
  bool luxAttention   = (outLuxCount >= 3);
  bool noiseAttention = (outNoiseCount >= 3);

  //Determina estado
  SystemState newState = OK_STATE;
  if (alarmByCriticalTime || alarmByMultiple || darkAlarm) newState = ALARME_STATE;
  else if (tempAttention || rhAttention || luxAttention || noiseAttention) newState = ATENCAO_STATE;
  else newState = OK_STATE;

  if (newState != sysState) {
    sysState = newState;
    Serial.print("Estado alterado para: ");
    if (sysState == OK_STATE) Serial.println("OK");
    else if (sysState == ATENCAO_STATE) Serial.println("ATENCAO");
    else Serial.println("ALARME");
  }

  //Modos de Monitoramento
  switch(sysState){
    case OK_STATE:
      digitalWrite(greenPin, HIGH);
      digitalWrite(yellowPin, LOW);
      digitalWrite(redPin, LOW);
      digitalWrite(buzzerPin, LOW);
      break;

    case ATENCAO_STATE:
      digitalWrite(greenPin, LOW);
      digitalWrite(yellowPin, HIGH);
      digitalWrite(redPin, LOW);
      digitalWrite(buzzerPin, LOW);
      break;

    case ALARME_STATE:
      digitalWrite(greenPin, LOW);
      digitalWrite(yellowPin, LOW);
      digitalWrite(redPin, HIGH);
      activateBuzzer();
      break;
  }

  //Após 10 minutos, o alarme é silenciado
  if (buzzerMuted && (millis() - muteStartMillis >= MUTE_TIME)) { 
    buzzerMuted = false;
    Serial.println(">>> SILENCIAMENTO EXPIRADO");
  }

  //Após 24h (ou 8min para teste), gera relatório
  if (!finished24h && millis() - startMillis >= RUN_24H) {
    finished24h = true;
    Serial.println(">>> FIM DA COLETA");

    char csvName[32];
    char jsonName[32];

    sprintf(csvName,  "/relatorio_dia_%03d.csv",  dayCounter);
    sprintf(jsonName, "/relatorio_dia_%03d.json", dayCounter);

    createFile(jsonName);
    createCSV(csvName);
    printFile(csvName);

    dayCounter++;
    saveDayCounter();

    //Prepara próximo ciclo
    resetMetrics();           
    startMillis = millis();
    finished24h = false;
  }

  delay(5);
}

//========= FUNÇÕES DE DETECÇÃO DIA/NOITE ============

// DETECÇÃO RÁPIDA (10 minutos)
void updateDayNight10min(float luxAvg1m, unsigned long now) {
  static unsigned long darkStart  = 0;
  static unsigned long lightStart = 0;

  // Tendência a NOITE
  if (luxAvg1m < LUX_DN_NIGHT) {
    if (darkStart == 0) {
      darkStart = now;
      Serial.println(">>> [10min] Iniciando contagem para NOITE...");
    }
    lightStart = 0;

    if (now - darkStart >= DN_TIME && dnState != DN_NIGHT) {
      dnState = DN_NIGHT;
      Serial.println(">>> [10min] NOITE CONFIRMADA");
      printDayNightStatus();
    }
  }
  // Tendência a DIA
  else if (luxAvg1m > LUX_DN_DAY) {
    if (lightStart == 0) {
      lightStart = now;
      Serial.println(">>> [10min] Iniciando contagem para DIA...");
    }
    darkStart = 0;

    if (now - lightStart >= DN_TIME && dnState != DN_DAY) {
      dnState = DN_DAY;
      Serial.println(">>> [10min] DIA CONFIRMADO");
      printDayNightStatus();
    }
  }
  // Zona neutra (entre 40 e 120 lux)
  else {
    darkStart  = 0;
    lightStart = 0;
  }
}

// CICLO MACRO (6 horas) - Confirmação de longo prazo
void updateMacroCycle(float luxAvg1m, unsigned long now) {
  static unsigned long macroDarkStart  = 0;
  static unsigned long macroLightStart = 0;

  // Tendência a NOITE MACRO
  if (luxAvg1m < LUX_MACRO_NIGHT) {
    if (macroDarkStart == 0) {
      macroDarkStart = now;
      Serial.println(">>> [MACRO] Iniciando contagem para NOITE (6h)...");
    }
    macroLightStart = 0;

    // Se passou 6 horas no escuro
    if (now - macroDarkStart >= MACRO_CONFIRM_TIME && macroState != DN_NIGHT) {
      macroState = DN_NIGHT;
      Serial.println(">>> [MACRO] NOITE CONFIRMADA (6 horas < 40 lux)");
      printDayNightStatus();
    }
  }
  // Tendência a DIA MACRO
  else if (luxAvg1m > LUX_MACRO_DAY) {
    if (macroLightStart == 0) {
      macroLightStart = now;
      Serial.println(">>> [MACRO] Iniciando contagem para DIA (6h)...");
    }
    macroDarkStart = 0;

    // Se passou 6 horas na luz
    if (now - macroLightStart >= MACRO_CONFIRM_TIME && macroState != DN_DAY) {
      macroState = DN_DAY;
      Serial.println(">>> [MACRO] DIA CONFIRMADO (6 horas > 120 lux)");
      printDayNightStatus();
    }
  }
}

// Verifica se é noite (usa hierarquia: macro primeiro, depois 10min)
bool nightNow() {
  // Se o macro já tem uma decisão confirmada, usa ela (mais confiável)
  if (macroState == DN_NIGHT) return true;
  if (macroState == DN_DAY) return false;
  
  // Senão, usa a detecção rápida de 10min como fallback
  return (dnState == DN_NIGHT);
}

// Detecta luz indevida durante período noturno
bool detectNightLight(float luxValue, unsigned long now) {
  static bool lightDetected = false;
  static unsigned long startTime = 0;

  // Se não é noite, reseta e retorna
  if (!nightNow()) {
    lightDetected = false;
    return false;
  }

  // Durante a noite, verifica se há luz
  if (luxValue > 20.0) {
    if (!lightDetected) {
      lightDetected = true;
      startTime = now;
      Serial.println(">>> Luz detectada durante a noite...");
    }
    else if (now - startTime >= 120000UL) { // 2 minutos
      Serial.println(">>> ALARME: Luz por mais de 2 minutos na noite!");
      return true;
    }
  } else {
    lightDetected = false;
  }

  return false;
}

// Imprime status das detecções (útil para debug)
void printDayNightStatus() {
  Serial.print(">>> Estado 10min: ");
  if (dnState == DN_DAY) Serial.print("DIA");
  else if (dnState == DN_NIGHT) Serial.print("NOITE");
  else Serial.print("DESCONHECIDO");
  
  Serial.print(" | Estado Macro: ");
  if (macroState == DN_DAY) Serial.print("DIA");
  else if (macroState == DN_NIGHT) Serial.print("NOITE");
  else Serial.print("DESCONHECIDO");
  
  Serial.print(" | nightNow(): ");
  Serial.println(nightNow() ? "NOITE" : "DIA");
}

//========= Funções auxiliares existentes ============
float readLuxSensor(){
  return lightMeter.readLightLevel();
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
}

void readSoundSensor(){
  int v = digitalRead(soundPin);
  Serial.print("SoundPin: "); Serial.println(v);
}

void activateBuzzer(){
  if (buzzerMuted) {
    noTone(buzzerPin);
    return;
  }

  unsigned long currentMillis = millis();
  unsigned long elapsed = currentMillis - lastMillis;

  if (elapsed < buzzDuration) {
    tone(buzzerPin, 660, 1000);
  } else {
    noTone(buzzerPin);
  }

  if (elapsed >= intervalBuzzer) {
    lastMillis = currentMillis;
  }
}

void resetMetrics() { 
  luxMin = 32555.0; luxMax = 0.0;
  tempMin = 32555.0; tempMax = 0.0;
  rhMin = 100.0; rhMax = 0.0;
  noiseMin24h = 100.0; noiseMax24h = 0.0;

  luxCount = tempCount = humCount = noiseCount = 0;

  outTempCount = inTempCount = 0;
  outRHCount = inRHCount = 0;
  outLuxCount = inLuxCount = 0;
  outNoiseCount = inNoiseCount = 0;

  critTempSec = critRHSec = critLuxSec = critNoiseSec = 0;
  multiOutSec = 0;

  tempTotalSamples = tempOutSamples = 0;
  rhTotalSamples = rhOutSamples = 0;
  luxTotalSamples = luxOutSamples = 0;
  noiseTotalSamples = noiseOutSamples = 0;

  darkAlarm = false;
  
  // Reset dos estados dia/noite para novo ciclo
  dnState = DN_UNKNOWN;
  macroState = DN_UNKNOWN;
}

void saveDayCounter() { 
  File f = LittleFS.open("/dayCounter.txt", "w");
  if (!f) {
    Serial.println("Erro ao salvar dayCounter");
    return;
  }
  f.print(dayCounter);
  f.close();
}

void loadDayCounter() { 
  File f = LittleFS.open("/dayCounter.txt", "r");
  if (!f) {
    Serial.println("dayCounter nao existe, iniciando em 0");
    dayCounter = 0;
    return;
  }
  dayCounter = f.parseInt();
  f.close();
}

void createFile(const char *filename) {
  DynamicJsonDocument doc(1024);

  float tempOutPct  = (tempTotalSamples > 0) ? 
    (100.0 * tempOutSamples / tempTotalSamples) : 0;

  float rhOutPct    = (rhTotalSamples > 0) ? 
      (100.0 * rhOutSamples / rhTotalSamples) : 0;

  float luxOutPct   = (luxTotalSamples > 0) ? 
      (100.0 * luxOutSamples / luxTotalSamples) : 0;

  float noiseOutPct = (noiseTotalSamples > 0) ? 
      (100.0 * noiseOutSamples / noiseTotalSamples) : 0;

  doc["timestamp"] = millis();

  JsonObject gy30 = doc["sensors"].createNestedObject("gy30");
  gy30["min"] = luxMin;
  gy30["max"] = luxMax;
  gy30["avg"] = luxAvg1m;
  gy30["timeout"] = luxOutPct;

  JsonObject dht = doc["sensors"].createNestedObject("dht11");

  JsonObject temp = dht.createNestedObject("temperature");
  temp["min"] = tempMin;
  temp["max"] = tempMax;
  temp["avg"] = tempAvg1m;
  temp["timeout"] = tempOutPct;

  JsonObject hum = dht.createNestedObject("humidity");
  hum["min"] = rhMin;
  hum["max"] = rhMax;
  hum["avg"] = rhAvg1m;
  hum["timeout"] = rhOutPct;

  JsonObject ky037 = doc["sensors"].createNestedObject("ky037");
  ky037["min"] = noiseMin24h;
  ky037["max"] = noiseMax24h;
  ky037["avg"] = noiseAvg1m;
  ky037["peak"] = noisePeak;
  ky037["timeout"] = noiseOutPct;

  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  if (serializeJson(doc, file) == 0) {
    Serial.println("Failed to write JSON to file");
  } else {
    Serial.println("JSON file saved successfully");
  }

  file.close();
}

void createCSV(const char *filename) {
  float tempOutPct  = (tempTotalSamples > 0) ? 
    (100.0 * tempOutSamples / tempTotalSamples) : 0;

  float rhOutPct    = (rhTotalSamples > 0) ? 
    (100.0 * rhOutSamples / rhTotalSamples) : 0;

  float luxOutPct   = (luxTotalSamples > 0) ? 
    (100.0 * luxOutSamples / luxTotalSamples) : 0;

  float noiseOutPct = (noiseTotalSamples > 0) ? 
    (100.0 * noiseOutSamples / noiseTotalSamples) : 0;

  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.println("Failed to open CSV file for writing");
    return;
  }

  file.println("Sensor,Min,Max,Avg,PercentOut");

  file.print("Lux,");
  file.print(luxMin); file.print(",");
  file.print(luxMax); file.print(",");
  file.print(luxAvg1m); file.print(",");
  file.println(luxOutPct);

  file.print("Temp,");
  file.print(tempMin); file.print(",");
  file.print(tempMax); file.print(",");
  file.print(tempAvg1m); file.print(",");
  file.println(tempOutPct);

  file.print("UR,");
  file.print(rhMin); file.print(",");
  file.print(rhMax); file.print(",");
  file.print(rhAvg1m); file.print(",");
  file.println(rhOutPct);

  file.print("Noise,");
  file.print(noiseMin24h); file.print(",");
  file.print(noiseMax24h); file.print(",");
  file.print(noiseAvg1m); file.print(",");
  file.println(noiseOutPct);

  file.close();
  Serial.println("CSV file saved successfully");
}

void printFile(const char *filename) {
  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.println("Failed to open file");
    return;
  }

  Serial.println("----- START FILE -----");
  while (file.available()) {
    Serial.write(file.read());
  }
  Serial.println("\n----- END FILE -----");
  file.close();
}