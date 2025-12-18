# environmental-sensing-system
Este projeto implementa um sistema de monitoramento ambiental inteligente usando ESP32, com análise contínua de:
- Temperatura
- Umidade relativa do ar
- Luminosidade
- Ruído ambiente

O sistema utiliza médias móveis, histerese, detecção de dia/noite, alarmes sonoros e visuais, além de geração automática de relatórios em CSV e JSON armazenados no LittleFS.

## Principais funcionalidades
- Monitoramento contínuo (24h ou modo teste)
- Detecção de condições OK / Atenção / Alarme
- Detecção de luz indevida durante a noite
- Detecção de picos prolongados de ruído
- Botão físico para silenciar alarme por 10 minutos
- Relatórios automáticos em CSV e JSON
- Persistência de dados usando LittleFS

## Hardware Utilizado
- ESP32
- Sensor de luminosidade GY-30 (BH1750)
- Sensor de temperatura e umidade DHT11
- Sensor de ruído KY-037
- 3 LEDs (Verde, Amarelo, Vermelho)
- Buzzer
- Botão de silenciamento

## Mapeamento de Pinos, Comunicação e Temporização
O sistema utiliza temporização baseada em millis(), comunicação por barramentos padrão e interrupções para garantir operação não bloqueante.

### Protocolos Utilizados

I2C:
- Sensor de luminosidade BH1750 (GY-30)
- Pinos: SDA (GPIO 21) e SCL (GPIO 22)

UART (Serial):
- Comunicação para debug e acompanhamento do sistema
- Baud rate: 115200

GPIO Digital / Analógico
- Leitura de sensores, botão
- Controle de LEDs e buzzer

### Temporização (Timers)

O sistema não utiliza delay() para controle lógico, de modo que todas as tarefas são controladas por timers baseados em millis(). O ciclo de monitoramento (24h ou modo teste) também é controlado por timer.

Cada sensor possui seu próprio intervalo de leitura:
- Luminosidade: 2 s
- Temperatura / Umidade: 10 s
- Ruído: janelas de 1 s

Essa abordagem permite:
- Execução paralela das tarefas
- Maior responsividade do sistema
- Funcionamento contínuo sem bloqueios

## Medição dos Sensores
### Temperatura (DHT11)
- Leitura: sensor digital DHT11
- Intervalo de leitura: 10 segundos
- Processamento: média móvel de 1 minuto (6 amostras)

Faixas:
- OK: 22 a 26 °C
- Atenção: 20 a 28 °C
- Crítico: < 20 °C ou > 28 °C

### Umidade Relativa (DHT11)
- Leitura: sensor digital DHT11
- Intervalo de leitura: 10 segundos
- Processamento: média móvel de 1 minuto (6 amostras)

Faixas:
- OK: 40 a 60 %
- Atenção: 35 a 65 %
- Crítico: < 35 % ou > 65 %

### Luminosidade (BH1750 / GY-30)
- Leitura: sensor I2C BH1750
- Intervalo de leitura: 2 segundos
- Processamento: média móvel de 1 minuto (30 amostras)

Faixas (período diurno):
- OK: 150 a 300 lux
- Atenção: 100 a 500 lux
- Crítico: < 100 lux ou > 500 lux

Período noturno:
- Luz > 20 lux por mais de 2 minutos → Alarme

### Ruído (KY-037)
- Leitura: saída digital (DO)
- Intervalo de amostragem: janela de 1 segundo
- Processamento: cálculo do percentual de tempo em nível HIGH
- Média: média móvel de 1 minuto (60 janelas)

Faixas (score percentual):
- OK: ≤ 40 %
- Atenção: 41 a 60 %

## Decisão Global de Estados

O estado do sistema é definido a partir da análise conjunta de temperatura, umidade, luminosidade e ruído, considerando médias móveis, histerese e tempo acumulado em condição crítica.

### Critérios Avaliados
Para cada variável, o sistema monitora:
- Se está dentro da faixa ideal (OK)
- Se está fora da faixa ideal por leituras consecutivas
- Se permanece em condição crítica por tempo prolongado

Além disso, o sistema avalia múltiplas variáveis fora da faixa ao mesmo tempo.

### Condição de Atenção (ATENCAO)
O sistema entra em ATENÇÃO quando qualquer sensor apresenta 3 leituras consecutivas fora da faixa ideal, considerando as médias de 1 minuto.

Nesse estado:
- LED amarelo ligado
- Buzzer desligado

### Condição de Alarme (ALARME)
O sistema entra em ALARME quando ocorre qualquer uma das situações abaixo:
- Uma variável permanece em condição crítica por mais de 5 minutos
- Duas ou mais variáveis ficam fora da faixa ideal simultaneamente por mais de 1 minuto
- Detecção de luz indevida durante o período noturno por mais de 2 minutos

Nesse estado:
- LED vermelho ligado
- Buzzer ativado (com possibilidade de silenciamento temporário)

### Condição Normal (OK)
O sistema permanece em OK quando:
- Todas as variáveis estão dentro da faixa ideal
- Não há condição crítica ativa
- Não há múltiplas variáveis fora do ideal simultaneamente

Nesse estado:
- LED verde ligado
- Buzzer desligado

## Detecção de Dia e Noite
Adotamos que o sistema embarcado não teria acesso ao WiFi, de modo que não seria possível obter o tempo real de execução. Deste modo, a aplicação contém:
- Detecção rápida (10 minutos)
- Confirmação macro (6 horas)

Essas funções avaliam a tendência a partir da luminosidade após um tempo curto e prolongado.

## Geração de Relatórios
O sistema gera automaticamente relatórios ao final de cada ciclo de monitoramento (24 horas ou 1 minuto no modo de teste).

Formatos Gerados:
- CSV – visualização rápida
- JSON – integração com sistemas

O formato JSON é mantido visando integração futura com um backend, facilitando o consumo, armazenamento e processamento dos dados por aplicações externas.
Os valores são salvos consecutivamentes e corresponde ao número do dia, armazenado de forma persistente no arquivo dayCounter.txt.

Formato do relatório:
- /relatorio_dia_XXX.csv
- /relatorio_dia_XXX.json

### Conteúdo dos Relatórios
Para cada sensor, são registrados:
- Valor mínimo
- Valor máximo
- Valor médio
- Percentual de tempo fora da faixa ideal

Os arquivos são armazenados localmente no LittleFS e podem ser lidos via Serial Monitor ou extraídos do sistema de arquivos do ESP32.

[Vídeo da aplicação] (https://drive.google.com/file/d/1VnFFJ3nH4gL6ddSOGJdfZgjzzgNC6EAj/view?usp=sharing)
