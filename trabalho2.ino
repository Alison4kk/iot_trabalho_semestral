/*
  Projeto: Célula 15 – Telemetria MQTT (ESP8266 TLS) com faixas de umidade, LEDs, CMD e CONFIG
  Alunos: Alison e Murilo
  Célula: 15
  Placa: NodeMCU/ESP-12E (ESP8266)

  Regras de LEDs (padrão do trabalho):
    - Umidade < 30%    -> LED VERMELHO (seco)
    - Umidade 30–70%   -> LED VERDE (adequado)
    - Umidade > 70%    -> LED AZUL (encharcado)
  Serial: imprime ADC, %, e status LED a cada ciclo.

  Tópicos:
    base = iot/<campus>/<curso>/<turma>/cell/<cellId>/device/<devId>/
    - state      (retained)    -> "online" no connect
    - telemetry  (QoS 1)       -> dados periódicos (~3s) e on-change
    - event      (QoS 1)       -> eventos (ex.: mudança de faixa de LED)
    - cmd        (sub)         -> recebe comandos (get_status, set_thresholds)
    - config     (retained)    -> publica configuração atual (limiares etc.)

  Dependências (Gerenciador de Bibliotecas):
    - PubSubClient
    - ArduinoJson
    - ESP8266WiFi
    - WiFiClientSecure
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h> // Necessário para TLS/porta 8883
#include <PubSubClient.h>     // Biblioteca MQTT
#include <ArduinoJson.h>

// ====================== CONFIG WI-FI / MQTT ====================== //
#include "secrets.h" // Inclui as constantes de configuração de rede e MQTT

String topicoBase;
String topicoEstado;      // retained
String topicoTelemetria;  // QoS 1
String topicoEvento;      // QoS 1
String topicoComando;     // sub
String topicoConfiguracao; // retained

// ====================== HARDWARE / LIMIARES ====================== //
// Pinos GPIOs
const uint8_t LED_VERMELHO = 5;  // GPIO5 (D1)
const uint8_t LED_VERDE    = 4;  // GPIO4 (D2)
const uint8_t LED_AZUL     = 12; // GPIO12 (D6)

// Leitura analógica
const int ADC_MIN = 0;
const int ADC_MAX = 1023;

// Limiares padrão do trabalho (ajustáveis via CMD)
struct Limiares {
  float normal  = 30.0f; // abaixo de normal -> seco/vermelho
  float encharcado = 70.0f; // acima de encharcado -> encharcado/azul
} limiares;

const float HISTERESE_PCT = 2.0f;  // histerese para evitar "pisca"

// Constantes para o status do LED (strings para consistência MQTT/Serial)
const char* STATUS_SECO = "seco";
const char* STATUS_ADEQUADO = "adequado";
const char* STATUS_ENCHARCADO = "encharcado";

// ====================== ESTADO / CLIENTES / TEMPO ====================== //
WiFiClientSecure clienteWifiSeguro;
PubSubClient clienteMqtt(clienteWifiSeguro); // PubSubClient usa o WiFiClientSecure

unsigned long INTERVALO_TELEMETRIA_MS = 3000UL; // Período de envio
unsigned long ultimoEnvioMs = 0;

// Estado atual do LED e o estado anterior (para eventos)
String statusLedAtual = "desconhecido";
String statusLedAnterior = "desconhecido";

// Variável para a nova regra de status de monitoramento (secagem prolongada)
unsigned long inicioStatusSecoMs = 0; // 0 se o status não for "seco"

// ====================== PROTÓTIPOS ====================== //
void conectarWifiSeNecessario();
void conectarMqttSeNecessario();
void callbackMqtt(char* topico, byte* payload, unsigned int length);

float lerUmidadePercentual(int* outAdc = nullptr);
String classificarStatusLed(float pct); // usa limiares.normal/encharcado + histerese
void atualizarLeds(const String& statusLed);

void publicarEstadoOnline();
void publicarConfiguracao();      // retained
void publicarTelemetria(bool forcarEnvio, float pct, int adc, const String& statusLed);
void publicarEventoMudancaStatus(const String& deStatus, const String& paraStatus, float pct, int adc);

void processarPayloadComando(const String& payload);
bool aplicarLimiaresDoJson(JsonObject data);

String obterStatusMonitoramento(); // Implementa a nova lógica (normal/atencao/critico)
String agoraEpochStr();

// ====================== SETUP ====================== //
void setup() {
  Serial.begin(9600);
  delay(100);

  // Configuração dos pinos dos LEDs
  pinMode(LED_VERMELHO, OUTPUT);
  pinMode(LED_VERDE,    OUTPUT);
  pinMode(LED_AZUL,     OUTPUT);
  digitalWrite(LED_VERMELHO, LOW);
  digitalWrite(LED_VERDE,    LOW);
  digitalWrite(LED_AZUL,     LOW);

  // Monta tópicos
  topicoBase        = String("iot/") + CAMPUS + "/" + CURSO + "/" + TURMA +
                      "/cell/" + String(CELL_ID) + "/device/" + DEV_ID + "/";
  topicoEstado      = topicoBase + "state";
  topicoTelemetria  = topicoBase + "telemetry";
  topicoEvento      = topicoBase + "event";
  topicoComando     = topicoBase + "cmd";
  topicoConfiguracao = topicoBase + "config";

  Serial.print(F("[TOPICO BASE] ")); Serial.println(topicoBase);

  // TLS sem verificação de cert (laboratório)
  clienteWifiSeguro.setInsecure(); 

  conectarWifiSeNecessario();

  // Configura servidor, porta e o callback de mensagens
  clienteMqtt.setServer(MQTT_HOST, MQTT_PORT);
  clienteMqtt.setCallback(callbackMqtt);

  conectarMqttSeNecessario();

  // Leitura e envio inicial
  int adc;
  float pct = lerUmidadePercentual(&adc);
  statusLedAtual = classificarStatusLed(pct);
  atualizarLeds(statusLedAtual);
  publicarTelemetria(true, pct, adc, statusLedAtual);

  Serial.println(F("[OK] Inicialização concluída."));
}

// ====================== LOOP PRINCIPAL ====================== //
void loop() {
  conectarWifiSeNecessario();
  
  // PubSubClient usa loop() para manter a conexão e processar mensagens recebidas
  if (!clienteMqtt.loop()) {
    conectarMqttSeNecessario();
  }

  // Leitura e classificação
  int adc;
  float pct = lerUmidadePercentual(&adc);
  String novoStatusLed = classificarStatusLed(pct);

  // Verifica mudança de status
  if (novoStatusLed != statusLedAtual) {
    statusLedAnterior = statusLedAtual;
    statusLedAtual    = novoStatusLed;
    
    // Atualiza o rastreamento do status "seco" para o monitoramento
    if (statusLedAtual == STATUS_SECO) {
      inicioStatusSecoMs = millis();
    } else {
      inicioStatusSecoMs = 0; // Saiu de seco, reseta o timer
    }

    atualizarLeds(statusLedAtual);
    publicarEventoMudancaStatus(statusLedAnterior, statusLedAtual, pct, adc);
    publicarTelemetria(true, pct, adc, statusLedAtual); // on-change
  } else if (statusLedAtual == STATUS_SECO && inicioStatusSecoMs == 0) {
    // Caso raro: se o status inicial já era seco no boot, o timer deve ser setado aqui.
    inicioStatusSecoMs = millis();
  }
  
  // Envio de telemetria periódica
  unsigned long agora = millis();
  if (agora - ultimoEnvioMs >= INTERVALO_TELEMETRIA_MS) {
    publicarTelemetria(false, pct, adc, statusLedAtual);
    ultimoEnvioMs = agora;
  }

  delay(50); // Pequeno delay para estabilidade do ESP8266
}

// ====================== CONEXÃO ====================== //

// Tenta conectar ao WiFi se não estiver conectado
void conectarWifiSeNecessario() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print(F("[WiFi] Conectando a "));
  Serial.print(WIFI_SSID);
  Serial.print(F(" ... "));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
  }
  Serial.println(F(" conectado!"));
  Serial.print(F("IP: ")); Serial.println(WiFi.localIP());
}

// Tenta conectar ao MQTT se não estiver conectado
void conectarMqttSeNecessario() {
  // Se já conectado, retorna. loop() garante a manutenção.
  if (clienteMqtt.connected()) return;

  Serial.print(F("[MQTT] Tentando conectar em "));
  Serial.print(MQTT_HOST); Serial.print(':'); Serial.print(MQTT_PORT);
  Serial.print(F(" ... "));

  // Tenta conectar (ID do Cliente, Usuário, Senha)
  if (clienteMqtt.connect(DEV_ID, MQTT_USER, MQTT_PASS)) {
    Serial.println(F("conectado!"));
    
    // Assina comandos (QoS 1 na assinatura)
    clienteMqtt.subscribe(topicoComando.c_str(), 1); 
    Serial.print(F("[MQTT] Subscrito em: ")); Serial.println(topicoComando);

    // Garante state=online (retained) e a CONFIG após reconexão
    publicarEstadoOnline();
    publicarConfiguracao();
  } else {
    Serial.print(F("falhou (rc="));
    Serial.print(clienteMqtt.state()); // Código de erro do PubSubClient
    Serial.println(F(")"));
    delay(1000); // Espera antes de tentar novamente
  }
}

// ====================== CALLBACK DE MENSAGENS ====================== //

// Função chamada quando uma mensagem MQTT é recebida em um tópico subscrito
void callbackMqtt(char* topico, byte* payload, unsigned int length) {
  Serial.print(F("[CMD] Mensagem recebida no tópico: ")); Serial.println(topico);

  if (String(topico) != topicoComando) {
    return; // Ignora se não for o tópico de comando
  }

  // Converte o array de bytes em String (com tamanho exato) para o ArduinoJson
  char payloadBuffer[length + 1];
  memcpy(payloadBuffer, payload, length);
  payloadBuffer[length] = '\0';

  String payloadStr = String(payloadBuffer);
  Serial.print(F("[CMD] Payload: ")); Serial.println(payloadStr);

  processarPayloadComando(payloadStr);
}

// ====================== LEITURA / CLASSIFICAÇÃO / LED ====================== //

// Função utilitária para limitar um float entre dois valores
float limitarFloat(float x, float a, float b) {
  return x < a ? a : (x > b ? b : x);
}

// Lê o ADC e retorna o valor em porcentagem
float lerUmidadePercentual(int* outAdc) {
  int raw = analogRead(A0); // 0..1023 no NodeMCU devkit
  if (outAdc) *outAdc = raw;
  raw = constrain(raw, ADC_MIN, ADC_MAX);
  float pct = 100.0f * (float)(raw - ADC_MIN) / (float)(ADC_MAX - ADC_MIN);
  return limitarFloat(pct, 0.0f, 100.0f);
}

// Classifica o percentual de umidade no status de LED (seco, adequado, encharcado)
String classificarStatusLed(float pct) {
  // Implementa histerese para evitar "pisca-pisca" nas bordas
  if (statusLedAtual == STATUS_SECO) {
    if (pct < limiares.normal + HISTERESE_PCT) return STATUS_SECO;
  }
  if (statusLedAtual == STATUS_ADEQUADO) {
    if (pct >= (limiares.normal - HISTERESE_PCT) && pct <= (limiares.encharcado + HISTERESE_PCT))
      return STATUS_ADEQUADO;
  }
  if (statusLedAtual == STATUS_ENCHARCADO) {
    if (pct > limiares.encharcado - HISTERESE_PCT) return STATUS_ENCHARCADO;
  }

  // Classificação direta (fallback ou primeira classificação)
  if (pct < limiares.normal)       return STATUS_SECO;
  if (pct <= limiares.encharcado)       return STATUS_ADEQUADO;
  /* pct > limiares.encharcado */        return STATUS_ENCHARCADO;
}

// Atualiza o estado dos pinos de LED com base no status
void atualizarLeds(const String& status) {
  uint8_t r = LOW, g = LOW, b = LOW;

  if (status == STATUS_SECO)      r = HIGH;
  else if (status == STATUS_ADEQUADO)  g = HIGH;
  else if (status == STATUS_ENCHARCADO) b = HIGH;

  digitalWrite(LED_VERMELHO, r);
  digitalWrite(LED_VERDE,    g);
  digitalWrite(LED_AZUL,     b);
}

// Implementa a lógica de status de monitoramento baseado no tempo seco
String obterStatusMonitoramento() {
  if (inicioStatusSecoMs == 0) {
    // Se não está em status "seco", é "normal"
    return "normal";
  }

  const unsigned long DURACAO_ATENCAO_MS = 5000UL;  // 5 segundos
  const unsigned long DURACAO_CRITICO_MS = 10000UL; // 10 segundos

  unsigned long tempoSecoMs = millis() - inicioStatusSecoMs;

  if (tempoSecoMs >= DURACAO_CRITICO_MS) {
    return "critico";
  }
  if (tempoSecoMs >= DURACAO_ATENCAO_MS) {
    return "atencao";
  }

  // Seco, mas por menos de 5 segundos
  return "normal"; 
}

// ====================== PUBLICAÇÕES MQTT ====================== //

// Publica "online" no topicState, retained=true
void publicarEstadoOnline() {
  clienteMqtt.publish(topicoEstado.c_str(), "online", /*retained=*/true);
}

// Publica a configuração atual dos limiares
void publicarConfiguracao() {
  if (!clienteMqtt.connected()) {
    Serial.println(F("[CONFIG] IGNORADO: Cliente MQTT não conectado."));
    return;
  }

  StaticJsonDocument<320> doc;
  doc["cellId"] = CELL_ID;
  doc["devId"]  = DEV_ID;

  // Objeto 'thresholds' (Payload JSON em inglês)
  JsonObject t = doc.createNestedObject("thresholds");
  t["normal"]  = limiares.normal;  // < normal -> seco
  t["encharcado"] = limiares.encharcado; // > encharcado -> encharcado

  JsonObject misc = doc.createNestedObject("misc");
  misc["telemetry_interval_ms"] = INTERVALO_TELEMETRIA_MS;

  char buf[360];
  size_t n = serializeJson(doc, buf, sizeof(buf));

  Serial.print(F("[CONFIG] Tamanho payload: ")); Serial.println(n);

  if (n == 0) {
    Serial.println(F("[CONFIG] ERRO: Falha na serialização JSON."));
    return;
  }

  bool publicado = clienteMqtt.publish(topicoConfiguracao.c_str(), (const uint8_t*)buf, n, /*retained=*/true);

  if (!publicado) {
    Serial.println(F("[CONFIG] ERRO: Falha ao publicar MQTT."));
  }
  delay(50);
}

// Publica os dados de telemetria
void publicarTelemetria(bool forcarEnvio, float pct, int adc, const String& statusLed) {
  Serial.println("[TELEMETRY] Iniciando publicação...");
  
  if (!clienteMqtt.connected()) {
    Serial.println(F("[TELEMETRY] IGNORADO: Cliente MQTT não conectado."));
    return;
  }

  StaticJsonDocument<512> doc; 
  
  // Campos de Identificação e Tempo (Payload JSON em inglês)
  doc["ts"] = agoraEpochStr();      // epoch (millis/1000)
  doc["cellId"] = CELL_ID;
  doc["devId"]  = DEV_ID;

  // Métricas: dados da leitura (Payload JSON em inglês)
  JsonObject metrics = doc.createNestedObject("metrics");
  metrics["pct"] = pct;
  metrics["raw"] = adc;

  // Status de monitoramento e limiares (Payload JSON em inglês)
  doc["status"]       = obterStatusMonitoramento(); // "normal|atencao|critico"
  doc["status_led"]   = statusLed;    // Estado do LED (seco|adequado|encharcado)

  JsonObject th = doc.createNestedObject("thresholds");
  th["normal"]  = limiares.normal;
  th["encharcado"] = limiares.encharcado;
  
  char buf[550]; 
  size_t n = serializeJson(doc, buf, sizeof(buf));
  
  Serial.print(F("[TELEMETRY] Tamanho payload: ")); Serial.println(n);

  if (n == 0) {
    Serial.println(F("[TELEMETRY] ERRO: Falha na serialização JSON."));
    return;
  }

  // QoS 1 (false=não retained)
  bool publicado = clienteMqtt.publish(topicoTelemetria.c_str(), (const uint8_t*)buf, n, /*retained=*/false);
  
  if (publicado) {
    Serial.println(F("[TELEMETRY] Publicado com SUCESSO."));
  } else {
    Serial.println(F("[TELEMETRY] ERRO: Falha ao publicar MQTT."));
  }
  delay(50);
}

// Publica evento de mudança de faixa de umidade/LED
void publicarEventoMudancaStatus(const String& deStatus, const String& paraStatus, float pct, int adc) {
  if (!clienteMqtt.connected()) {
    Serial.println(F("[EVENT] IGNORADO: Cliente MQTT não conectado."));
    return;
  }

  StaticJsonDocument<288> doc;
  doc["ts"] = agoraEpochStr();
  doc["type"] = "mudanca_led";
  doc["from"] = deStatus;
  doc["to"]   = paraStatus;
  doc["pct"] = pct; // Payload JSON em inglês
  doc["sensor_raw"] = adc;      // Payload JSON em inglês

  char buf[300];
  size_t n = serializeJson(doc, buf, sizeof(buf));

  Serial.print(F("[EVENT] Tamanho payload: ")); Serial.println(n);
  
  if (n == 0) {
    Serial.println(F("[EVENT] ERRO: Falha na serialização JSON."));
    return;
  }

  // QoS 1 (false=não retained)
  bool publicado = clienteMqtt.publish(topicoEvento.c_str(), (const uint8_t*)buf, n, /*retained=*/false);

  if (!publicado) {
    Serial.println(F("[EVENT] ERRO: Falha ao publicar MQTT."));
  }
  delay(50);
}

// ====================== CMD HANDLER ====================== //

// Processa o payload de comando recebido
void processarPayloadComando(const String& payload) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload);
  
  if (err) {
    Serial.print(F("[CMD] JSON inválido: ")); Serial.println(err.c_str());
    return;
  }

  const char* acao = doc["action"]; // Campo JSON mantido em inglês
  if (!acao) {
    Serial.println(F("[CMD] action ausente"));
    return;
  }

  String acaoStr = acao;

  Serial.print(F("[CMD] Comando acao: ")); Serial.println(acaoStr);

  if (acaoStr == "get_status") {
    Serial.println(F("[CMD] Executando get_status..."));
    int adc; float pct = lerUmidadePercentual(&adc);
    String led = classificarStatusLed(pct);
    publicarTelemetria(true, pct, adc, led);
    return;
  }

  if (acaoStr == "set_thresholds") {
    JsonObject dados = doc["data"]; // Campo JSON mantido em inglês
    if (dados.isNull()) {
      Serial.println(F("[CMD] set_thresholds sem data"));
      return;
    }
    if (aplicarLimiaresDoJson(dados)) {
      publicarConfiguracao(); // retained
      
      // Reclassifica e envia telemetria com os novos limiares
      int adc; float pct = lerUmidadePercentual(&adc);
      statusLedAtual = classificarStatusLed(pct);
      atualizarLeds(statusLedAtual);
      publicarTelemetria(true, pct, adc, statusLedAtual);

      Serial.println(F("[CMD] thresholds atualizados e telemetria enviada."));
    } else {
      Serial.println(F("[CMD] thresholds inválidos (use 0–100 com low < high)"));
    }
    return;
  }

  Serial.print(F("[CMD] ação não suportada: "));
  Serial.println(acaoStr);
}

// Aplica os novos limiares se forem válidos
bool aplicarLimiaresDoJson(JsonObject data) {
  // Aceita campos opcionais: "normal", "encharcado" (float 0–100)
  float novoBaixo  = data.containsKey("normal")  ? data["normal"].as<float>()  : limiares.normal;
  float novoAlto = data.containsKey("encharcado") ? data["encharcado"].as<float>() : limiares.encharcado;

  if (isnan(novoBaixo) || isnan(novoAlto)) return false;
  if (novoBaixo < 0.0f || novoBaixo > 100.0f) return false;
  if (novoAlto < 0.0f || novoAlto > 100.0f) return false;
  if (novoBaixo >= novoAlto) return false; // Limiar 'normal' deve ser sempre menor que 'encharcado'

  limiares.normal = novoBaixo;
  limiares.encharcado = novoAlto;
  return true;
}

// ====================== UTIL ====================== //

// Retorna o tempo decorrido desde o boot em segundos (epoch simulado)
String agoraEpochStr() {
  unsigned long s = millis() / 1000UL;
  return String(s);
}
