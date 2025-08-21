/*
 * Monitamento Ambiental Tainã v1.1 (agendamento + botão; solo desativado)
 * Oficina Extensão Comunitária 2024 – 35 anos!
 *
 * Alterações:
 * - Desativado o sensor de umidade do solo (HD-38).
 * - Irrigação automática diária às 07:00 e 19:00 por 20 minutos.
 * - Botão manual: pressiona -> liga por até 20 min; pressiona de novo -> desliga.
 * - Leituras dos sensores a cada 30s (não bloqueante).
 */

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <U8g2lib.h>
#include <PMserial.h>
#include <WiFiEspAT.h>
#include <PubSubClient.h>
#include <SparkFunBME280.h>
#include "wiring_private.h"
#include "private_settings.h"

// --- Aliases de pinos se a variant não tiver Dx definidos ---
#ifndef D1
  #define D1 (1u)
#endif
#ifndef D2
  #define D2 (2u)
#endif
#ifndef D3
  #define D3 (3u)
#endif
#ifndef D7
  #define D7 (7u)
#endif
#ifndef D8
  #define D8 (8u)
#endif

// === PINAGEM ===
const int RELAY_PIN   = D1;   // Relé (válvula)
const int PMS_RST_PIN = D2;   // Reset do PMS7003
const int BUTTON_PIN  = D3;   // Botão manual (INPUT_PULLUP, ativo em LOW)

// Módulo ESP-12F em SERCOM2 (SAMD21)
#define PIN_SERIAL2_RX   (15ul)            // D6 (ajuste conforme sua placa)
#define PIN_SERIAL2_TX   (14ul)            // D5 (ajuste conforme sua placa)
#define PAD_SERIAL2_TX   (UART_TX_PAD_2)   // SERCOM2.2
#define PAD_SERIAL2_RX   (SERCOM_RX_PAD_3) // SERCOM2.3

// MQTT – PubSubClient usa keepalive em segundos
#define MQTT_KEEPALIVE   15
#define SENSOR1_TOPIC1   "/tc/sensor_1/pm2_5"
#define SENSOR1_TOPIC2   "/tc/sensor_1/pm10"
#define SENSOR2_TOPIC1   "/tc/sensor_2/umidade"
#define SENSOR2_TOPIC2   "/tc/sensor_2/temperatura"
#define SENSOR2_TOPIC3   "/tc/sensor_2/pressao"

// === SENSORES ===
// PMS7003 em Serial1
SerialPM pms(PMSx003, Serial1);

// BME280 (I2C)
BME280 bme280;
float humidity = 0, pressure = 0, temp = 0;
String temp_var;

// === RTC ===
RTC_PCF8523 rtc;
DateTime nowDT;                         // hora atual
DateTime relayStartTime;                // início do ciclo com relé ligado
const uint32_t maxRelayTime = 20UL * 60UL;  // 20 minutos (s)

// Controle de agenda (executa 1x por período)
int lastMorningRunYMD = -1;  // YYYYMMDD da última execução 07:00
int lastEveningRunYMD = -1;  // YYYYMMDD da última execução 19:00

// === ESTADO RELÉ ===
bool relayState = LOW; // inicia desligado

// === WiFi/MQTT ===
int status = WL_IDLE_STATUS;
WiFiClient ESPclient;
PubSubClient client(ESPclient);

// === OLED (I2C HW) ===
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// === Serial2 (ESP-12F) via SERCOM2 ===
Uart Serial2(&sercom2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

// === BOTÃO (debounce) ===
bool buttonStateStable = HIGH;      // estável (com PULLUP, HIGH = solto)
bool buttonReadingLast = HIGH;      // última leitura bruta
unsigned long lastDebounceMs = 0;
const unsigned long debounceDelayMs = 50;

// === TIMERS DE LEITURA ===
unsigned long lastEnvReadMs = 0;
const unsigned long ENV_PERIOD_MS = 30000; // 30s

// === Helpers ===
static inline int ymdFromDateTime(const DateTime& dt) {
  return dt.year() * 10000 + dt.month() * 100 + dt.day();
}

void setRelay(bool on) {
  relayState = on ? HIGH : LOW;
  digitalWrite(RELAY_PIN, relayState);
  if (on) relayStartTime = nowDT;
}

void startIrrigationCycle(const char* reason) {
  if (!relayState) {
    setRelay(true);
    Serial.print(F("[Rele] Ligando (20 min). Motivo: "));
    Serial.println(reason);
  }
}

void stopIrrigationCycle(const char* reason) {
  if (relayState) {
    setRelay(false);
    Serial.print(F("[Rele] Desligando. Motivo: "));
    Serial.println(reason);
  }
}

// Impressão com 2 dígitos (para relógio/tempo restante)
void print2u8(uint8_t v) {
  if (v < 10) display.print('0');
  display.print(v);
}
void print2ul(unsigned long v) {
  if (v < 10) display.print('0');
  display.print(v);
}

void drawHeaderClock() {
  display.setCursor(1, 10);
  display.print(" ");
  print2u8(nowDT.day());   display.print("/");
  print2u8(nowDT.month()); display.print("/");
  display.print(nowDT.year()); display.print(" ");
  print2u8(nowDT.hour());  display.print(":");
  print2u8(nowDT.minute());
}

void drawValveLine() {
  display.setCursor(2, 60);
  display.print("Valvula: ");
  if (relayState) {
    uint32_t elapsed = nowDT.unixtime() - relayStartTime.unixtime();
    uint32_t remain  = (elapsed >= maxRelayTime) ? 0 : (maxRelayTime - elapsed);
    unsigned long min = remain / 60, sec = remain % 60;
    display.print("ON ");
    display.print("(");
    print2ul(min);
    display.print(":");
    print2ul(sec);
    display.print(")");
  } else {
    display.print("OFF");
  }
}

void readAndPublishEnvironment() {
  // PMS7003
  Serial.println(F("[PMS7003] Leitura agendada..."));
  digitalWrite(PMS_RST_PIN, HIGH);
  pms.wake();
  pms.read();
  delay(3000); // estabiliza
  pms.read();

  // BME280
  humidity = bme280.readFloatHumidity();
  temp     = bme280.readTempC();
  pressure = (bme280.readFloatPressure() / 100.0F);

  // MQTT
  temp_var = String(humidity, 0);
  client.publish(SENSOR2_TOPIC1, temp_var.c_str());
  temp_var = String(temp, 2);
  client.publish(SENSOR2_TOPIC2, temp_var.c_str());
  temp_var = String(pressure, 2);
  client.publish(SENSOR2_TOPIC3, temp_var.c_str());

  if (pms.has_particulate_matter()) {
    temp_var = String(pms.pm25);
    client.publish(SENSOR1_TOPIC1, temp_var.c_str());
    temp_var = String(pms.pm10);
    client.publish(SENSOR1_TOPIC2, temp_var.c_str());
  }

  // OLED
  display.clearBuffer();
  display.setCursor(2, 20); display.print("Umidade (%): ");
  display.setCursor(86, 20); display.print(humidity, 0);

  display.setCursor(2, 30); display.print("Temp. (C): ");
  display.setCursor(86, 30); display.print(temp, 2);

  display.setCursor(2, 40); display.print("Press.(hPa): ");
  display.setCursor(86, 40); display.print(pressure, 2);

  if (pms.has_particulate_matter()) {
    display.setCursor(2, 50); display.print("PM2.5/PM10: ");
    display.setCursor(86, 50); 
    display.print(pms.pm25);
    display.print("/");
    display.print(pms.pm10);
  }

  drawHeaderClock();
  drawValveLine();
  display.sendBuffer();

  // Dorme PMS
  pms.sleep();
  digitalWrite(PMS_RST_PIN, LOW);
}

void handleButton() {
  // Debounce com INPUT_PULLUP (LOW = pressionado)
  int reading = digitalRead(BUTTON_PIN);
  if (reading != buttonReadingLast) {
    lastDebounceMs = millis();
    buttonReadingLast = reading;
  }
  if ((millis() - lastDebounceMs) > debounceDelayMs) {
    if (reading != buttonStateStable) {
      buttonStateStable = reading;
      if (buttonStateStable == LOW) { // borda de PRESS
        if (!relayState) {
          startIrrigationCycle("botao");
        } else {
          stopIrrigationCycle("botao");
        }
      }
    }
  }
}

void handleSchedule() {
  // Dispara 1x entre HH:00 e HH:01 (janela <2 min)
  int ymd = ymdFromDateTime(nowDT);

  // 07:00
  if (nowDT.hour() == 7 && nowDT.minute() < 2 && lastMorningRunYMD != ymd) {
    if (!relayState) startIrrigationCycle("agenda 07h");
    lastMorningRunYMD = ymd;
  }
  // 19:00
  if (nowDT.hour() == 19 && nowDT.minute() < 2 && lastEveningRunYMD != ymd) {
    if (!relayState) startIrrigationCycle("agenda 19h");
    lastEveningRunYMD = ymd;
  }
}

void enforceMaxTime() {
  if (relayState) {
    uint32_t elapsed = nowDT.unixtime() - relayStartTime.unixtime();
    if (elapsed >= maxRelayTime) {
      stopIrrigationCycle("tempo max");
    }
  }
}

// === REDE ===
void wifi_connect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print(F("[ESP-12F] Tentando conectar ao SSID: "));
    Serial.println(WLAN_SSID);
    status = WiFi.begin(WLAN_SSID, WLAN_PASS);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void mqtt_connect() {
  Serial.print(F("[MQTT] Conectando ao broker: "));
  Serial.println(MQTT_SERVER);
  if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWD)) {
    Serial.println(F("[MQTT] Conectado!"));
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    Serial.print(F("[MQTT] Falhou, state="));
    Serial.println(client.state());
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void mqtt_reconnect() {
  for (int i = 1; i <= 4 && !client.connected(); i++) {
    Serial.print(F("[MQTT] Tentando conexao... tentativa "));
    Serial.println(i);
    if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWD)) {
      Serial.println(F("[MQTT] Conectado!"));
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    } else {
      Serial.print(F("[MQTT] Falhou, state="));
      Serial.println(client.state());
      digitalWrite(LED_BUILTIN, LOW);
      delay(2000);
    }
  }
}

// === IRQ SERCOM2 p/ Serial2 ===
void SERCOM2_Handler() {
  Serial2.IrqHandler();
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { /* no-op */ }
  Serial.println(F("[Sistema] Iniciando sistema de monitoramento ambiental."));

  // GPIO
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false);

  pinMode(PMS_RST_PIN, OUTPUT);
  digitalWrite(PMS_RST_PIN, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // I2C + OLED
  Wire.begin();
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.setCursor(2,20); display.print("Monitor Ambiental v1.1");
  display.setCursor(2,30); display.print("Agenda: 07:00 & 19:00");
  display.setCursor(2,40); display.print("Botao: toggle 20min");
  display.sendBuffer();

  // BME280
  bme280.setI2CAddress(0x77);
  bme280.beginI2C();

  // RTC
  if (!rtc.begin()) {
    Serial.println(F("[RTC] Nao encontrado."));
    while (1) { delay(10); }
  }
  if (!rtc.initialized() || rtc.lostPower()) {
    Serial.println(F("[RTC] Ajustando data/hora do build."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  nowDT = rtc.now();

  // PMS
  pms.init();

  // ESP-12F em SERCOM2
  pinPeripheral(PIN_SERIAL2_RX, PIO_SERCOM);
  pinPeripheral(PIN_SERIAL2_TX, PIO_SERCOM);
  Serial2.begin(115200);
  WiFi.init(&Serial2);

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("[ESP-12F] Modulo WiFi nao encontrado."));
  } else {
    wifi_connect();
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setKeepAlive(MQTT_KEEPALIVE);
    mqtt_connect();
  }
}

void loop() {
  // Atualiza tempo
  nowDT = rtc.now();

  // Rede
  wifi_connect();
  if (!client.connected()) mqtt_reconnect();
  else client.loop();

  // Agendamento e limites
  handleSchedule();
  enforceMaxTime();

  // Botão manual (toggle)
  handleButton();

  // Leituras e display a cada 30s
  if (millis() - lastEnvReadMs >= ENV_PERIOD_MS) {
    lastEnvReadMs = millis();
    readAndPublishEnvironment();
  }

  delay(20);
}
