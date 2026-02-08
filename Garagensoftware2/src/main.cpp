#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <credentials.h>
#include "MCP23017Controller.h"
#include "Relay.h"
#include <ArduinoJson.h>
#include "mqtt_helper.h"
#include "bitmaps.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 displayLeft(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 displayRight(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t displayRightMode = 0;
unsigned long displayRightNextUpdate = 0;
uint8_t displaySelectedValue = 0; // value => 30 minutes

// Angezeigte Werte
char timePrev[16] = "";
char timeNow[16] = "";
char tempPrev[16] = "";
char tempNow[16] = "";
int netPowerPrev = 0;
int netPowerNow = 0;
int batteryPowerPrev = 0;
int batteryPowerNow = 0;
float batterySocPrev = 0;
float batterySocNow = 0;
bool lastPowerNet = true;

class Timer
{
  unsigned long startTime = 0;

public:
  void start(unsigned long now) { startTime = now; }
  unsigned long left(unsigned long now, unsigned long durationMs)
  {
    if (startTime >= now)
      startTime = now - 1;

    unsigned long yet = now - startTime;
    if (yet >= durationMs)
      return 0;
    return durationMs - yet;
  }
  bool elapsed(unsigned long now, unsigned long durationMs)
  {
    return left(now, durationMs) == 0;
  }
};

// Gate states
enum GateModus
{
  GATE_CLOSED,
  GATE_OPENED,
  GATE_RUNNING,
  GATE_STOPPED
};
GateModus gateModus = GATE_STOPPED;

// ----------------------------------------------------------
// MCP23017
// ----------------------------------------------------------
#define MCP_BACKEND_ADDR 0x20
MCP23017Controller mcp_backend;

#define MCP_HANDHELD_ADDR 0x24
MCP23017Controller mcp_handheld;

#define DOOR_OPEN HIGH
#define DOOR_CLOSED LOW

// Schalter Backend
#define SW_24VOLT 0
#define SW_RADIO0 1
#define SW_RADIO1 2
#define SW_RADIO2 3
#define SW_MOTION 4
#define SW_LID_END_UP 5
#define SW_LID_END_DOWN 6
#define SW_DOOR_BACK 7

// Relais Backend
#define RELAY_0_HEATING 0
#define RELAY_1_CONTACTOR_3_PHASE 1
#define RELAY_2_CONTACTOR_1_PHASE 2
#define RELAY_3_LID_MOTOR 3
#define RELAY_4_UNUSED 4
#define RELAY_5_LIGHT_MIDDLE 5
#define RELAY_6_LIGHT_FRONT 6
#define RELAY_7_LIGHT_BACK 7

Relay relays[8] = {
    Relay(mcp_backend, RELAY_0_HEATING, (char *)"Heizung", 100, 5),
    Relay(mcp_backend, RELAY_1_CONTACTOR_3_PHASE, (char *)"Dreiphasen-Kontaktor", 0, 0),
    Relay(mcp_backend, RELAY_2_CONTACTOR_1_PHASE, (char *)"Einphasen-Kontaktor", 0, 0),
    Relay(mcp_backend, RELAY_3_LID_MOTOR, (char *)"Tor-Motor", 20, 5),
    Relay(mcp_backend, RELAY_4_UNUSED, (char *)"Unbenutzt", 0, 1),
    Relay(mcp_backend, RELAY_5_LIGHT_MIDDLE, (char *)"Licht Mitte", 8, 3),
    Relay(mcp_backend, RELAY_6_LIGHT_FRONT, (char *)"Licht vorne", 8, 5),
    Relay(mcp_backend, RELAY_7_LIGHT_BACK, (char *)"Licht hinten", 8, 5),
};

// Handheld Schalter

#define SW_BTN_LIGHT 0
#define SW_BTN_GATE 1
#define SW_BTN_LID 2
#define SW_BTN_UP 3
#define SW_BTN_RIGHT 4
#define SW_BTN_DOWN 5
#define SW_BTN_LEFT 6
#define SW_BTN_OKAY 7
#define LED_CHARGE_LIGHT 8 + 0
#define LED_CHARGE_HEAVY 8 + 1
#define LED_BTNS 8 + 2
#define LED_GATE 8 + 3
#define LED_LID 8 + 4
#define SW_UNUSED_1 8 + 5
#define SW_DOOR_FRONT 8 + 6
#define SW_UNUSED_2 8 + 7

// Logging

#define LOG_LINES (48 * 5)
#define LOG_TEXT_LEN 22

char logBuffer[LOG_LINES][LOG_TEXT_LEN];
uint16_t logHead = 0;  // zeigt auf NÄCHSTE freie Stelle
uint16_t logCount = 0; // wie viele Logs existieren

void addLog(const char *text, bool mqtt = true)
{

  if (mqtt)
    mqttDebug(text);

  snprintf(logBuffer[logHead], LOG_TEXT_LEN, "%s %s", timeNow, text);

  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES)
    logCount++;

  displayRightNextUpdate = 0;
}

//  ----------------------------------------------------------
// Automatische Werte für Auto-Laden
enum class ChargeState
{
  IDLE,              // Laden aus
  PRECONDITION,      // Bedingungen prüfen (2 kW, Akku > 30 %)
  CHARGING_MIN_TIME, // Mindest-Ladezeit läuft
  CHARGING_ACTIVE    // Mindestzeit vorbei, aber Laden läuft weiter
};

struct ChargeConfig
{
  uint8_t relayPin;
  float socThreshold;   // in %
  float powerThreshold; // W (+ = lädt, - = entlädt)
  unsigned long minimumChargeTime = 0;
  ChargeState chargeState = ChargeState::IDLE;
  Timer conditionTimer;
  Timer minChargeTimer;
};
ChargeConfig chargeConfigs[] = {
    {RELAY_2_CONTACTOR_1_PHASE, 30.0, -2000.0}, // Einphasen-Kontaktor
    {RELAY_1_CONTACTOR_3_PHASE, 50.0, -4000.0}  // Dreiphasen-Kontaktor
};
//  ----------------------------------------------------------

// Heizungsparameter
#define HEATING_MIN_VALUE 3
// Minimale Batteriezellspannung in Volt
#define BATTERY_MIN_VALUE 2.79

// ----------------------------------------------------------
// Motor + BTS7960
// ----------------------------------------------------------
#define MOTOR_PWM_UP D5
#define MOTOR_PWM_DOWN D6
#define MOTOR_CURRENT A0

// Motor-Parameter
#define MOTOR_POWER 50 // Watt
#define MOTOR_MAX_PWM 255
#define MOTOR_SOFTSTART_TIME 1500 // ms
#define MOTOR_SOFTSTOP_TIME 100   // ms
#define MOTOR_MAX_CURRENT 470     // ADC value from A0

// Neue globale Variablen für ADC-Sampling
const unsigned long ANALOG_SAMPLE_INTERVAL_MS = 10; // minimaler Abstand zwischen Reads
const int ANALOG_BUFFER_SIZE = 5;                   // Anzahl Werte für Mittelwert
unsigned long _lastAnalogSample = 0;
int _analogSamples[ANALOG_BUFFER_SIZE] = {0};
int _analogSampleIndex = 0;
int _analogSampleCount = 0;

int getAverageMotorCurrent(unsigned long now)
{
  // nur alle ANALOG_SAMPLE_INTERVAL_MS lesen
  if (_lastAnalogSample == 0 || (now - _lastAnalogSample) >= ANALOG_SAMPLE_INTERVAL_MS)
  {
    _lastAnalogSample = now;
    int v = analogRead(MOTOR_CURRENT);
    _analogSamples[_analogSampleIndex] = v;
    _analogSampleIndex = (_analogSampleIndex + 1) % ANALOG_BUFFER_SIZE;
    if (_analogSampleCount < ANALOG_BUFFER_SIZE)
      _analogSampleCount++;
  }

  if (_analogSampleCount == 0)
    return 0;

  long sum = 0;
  for (int i = 0; i < _analogSampleCount; i++)
    sum += _analogSamples[i];

  return (int)(sum / _analogSampleCount);
}

void resetAveraegeMotorCurrent()
{
  _analogSampleIndex = 0;
  _analogSampleCount = 0;
  _lastAnalogSample = 0;
}

// ----------------------------------------------------------
// WLAN & MQTT
// ----------------------------------------------------------
#define WLAN_RECONNECT_TIME 10000           // 10 Sekunden
#define MQTT_RECONNECT_TIME 5000            // 5 Sekunden
#define MQTT_STATUS_INTERVAL 10 * 60 * 1000 // 10 Minuten
WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastWifiReconnect = 0;
unsigned long lastMqttReconnect = 0;
unsigned long lastMqttStatusUpdate = 0;

// ----------------------------------------------------------
// Tor-Zustände
// ----------------------------------------------------------
enum LidState
{
  OPENING_POWER_ON,
  OPENING_SOFTSTART,
  OPENING_RUNNING,
  OPENING_SOFTSTOP,

  CLOSING_POWER_ON,
  CLOSING_SOFTSTART,
  CLOSING_RUNNING,
  CLOSING_SOFTSTOP,

  STOPPING,
  IDLE
};

enum LidDirection
{
  UP,
  DOWN,
};

enum LidRealState
{
  LID_REAL_OPEN,
  LID_REAL_OPENING,
  LID_REAL_CLOSED,
  LID_REAL_CLOSING,
  LID_REAL_STOPPED,
};

LidState state = IDLE;
LidDirection lastDirection = UP;
LidRealState lidRealState = LID_REAL_STOPPED;
unsigned long stateStart = 0;
unsigned long lastClickLid = 0;
#define LidClickDebounceTime 2000
// #define MOTOR_RUN_MAX_TIME 30000 // 30 Sekunden
#define MOTOR_RUN_MAX_TIME 300000 // 300 Sekunden

void triggerLight()
{
  unsigned long now = millis();
  relays[RELAY_6_LIGHT_FRONT].triggerTimeout(now);
  if (relays[RELAY_5_LIGHT_MIDDLE].read())
    relays[RELAY_5_LIGHT_MIDDLE].triggerTimeout(now);
  if (relays[RELAY_7_LIGHT_BACK].read())
    relays[RELAY_7_LIGHT_BACK].triggerTimeout(now);
}

void clickLid()
{
  if (lastClickLid + LidClickDebounceTime > millis())
  {
    mqttDebug("click lid ignored due to debounce");
    return;
  }

  addLog("Lid toggle");
  triggerLight();

  lastClickLid = millis();
  if (state == STOPPING || state == IDLE)
  {
    // Lid is open, so close it
    if (mcp_backend.digitalRead(SW_LID_END_UP) == LOW)
      state = CLOSING_POWER_ON;
    // Lid is closed, so open it
    else if (mcp_backend.digitalRead(SW_LID_END_DOWN) == LOW)
      state = OPENING_POWER_ON;
    // Letzte Richtung war auf, also schließen
    else if (lastDirection == UP)
      state = CLOSING_POWER_ON;
    // Letzte Richtung war zu, also öffnen
    else if (lastDirection == DOWN)
      state = OPENING_POWER_ON;
  }
  else
  {
    // If lid is moving, stop it
    state = STOPPING;
  }
}

// ----------------------------------------------------------
// MQTT
// ----------------------------------------------------------

void mqttCallback(char *topic, byte *payload, unsigned int length)
{

  char buffer[56];

  if (strcmp(topic, "adebar/carport/gate/state") == 0)
  {
    if (!strncmp((char *)payload, "closed", length) || !strncmp((char *)payload, "unknown", length))
    {
      gateModus = GATE_CLOSED;
      mcp_handheld.setOutputModus(LED_GATE, off);
    }
    else if (!strncmp((char *)payload, "open", length))
    {
      gateModus = GATE_OPENED;
      mcp_handheld.setOutputModus(LED_GATE, on);
    }
    else if (!strncmp((char *)payload, "stopped", length))
    {
      gateModus = GATE_STOPPED;
      mcp_handheld.setOutputModus(LED_GATE, off);
    }
    else if (!strncmp((char *)payload, "opening", length) || !strncmp((char *)payload, "closing", length))
    {
      gateModus = GATE_RUNNING;
      mcp_handheld.setOutputModus(LED_GATE, fastBlink);
    }
  }

  if (!strcmp(topic, "adebar/garage/cover/set"))
  {
    if (!strncmp((char *)payload, "STOP", length))
      state = STOPPING;
    else if (!strncmp((char *)payload, "CLOSE", length))
    {
      if (mcp_backend.digitalRead(SW_LID_END_DOWN) == HIGH)
      {
        // Noch nicht zu
        state = CLOSING_POWER_ON;
        triggerLight();
      }
    }
    else if (!strncmp((char *)payload, "OPEN", length))
    {
      if (mcp_backend.digitalRead(SW_LID_END_UP) == HIGH)
      {
        // Noch nicht auf
        state = OPENING_POWER_ON;
        triggerLight();
      }
    }
    else if (!strncmp((char *)payload, "TOGGLE", length))
      clickLid();
    return;
  }

  if (!strcmp(topic, "adebar/garage/system/set"))
  {
    if (!strncmp((char *)payload, "RESTART", length))
    {
      ESP.restart();
    }
    return;
  }

  if (!strcmp(topic, "deye12k/cell_temp_lowest/state"))
  {
    char *endptr;
    float value = strtof((char *)payload, &endptr);

    if (value < HEATING_MIN_VALUE)
    {
      addLog("Heating ON", false);
      relays[RELAY_0_HEATING].triggerTimeout(millis());
    }

    return;
  }

  if (!strcmp(topic, "deye12k/cell_voltage_lowest/state"))
  {
    char *endptr;
    float value = strtof((char *)payload, &endptr);

    if (value < BATTERY_MIN_VALUE)
    {
      addLog("Loading ON", false);
      relays[RELAY_3_LID_MOTOR].triggerTimeout(millis());
    }

    return;
  }

  if (strcmp(topic, "adebar/time") == 0)
  {
    strncpy(timeNow, (char *)payload, length);
    timeNow[length] = '\0';

    return;
  }

  if (strcmp(topic, "outdoor/outdoor_temp/state") == 0)
  {
    strncpy(tempNow, (char *)payload, length);
    tempNow[length] = '\'';
    tempNow[length + 1] = 'C';
    tempNow[length + 2] = '\0';

    return;
  }

  if (strcmp(topic, "energymeter/power_curr/state") == 0)
  {
    strncpy(buffer, (char *)payload, length);
    buffer[length] = '\0';
    char *endptr;
    long value = strtol(buffer, &endptr, 10);

    if (*endptr != '\0')
      return;

    netPowerNow = (int)value;
    return;
  }

  if (strcmp(topic, "deye12k/battery_power/state") == 0)
  {
    strncpy(buffer, (char *)payload, length);
    buffer[length] = '\0';
    char *endptr;
    long value = strtol(buffer, &endptr, 10);

    if (*endptr != '\0')
      return;

    batteryPowerNow = (int)value;
    return;
  }

  if (strcmp(topic, "deye12k/battery_soc/state") == 0)
  {
    strncpy(buffer, (char *)payload, length);
    buffer[length] = '\0';
    char *endptr;
    float value = strtof(buffer, &endptr);

    if (*endptr != '\0')
      return;

    batterySocNow = value;
    return;
  }

  for (uint8_t relay = 0; relay < 8; relay++)
  {
    sprintf(buffer, "adebar/garage/relay%d/set", relay);
    if (!strcmp(topic, buffer))
    {
      if (!strncmp((char *)payload, "ON", length))
      {
        relays[relay].triggerOrSet(true, millis());

        for (uint8_t config = 0; config < 2; config++)
        {
          if (relay == chargeConfigs[config].relayPin)
          {
            chargeConfigs[config].chargeState = ChargeState::CHARGING_MIN_TIME;
            chargeConfigs[config].minimumChargeTime = millis() + 15UL * 60UL * 1000UL; // Mindestladezeit 15 Minuten
            chargeConfigs[config].minChargeTimer.start(millis());
          }
        }
      }
      else if (!strncmp((char *)payload, "OFF", length))
      {
        relays[relay].set(false, millis());

        for (uint8_t config = 0; config < 2; config++)
        {
          if (relay == chargeConfigs[config].relayPin)
            chargeConfigs[config].chargeState = ChargeState::IDLE;
        }
      }
      return;
    }
  }
}

bool connectWifiNonBlocking(unsigned long now)
{
  if (WiFi.status() == WL_CONNECTED)
    return true;

  if (now - lastWifiReconnect < WLAN_RECONNECT_TIME)
    return false;

  lastWifiReconnect = now;
  WiFi.begin(wifiSsid, wifiPassword);
  return false;
}

// MQTT senden
bool mqttPublish(const char *topic, const char *message)
{
  if (mqttClient.connected())
  {
    mqttClient.publish(topic, message);
    return true;
  }
  else
  {
    return false;
  }
}

// MQTT Debug senden
bool mqttDebug(const char *message)
{
  return mqttPublish("adebar/garage/system/debug", message);
}

void mqttSendAdebarGarageIpAddress(boolean full)
{
  if (full)
  {

    // see https://www.home-assistant.io/integrations/sensor.mqtt/
    const char *discoveryConfig = "{"
                                  "\"name\":\"Garage IP-Adresse\","
                                  "\"stat_t\":\"adebar/garage/ip_address/state\","
                                  "\"uniq_id\":\"adebar_garage_ip_address\","
                                  "\"dev\":{"
                                  "\"identifiers\":[\"adebar_garage\"],"
                                  "\"name\":\"Garage\""
                                  "}"
                                  "}";
    mqttPublish("homeassistant/sensor/adebar_garage_ip_address/config", discoveryConfig);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    String ip = WiFi.localIP().toString();
    char ip_char[ip.length() + 1];
    ip.toCharArray(ip_char, ip.length() + 1);

    mqttPublish("adebar/garage/ip_address/state", ip_char);
  }
}

void mqttSendAdebarGarageRestartButton(boolean full)
{
  if (!full)
    return;

  // see https://www.home-assistant.io/integrations/button.mqtt/
  const char *discoveryConfig = "{"
                                "\"name\":\"Garage Neustart\","
                                "\"uniq_id\":\"adebar_garage_system_restart\","
                                "\"cmd_t\":\"adebar/garage/system/set\","
                                "\"payload_press\":\"RESTART\","
                                "\"dev\":{"
                                "\"identifiers\":[\"adebar_garage\"],"
                                "\"name\":\"Garage\""
                                "}"
                                "}";
  mqttPublish("homeassistant/button/adebar_garage_system_restart/config", discoveryConfig);
}

void mqttSendAdebarGarageDoor(uint8_t door, boolean full)
{
  if (full)
  {
    // see https://www.home-assistant.io/integrations/binary_sensor.mqtt/
    JsonDocument discoveryConfig;
    discoveryConfig["name"] = door == 0 ? "Garage Vordertuer" : "Garage Hintertuer";
    discoveryConfig["dev_cla"] = "door";
    discoveryConfig["stat_t"] = door == 0 ? "adebar/garage/door_front/state" : "adebar/garage/door_back/state";
    discoveryConfig["uniq_id"] = door == 0 ? "adebar_garage_door_front" : "adebar_garage_door_back";

    JsonObject device = discoveryConfig["dev"].to<JsonObject>();
    device["identifiers"][0] = "adebar_garage";
    device["name"] = "Garage";

    char buffer[256];
    serializeJson(discoveryConfig, buffer);

    mqttPublish(door == 0 ? "homeassistant/binary_sensor/adebar_garage_door_front/config" : "homeassistant/binary_sensor/adebar_garage_door_back/config", buffer);
  }

  if (door == 0)
    mqttPublish("adebar/garage/door_front/state", mcp_handheld.digitalRead(SW_DOOR_FRONT) == DOOR_OPEN ? "ON" : "OFF");
  else
    mqttPublish("adebar/garage/door_back/state", mcp_backend.digitalRead(SW_DOOR_BACK) == DOOR_OPEN ? "ON" : "OFF");
}

void mqttSendAdebarGarageCover(boolean full)
{
  if (full)
  {
    // see https://www.home-assistant.io/integrations/cover.mqtt/
    JsonDocument discoveryConfig;
    discoveryConfig["name"] = "Garagentor";
    discoveryConfig["dev_cla"] = "garage";
    discoveryConfig["cmd_t"] = "adebar/garage/cover/set";
    discoveryConfig["stat_t"] = "adebar/garage/cover/state";
    discoveryConfig["uniq_id"] = "adebar_garage_cover";

    JsonObject device = discoveryConfig["dev"].to<JsonObject>();
    device["identifiers"][0] = "adebar_garage";
    device["name"] = "Garage";

    char buffer[256];
    serializeJson(discoveryConfig, buffer);
    mqttPublish("homeassistant/cover/adebar_garage_cover/config", buffer);
  }

  if (lidRealState == LID_REAL_CLOSED)
    mqttPublish("adebar/garage/cover/state", "closed");
  else if (lidRealState == LID_REAL_OPEN)
    mqttPublish("adebar/garage/cover/state", "open");
  else if (lidRealState == LID_REAL_OPENING)
    mqttPublish("adebar/garage/cover/state", "opening");
  else if (lidRealState == LID_REAL_CLOSING)
    mqttPublish("adebar/garage/cover/state", "closing");
  else if (lidRealState == LID_REAL_STOPPED)
    mqttPublish("adebar/garage/cover/state", "stopped");
}

void mqttSendStatus(boolean full)
{
  for (int i = 0; i < 8; i++)
    relays[i].mqttPublishState(full);

  mqttSendAdebarGarageIpAddress(full);
  mqttSendAdebarGarageDoor(0, full);
  mqttSendAdebarGarageDoor(1, full);
  mqttSendAdebarGarageCover(full);
  mqttSendAdebarGarageRestartButton(full);

  lastMqttStatusUpdate = millis();
}

bool connectMQTT(unsigned long now)
{
  if (mqttClient.connected())
    return true;

  if (now - lastMqttReconnect < MQTT_RECONNECT_TIME)
    return false;

  lastMqttReconnect = now;

  if (mqttClient.connect("garage2", mqttUser, mqttPassword))
  {
    addLog("MQTT connected", false);
    mqttClient.subscribe("adebar/time"); // Uhrzeit

    mqttClient.subscribe("deye12k/battery_power/state");       // Batteriewechselrichter Leistung
    mqttClient.subscribe("deye12k/battery_soc/state");         // Batteriewechselrichter SOC
    mqttClient.subscribe("deye12k/cell_temp_lowest/state");    // Batteriewechselrichter Temperatur
    mqttClient.subscribe("deye12k/cell_voltage_lowest/state"); // Batteriewechselrichter Zellspannung
    mqttClient.subscribe("outdoor/outdoor_temp/state");        // Temperatur

    mqttClient.subscribe("energymeter/power_curr/state"); // Zähler Leistung
    mqttClient.subscribe("adebar/garage/+/set");
    mqttClient.subscribe("adebar/carport/gate/state");
    mqttClient.publish("adebar/garage/system/state", "connected");
    mqttSendStatus(true);
    return true;
  }
  else
  {
    addLog("MQTT failed", false);
    Serial.print("Fehler beim MQTT-Verbindungsversuch: ");
    Serial.println(mqttClient.state());
    return false;
  }
}

float smootherstep(float t)
{
  return t * t * t * (t * (t * 6 - 15) + 10);
}
int getSoftstartPWM(unsigned long elapsed, unsigned long duration)
{
  if (elapsed >= duration)
    return MOTOR_MAX_PWM;

  float x = (float)elapsed / (float)duration; // 0.0 ... 1.0
  float s = smootherstep(x);                  // 0.0 ... 1.0
  return (int)(s * MOTOR_MAX_PWM);
}
int getSoftstopPWM(unsigned long elapsed, unsigned long duration)
{
  if (elapsed >= duration)
    return 0;

  float x = (float)elapsed / (float)duration; // 0..1
  float s = smootherstep(1.0 - x);
  return (int)(s * MOTOR_MAX_PWM);
}

// ----------------------------------------------------------
// Tor-Statemachine
// ----------------------------------------------------------
void handleLid(unsigned long now)
{
  if ((state != IDLE && state != STOPPING) && (now - stateStart > MOTOR_RUN_MAX_TIME) && now - stateStart > 0 && stateStart > 0)
  {
    addLog("Lid timeout");

    state = STOPPING;
    lidRealState = LID_REAL_STOPPED;
    return;
  }

  switch (state)
  {

  // -------------------------- OPENING -------------------------
  case OPENING_POWER_ON:
  {
    lastDirection = UP;
    mcp_handheld.setOutputModus(LED_LID, fastBlink);
    relays[RELAY_3_LID_MOTOR].set(true, now);
    if (mcp_backend.digitalRead(SW_24VOLT)) // 24 Volt liegen an
    {
      state = OPENING_SOFTSTART;
      lidRealState = LID_REAL_OPENING;
      stateStart = now;
    }
    return;
  }

  case OPENING_SOFTSTART:
  {
    lastDirection = UP;
    if (mcp_backend.digitalRead(SW_LID_END_UP) == LOW) // Ende erreicht
    {
      state = OPENING_SOFTSTOP;
      lidRealState = LID_REAL_OPENING;
      return;
    }

    unsigned long t = now - stateStart;
    if (t >= MOTOR_SOFTSTART_TIME)
    {
      state = OPENING_RUNNING;
      lidRealState = LID_REAL_OPENING;
      return;
    }

    int pwm = getSoftstartPWM(t, MOTOR_SOFTSTART_TIME);
    analogWrite(MOTOR_PWM_UP, pwm);
    analogWrite(MOTOR_PWM_DOWN, 0);
    return;
  }

  case OPENING_RUNNING:
  {
    lastDirection = UP;
    if (mcp_backend.digitalRead(SW_LID_END_UP) == LOW) // Ende erreicht
    {
      state = OPENING_SOFTSTOP;
      lidRealState = LID_REAL_OPENING;
      stateStart = now;
      return;
    }

    int current = getAverageMotorCurrent(now);
    if (current > MOTOR_MAX_CURRENT)
    {
      // Serial.print("Motor current to high: ");
      // Serial.print(current);
      // Serial.println("  . Stopping");
      char buffer[64];
      snprintf(buffer, 64, "Motor current too high: %d", current);
      addLog(buffer);

      state = STOPPING;
      lidRealState = LID_REAL_STOPPED;
    }
    analogWrite(MOTOR_PWM_UP, MOTOR_MAX_PWM);
    analogWrite(MOTOR_PWM_DOWN, 0);
    return;
  }

  case OPENING_SOFTSTOP:
  {
    lastDirection = UP;
    unsigned long t = now - stateStart;
    if (t >= MOTOR_SOFTSTOP_TIME)
    {
      state = STOPPING;
      lidRealState = LID_REAL_OPEN;
      return;
    }
    int pwm = getSoftstopPWM(t, MOTOR_SOFTSTOP_TIME);
    analogWrite(MOTOR_PWM_UP, pwm);
    analogWrite(MOTOR_PWM_DOWN, 0);
    return;
  }

  // -------------------------- CLOSING -------------------------
  case CLOSING_POWER_ON:
  {
    lastDirection = DOWN;
    mcp_handheld.setOutputModus(LED_LID, fastBlink);
    relays[RELAY_3_LID_MOTOR].set(true, now);
    if (mcp_backend.digitalRead(SW_24VOLT)) // 24 Volt liegen an
    {
      state = CLOSING_SOFTSTART;
      lidRealState = LID_REAL_CLOSING;
      stateStart = now;
    }
    return;
  }

  case CLOSING_SOFTSTART:
  {
    lastDirection = DOWN;
    if (mcp_backend.digitalRead(SW_LID_END_DOWN) == LOW) // Ende erreicht
    {
      state = CLOSING_SOFTSTOP;
      lidRealState = LID_REAL_CLOSING;
      stateStart = now;
      return;
    }

    unsigned long t = now - stateStart;
    if (t >= MOTOR_SOFTSTART_TIME)
    {
      state = CLOSING_RUNNING;
      lidRealState = LID_REAL_CLOSING;
      return;
    }
    int pwm = getSoftstartPWM(t, MOTOR_SOFTSTART_TIME);
    analogWrite(MOTOR_PWM_UP, 0);
    analogWrite(MOTOR_PWM_DOWN, pwm);
    return;
  }

  case CLOSING_RUNNING:
  {
    lastDirection = DOWN;
    if (mcp_backend.digitalRead(SW_LID_END_DOWN) == LOW) // Ende erreicht
    {
      state = CLOSING_SOFTSTOP;
      lidRealState = LID_REAL_CLOSING;
      stateStart = now;
      return;
    }

    int current = getAverageMotorCurrent(now);
    if (current > MOTOR_MAX_CURRENT)
    {
      // Serial.print("Motor current to high: ");
      // Serial.print(current);
      // Serial.println("  . Stopping");
      char buffer[64];
      snprintf(buffer, 64, "Motor current too high: %d", current);
      addLog(buffer);

      state = STOPPING;
      lidRealState = LID_REAL_STOPPED;
      return;
    }
    analogWrite(MOTOR_PWM_UP, 0);
    analogWrite(MOTOR_PWM_DOWN, MOTOR_MAX_PWM);
    return;
  }

  case CLOSING_SOFTSTOP:
  {
    lastDirection = DOWN;
    unsigned long t = now - stateStart;
    if (t >= MOTOR_SOFTSTOP_TIME)
    {
      state = STOPPING;
      lidRealState = LID_REAL_CLOSED;
      return;
    }

    int pwm = getSoftstopPWM(t, MOTOR_SOFTSTOP_TIME);
    analogWrite(MOTOR_PWM_UP, 0);
    analogWrite(MOTOR_PWM_DOWN, pwm);
    return;
  }

  // -------------------------- STOPPING -------------------------
  case STOPPING:
  {
    analogWrite(MOTOR_PWM_UP, 0);
    analogWrite(MOTOR_PWM_DOWN, 0);
    relays[RELAY_3_LID_MOTOR].set(false, now);

    state = IDLE;

    if (lidRealState != LID_REAL_OPEN && lidRealState != LID_REAL_CLOSED)
      lidRealState = LID_REAL_STOPPED;

    mqttSendStatus(false);
    resetAveraegeMotorCurrent();

    if (lidRealState == LID_REAL_CLOSED)
      mcp_handheld.setOutputModus(LED_LID, off);
    else if (lidRealState == LID_REAL_OPEN)
      mcp_handheld.setOutputModus(LED_LID, on);

    return;
  }

  // -------------------------- IDLE ------------------------------
  case IDLE:
    stateStart = 0;
    return;
  }
}

void clickGate()
{
  addLog("Gate toggle");
  if (gateModus == GATE_CLOSED || gateModus == GATE_STOPPED)
  {
    mqttPublish("adebar/carport/gate/set", "OPEN");
    mcp_handheld.setOutputModus(LED_GATE, fastBlink);
  }
  else if (gateModus == GATE_OPENED)
  {
    mqttPublish("adebar/carport/gate/set", "CLOSE");
    mcp_handheld.setOutputModus(LED_GATE, fastBlink);
  }
  else if (gateModus == GATE_RUNNING)
  {
    mqttPublish("adebar/carport/gate/set", "STOP");
    mcp_handheld.setOutputModus(LED_GATE, off);
  }
}

void clickBell()
{
  addLog("Bell btn pressed", false);
  mqttPublish("adebar/klingelbox/bell_button/set", "SET");
}

// ----------------------------------------------------------
// OTA
// ----------------------------------------------------------
void setupOTA()
{
  ArduinoOTA.setHostname("garage-controller");
  ArduinoOTA.begin();
}

#define CONDITION_TIME 5 * 60000 // 5 Minuten
#define TOLERANCE_FACTOR 10      // 10 * 50% (SOC) / 2 => 250 W Toleranz

void updateCharging(unsigned long now, uint8_t configIndex)
{
  char buffer[64];

  switch (chargeConfigs[configIndex].chargeState)
  {

  case ChargeState::IDLE:
    if ((batterySocNow > chargeConfigs[configIndex].socThreshold && batteryPowerNow < chargeConfigs[configIndex].powerThreshold) ||
        netPowerNow < chargeConfigs[configIndex].powerThreshold)
    {
      chargeConfigs[configIndex].conditionTimer.start(now);
      chargeConfigs[configIndex].chargeState = ChargeState::PRECONDITION;
      snprintf(buffer, sizeof(buffer), "Precondition started relay %d", chargeConfigs[configIndex].relayPin);
      addLog(buffer);
    }
    break;

  case ChargeState::PRECONDITION:
    if (batteryPowerNow >= chargeConfigs[configIndex].powerThreshold && netPowerNow >= chargeConfigs[configIndex].powerThreshold)
    {
      chargeConfigs[configIndex].chargeState = ChargeState::IDLE;
      break;
    }

    if (chargeConfigs[configIndex].conditionTimer.elapsed(now, CONDITION_TIME))
    {
      relays[chargeConfigs[configIndex].relayPin].set(true, now);
      chargeConfigs[configIndex].minimumChargeTime = CONDITION_TIME;
      chargeConfigs[configIndex].minChargeTimer.start(now);
      chargeConfigs[configIndex].chargeState = ChargeState::CHARGING_MIN_TIME;
      snprintf(buffer, sizeof(buffer), "Charging min time relay %d", chargeConfigs[configIndex].relayPin);
      addLog(buffer);
    }
    break;

  case ChargeState::CHARGING_MIN_TIME:
    if (chargeConfigs[configIndex].minChargeTimer.elapsed(now, chargeConfigs[configIndex].minimumChargeTime))
    {
      chargeConfigs[configIndex].chargeState = ChargeState::CHARGING_ACTIVE;
      snprintf(buffer, sizeof(buffer), "Charging active relay %d", chargeConfigs[configIndex].relayPin);
      addLog(buffer);
    }
    break;

  case ChargeState::CHARGING_ACTIVE:
    // Laden darf weiterlaufen solange *irgendwie* geladen wird
    int tolerance = TOLERANCE_FACTOR * batterySocNow / 2; // je voller der Akku, desto mehr Toleranz
    if ((batteryPowerNow > tolerance && netPowerNow > 0) || (batteryPowerNow > 0 && netPowerNow > tolerance))
    {
      relays[chargeConfigs[configIndex].relayPin].set(false, now);
      chargeConfigs[configIndex].chargeState = ChargeState::IDLE;

      snprintf(buffer, sizeof(buffer), "Charging stopped relay %d", chargeConfigs[configIndex].relayPin);
      addLog(buffer);
      return;
    }

    if (configIndex == 0) // Einphasen-Kontaktor
    {
      // Wenn der Einphasen-Kontaktor aus ist und der Dreiphasen-Kontaktor inaktiv ist, aktiviere ihn
      if (relays[chargeConfigs[configIndex].relayPin].read() == false && chargeConfigs[1].chargeState == ChargeState::IDLE)
      {
        snprintf(buffer, sizeof(buffer), "Switching to single-phase charging relay %d", chargeConfigs[configIndex].relayPin);
        addLog(buffer);
        relays[chargeConfigs[configIndex].relayPin].set(true, now);
      }
    }
    break;
  }
}

void updateDisplayLeft(unsigned long now)
{
  bool changed = false;
  displayLeft.setTextSize(2);
  displayLeft.setTextColor(SSD1306_WHITE);

  // ----- Uhr -----
  if (strcmp(timeNow, timePrev) != 0)
  {
    displayLeft.fillRect(0, 0, 128, 16, SSD1306_BLACK);
    displayLeft.drawBitmap(0, 0, icon_clock, 16, 16, SSD1306_WHITE);
    displayLeft.setCursor(20, 0);
    displayLeft.print(timeNow);
    strcpy(timePrev, timeNow);
    changed = true;
  }

  // // ----- Temperatur -----
  if (strcmp(tempNow, tempPrev) != 0)
  {
    displayLeft.fillRect(0, 16, 128, 16, SSD1306_BLACK);
    displayLeft.drawBitmap(0, 16, icon_temp, 16, 16, SSD1306_WHITE);
    displayLeft.setCursor(20, 16);
    displayLeft.print(tempNow);
    strcpy(tempPrev, tempNow);
    changed = true;
  }

  // ----- Batterie SOC -----
  if (batterySocNow != batterySocPrev)
  {
    displayLeft.fillRect(0, 32, 128, 16, SSD1306_BLACK);

    if (batterySocNow < 10)
      displayLeft.drawBitmap(0, 32, icon_batt_empty, 16, 16, SSD1306_WHITE);
    else if (batterySocNow < 30)
      displayLeft.drawBitmap(0, 32, icon_batt_two_quarter, 16, 16, SSD1306_WHITE);
    else if (batterySocNow < 70)
      displayLeft.drawBitmap(0, 32, icon_batt_middle, 16, 16, SSD1306_WHITE);
    else if (batterySocNow < 90)
      displayLeft.drawBitmap(0, 32, icon_batt_three_quarter, 16, 16, SSD1306_WHITE);
    else
      displayLeft.drawBitmap(0, 32, icon_batt_full, 16, 16, SSD1306_WHITE);

    displayLeft.setCursor(20, 32);
    displayLeft.print(batterySocNow);
    displayLeft.print("%");

    batterySocPrev = batterySocNow;
    changed = true;
  }

  // ----- Leistung -----
  if (now / 2000 % 2 == 0)
  {
    if (!lastPowerNet || netPowerNow != netPowerPrev)
    {
      displayLeft.fillRect(0, 48, 128, 16, SSD1306_BLACK);
      displayLeft.drawBitmap(0, 48, icon_power, 16, 16, SSD1306_WHITE);
      displayLeft.setCursor(20, 48);

      float kw;
      if (netPowerNow >= 1000 || netPowerNow <= -1000)
      {
        kw = netPowerNow / 1000.0;
        displayLeft.print(kw, 1);
        displayLeft.print(" kW");
      }
      else
      {
        displayLeft.print(netPowerNow);
        displayLeft.print(" W");
      }
      netPowerPrev = netPowerNow;
      changed = true;
      lastPowerNet = true;
    }
  }
  else
  {
    if (lastPowerNet || batteryPowerNow != batteryPowerPrev)
    {
      displayLeft.fillRect(0, 48, 128, 16, SSD1306_BLACK);
      displayLeft.drawBitmap(0, 48, icon_batt_middle, 16, 16, SSD1306_WHITE);
      displayLeft.setCursor(20, 48);

      float kw;
      if (batteryPowerNow >= 1000 || batteryPowerNow <= -1000)
      {
        kw = batteryPowerNow / 1000.0;
        displayLeft.print(kw, 1);
        displayLeft.print(" kW");
      }
      else
      {
        displayLeft.print(batteryPowerNow);
        displayLeft.print(" W");
      }
      batteryPowerPrev = batteryPowerNow;
      changed = true;
      lastPowerNet = false;
    }
  }

  if (changed)
    displayLeft.display();
}

void updateDisplayRight(unsigned long now)
{
  uint8_t hours;
  uint8_t minutes;
  if (now < displayRightNextUpdate)
    return;

  displayRight.clearDisplay();

  displayRight.setTextSize(1);
  displayRight.setTextColor(SSD1306_WHITE);
  displayRight.setCursor(0, 4);
  displayRight.print('<');
  displayRight.setCursor(122, 4);
  displayRight.print('>');
  displayRight.setTextSize(2);

  if (displayRightMode == 0 || displayRightMode == 1)
  {
    if (displayRightMode == 0)
    {
      displayRight.setCursor(30, 0);
      displayRight.print("Auto");
      displayRight.drawBitmap(90, 0, icon_power, 16, 16, SSD1306_WHITE);
    }
    else
    {
      displayRight.setCursor(10, 0);
      displayRight.print("Auto");
      displayRight.drawBitmap(80, 0, icon_power, 16, 16, SSD1306_WHITE);
      displayRight.drawBitmap(60, 0, icon_power, 16, 16, SSD1306_WHITE);
      displayRight.drawBitmap(100, 0, icon_power, 16, 16, SSD1306_WHITE);
    }
    displayRight.setCursor(0, 24);
    displayRight.setTextSize(1);
    displayRight.print("Ist: ");
    displayRight.setCursor(0, 44);
    displayRight.print("Soll: ");
    displayRight.setTextSize(2);

    displayRight.setCursor(35, 20);
    if (chargeConfigs[displayRightMode].chargeState == ChargeState::IDLE || chargeConfigs[displayRightMode].chargeState == ChargeState::PRECONDITION)
      displayRight.print("aus");
    else if (chargeConfigs[displayRightMode].chargeState == ChargeState::CHARGING_ACTIVE)
      displayRight.print("an");
    else
    {
      unsigned long left = chargeConfigs[displayRightMode].minChargeTimer.left(now, chargeConfigs[displayRightMode].minimumChargeTime);
      left += 60 * 1000; // Aufrunden auf nächste Minute
      hours = left / (60UL * 60UL * 1000UL);
      minutes = (left % (60UL * 60UL * 1000UL)) / (60UL * 1000UL);
      if (hours < 10)
        displayRight.print("0");
      displayRight.print(hours);
      displayRight.print(":");
      if (minutes < 10)
        displayRight.print("0");
      displayRight.print(minutes);
      displayRight.print(" h");
    }

    displayRight.setCursor(35, 40);
    hours = displaySelectedValue / 2;
    minutes = (displaySelectedValue % 2) * 30;
    if (hours < 10)
      displayRight.print("0");
    displayRight.print(hours);
    displayRight.print(":");
    if (minutes < 10)
      displayRight.print("0");
    displayRight.print(minutes);
    displayRight.print(" h");
  }
  else if (displayRightMode == 2)
  {
    displayRight.setCursor(45, 0);
    displayRight.print("Logs");

    displayRight.setTextSize(1);

    for (uint8_t i = 0; i < 6; i++)
    {
      uint16_t offset = displaySelectedValue * 5 + i;

      if (offset >= logCount)
        break;

      int index = (logHead - 1 - offset);
      if (index < 0)
        index += LOG_LINES;

      displayRight.setCursor(0, i * 8 + 20);
      displayRight.print(logBuffer[index]);
    }
  }

  displayRight.display();
  displayRightNextUpdate = now + 10UL * 1000UL;
}

void btnLightClicked()
{
  addLog("BTN_LIGHT clicked");
  unsigned long now = millis();

  if (relays[RELAY_7_LIGHT_BACK].read())
  {
    relays[RELAY_5_LIGHT_MIDDLE].set(false, now);
    relays[RELAY_6_LIGHT_FRONT].set(false, now);
    relays[RELAY_7_LIGHT_BACK].set(false, now);
    return;
  }

  if (relays[RELAY_5_LIGHT_MIDDLE].read())
  {
    relays[RELAY_5_LIGHT_MIDDLE].triggerTimeout(now);
    relays[RELAY_6_LIGHT_FRONT].triggerTimeout(now);
    relays[RELAY_7_LIGHT_BACK].triggerTimeout(now);
    return;
  }

  if (relays[RELAY_6_LIGHT_FRONT].read())
  {
    relays[RELAY_5_LIGHT_MIDDLE].triggerTimeout(now);
    relays[RELAY_6_LIGHT_FRONT].triggerTimeout(now);
    relays[RELAY_7_LIGHT_BACK].set(false, now);
    return;
  }

  relays[RELAY_5_LIGHT_MIDDLE].set(false, now);
  relays[RELAY_6_LIGHT_FRONT].triggerTimeout(now);
  relays[RELAY_7_LIGHT_BACK].set(false, now);
}

void btnUpClicked()
{
  displaySelectedValue++;
  if (displaySelectedValue > 48)
    displaySelectedValue = 0;
  displayRightNextUpdate = 0;
}
void btnRightClicked()
{
  displayRightMode = (displayRightMode + 1) % 3;
  displaySelectedValue = 0;
  displayRightNextUpdate = 0;
}
void btnDownClicked()
{
  displaySelectedValue--;
  if (displaySelectedValue > 48)
    displaySelectedValue = 48;
  displayRightNextUpdate = 0;
}
void btnLeftClicked()
{
  displayRightMode = (displayRightMode + 2) % 3;
  displaySelectedValue = 0;
  displayRightNextUpdate = 0;
}
void btnOkayClicked()
{
  displayRightNextUpdate = 0;
  if (displayRightMode == 0 || displayRightMode == 1)
  {
    relays[chargeConfigs[displayRightMode].relayPin].set(true, millis());
    chargeConfigs[displayRightMode].chargeState = ChargeState::CHARGING_MIN_TIME;
    chargeConfigs[displayRightMode].minimumChargeTime = displaySelectedValue * 30UL * 60UL * 1000UL;
    chargeConfigs[displayRightMode].minChargeTimer.start(millis());
    displaySelectedValue = 0;
  }
}

// ----------------------------------------------------------
// SETUP
// ----------------------------------------------------------
void setup()
{

  Serial.begin(115200);

  // MCP23017 zurücksetzen
  pinMode(D0, OUTPUT);
  digitalWrite(D0, HIGH);
  delay(10);
  digitalWrite(D0, LOW);
  delay(100);
  digitalWrite(D0, HIGH);

  // I2C starten
  Wire.begin();

  bool error = true;
  bool addressFound = false;
  for (int i = 1; i < 128; i++)
  {
    Wire.beginTransmission(i);
    error = Wire.endTransmission();
    if (error == 0)
    {
      addressFound = true;
      Serial.print("0x");
      Serial.println(i, HEX);
    }
  }
  if (!addressFound)
  {
    Serial.println("Keine Adresse erkannt");
  }
  Serial.println();

  //   // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!displayLeft.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  //   // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!displayRight.begin(SSD1306_SWITCHCAPVCC, 0x3D))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  // MCP auf dem Backend einrichten
  mcp_backend.begin(MCP_BACKEND_ADDR);

  mcp_backend.setPinMode(SW_24VOLT, INPUT);
  mcp_backend.configureClick(SW_RADIO0, INPUT_PULLUP, clickGate);
  mcp_backend.configureClick(SW_RADIO1, INPUT_PULLUP, clickLid);
  mcp_backend.configureClick(SW_RADIO2, INPUT_PULLUP, clickBell);
  mcp_backend.setPinMode(SW_MOTION, INPUT);
  mcp_backend.setCallbackChange(SW_MOTION, triggerLight);
  mcp_backend.setPinMode(SW_LID_END_UP, INPUT_PULLUP);
  mcp_backend.setPinMode(SW_LID_END_DOWN, INPUT_PULLUP);
  mcp_backend.setPinMode(SW_DOOR_BACK, INPUT_PULLUP, LOW);
  mcp_backend.setCallbackChange(SW_DOOR_BACK, []()
                                {
                                    addLog("DOOR BACK");
                                    mqttSendAdebarGarageDoor(1, false); });

  mcp_backend.setPinMode(8 + RELAY_0_HEATING, OUTPUT);
  mcp_backend.digitalWrite(8 + RELAY_0_HEATING, HIGH); // Relais aus
  mcp_backend.setPinMode(8 + RELAY_1_CONTACTOR_3_PHASE, OUTPUT);
  mcp_backend.digitalWrite(8 + RELAY_1_CONTACTOR_3_PHASE, HIGH); // Relais aus
  mcp_backend.setPinMode(8 + RELAY_2_CONTACTOR_1_PHASE, OUTPUT);
  mcp_backend.digitalWrite(8 + RELAY_2_CONTACTOR_1_PHASE, HIGH); // Relais aus
  mcp_backend.setPinMode(8 + RELAY_3_LID_MOTOR, OUTPUT);
  mcp_backend.digitalWrite(8 + RELAY_3_LID_MOTOR, HIGH); // Relais aus
  mcp_backend.setPinMode(8 + RELAY_4_UNUSED, OUTPUT);
  mcp_backend.digitalWrite(8 + RELAY_4_UNUSED, HIGH); // Relais aus
  mcp_backend.setPinMode(8 + RELAY_5_LIGHT_MIDDLE, OUTPUT);
  mcp_backend.digitalWrite(8 + RELAY_5_LIGHT_MIDDLE, HIGH); // Relais aus
  mcp_backend.setPinMode(8 + RELAY_6_LIGHT_FRONT, OUTPUT);
  mcp_backend.digitalWrite(8 + RELAY_6_LIGHT_FRONT, HIGH); // Relais aus
  mcp_backend.setPinMode(8 + RELAY_7_LIGHT_BACK, OUTPUT);
  mcp_backend.digitalWrite(8 + RELAY_7_LIGHT_BACK, HIGH); // Relais aus

  // MCP auf dem Handheld einrichten
  mcp_handheld.begin(MCP_HANDHELD_ADDR);

  mcp_handheld.configureClick(SW_BTN_LIGHT, INPUT_PULLUP, btnLightClicked, HIGH);
  mcp_handheld.configureClick(SW_BTN_GATE, INPUT_PULLUP, clickGate, HIGH);
  mcp_handheld.configureClick(SW_BTN_LID, INPUT_PULLUP, clickLid, HIGH);
  mcp_handheld.configureClick(SW_BTN_UP, INPUT_PULLUP, btnUpClicked, HIGH);
  mcp_handheld.configureClick(SW_BTN_RIGHT, INPUT_PULLUP, btnRightClicked, HIGH);
  mcp_handheld.configureClick(SW_BTN_DOWN, INPUT_PULLUP, btnDownClicked, HIGH);
  mcp_handheld.configureClick(SW_BTN_LEFT, INPUT_PULLUP, btnLeftClicked, HIGH);
  mcp_handheld.configureClick(SW_BTN_OKAY, INPUT_PULLUP, btnOkayClicked, HIGH);

  mcp_handheld.setPinMode(LED_CHARGE_HEAVY, OUTPUT);
  mcp_handheld.digitalWrite(LED_CHARGE_HEAVY, LOW);

  mcp_handheld.setPinMode(LED_CHARGE_LIGHT, OUTPUT);
  mcp_handheld.digitalWrite(LED_CHARGE_LIGHT, LOW);

  mcp_handheld.setPinMode(LED_BTNS, OUTPUT);
  mcp_handheld.digitalWrite(LED_BTNS, LOW);

  mcp_handheld.setPinMode(LED_GATE, OUTPUT);
  mcp_handheld.digitalWrite(LED_GATE, HIGH);

  mcp_handheld.setPinMode(LED_LID, OUTPUT);
  mcp_handheld.digitalWrite(LED_LID, LOW);

  mcp_handheld.setPinMode(SW_UNUSED_1, INPUT_PULLUP);
  mcp_handheld.setPinMode(SW_DOOR_FRONT, INPUT_PULLUP, LOW);
  mcp_handheld.setCallbackChange(SW_DOOR_FRONT, []()
                                 {
    addLog("DOOR FRONT Changed");
    mqttSendAdebarGarageDoor(0, false); });
  mcp_handheld.setPinMode(SW_UNUSED_2, INPUT_PULLUP);

  // Motor einrichten
  pinMode(MOTOR_PWM_UP, OUTPUT);
  pinMode(MOTOR_PWM_DOWN, OUTPUT);
  pinMode(A0, INPUT);

  state = STOPPING;

  // WLAN und MQTT vorbereiten
  WiFi.mode(WIFI_STA);
  mqttClient.setBufferSize(1024);
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);

  // Over the Air updates
  setupOTA();

  // Alle Relais aus
  for (int i = 0; i < 8; i++)
    relays[i].set(false, millis());

  displayLeft.clearDisplay();
  displayLeft.setTextColor(SSD1306_WHITE);
  displayLeft.display();
  displayRight.clearDisplay();
  displayRight.setTextColor(SSD1306_WHITE);
  displayRight.display();
}

// ----------------------------------------------------------
// LOOP
// ----------------------------------------------------------
void loop()
{
  // Aktuelle Zeit nur einmal berechnen
  unsigned long now = millis();

  // WLAN und MQTT-Stuff
  if (connectWifiNonBlocking(now))
  {
    ArduinoOTA.handle();
    if (connectMQTT(now))
    {
      mqttClient.loop();

      if ((lastMqttStatusUpdate + MQTT_STATUS_INTERVAL) < now)
        mqttSendStatus(true);
    }
  }

  // MCPs auslesen und Handler ausführen
  mcp_backend.loop(now);
  mcp_handheld.loop(now);

  // Tor-Steuerung
  handleLid(now);

  // Alle Relais
  for (uint8_t i = 0; i < 8; i++)
    relays[i].loop(now);

  updateCharging(now, 0);
  updateCharging(now, 1);

  updateDisplayLeft(now);
  updateDisplayRight(now);

  if (mcp_backend.digitalRead(SW_DOOR_BACK) == DOOR_OPEN || mcp_handheld.digitalRead(SW_DOOR_FRONT) == DOOR_OPEN)
    triggerLight();

  if (chargeConfigs[0].chargeState == ChargeState::IDLE || chargeConfigs[0].chargeState == ChargeState::PRECONDITION)
    mcp_handheld.setOutputModus(LED_CHARGE_LIGHT, off);
  else
    mcp_handheld.setOutputModus(LED_CHARGE_LIGHT, blink);

  if (chargeConfigs[1].chargeState == ChargeState::IDLE || chargeConfigs[1].chargeState == ChargeState::PRECONDITION)
    mcp_handheld.setOutputModus(LED_CHARGE_HEAVY, off);
  else
    mcp_handheld.setOutputModus(LED_CHARGE_HEAVY, blink);
}
