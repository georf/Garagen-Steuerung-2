#include "Relay.h"
#include "build_config.h"
// Forward declaration of global relays array (defined in main.cpp)
extern Relay relays[];

// Special relays that must never be on at the same time and require a 30s both-off delay
#define BOTH_OFF_DELAY 30UL * 1000UL // 30 seconds
#define SPECIAL_RELAY_3PHASE 1       // Dreiphasen-Kontaktor
#define SPECIAL_RELAY_1PHASE 2       // Einphasen-Kontaktor

static bool phase1wanted = false;
static bool phase3wanted = false;
static bool todo = false;

void loopSpecialRelays(unsigned long now)
{
  Relay &relay3 = relays[SPECIAL_RELAY_3PHASE];
  Relay &relay1 = relays[SPECIAL_RELAY_1PHASE];

  if (phase3wanted && !relay3.read())
  {
    // Schritt 1: Beide Relais aus, damit Umschalten möglich ist
    if (relay1.read()) {
      relay1.set(false, now, true);
      return;
    }

    // Schritt 2: Beide Relais müssen mindestens 30 Sekunden aus sein, bevor wir das andere einschalten können
    if (now - relay3.getLastStopTime() >= BOTH_OFF_DELAY && now - relay1.getLastStopTime() >= BOTH_OFF_DELAY)
    {
      relay3.set(true, now, true);
      todo = false;
      return;
    }   
  }
  else if (phase1wanted && !relay1.read() && !relay3.read())
  {
    // Schritt 1: Beide Relais müssen mindestens 30 Sekunden aus sein, bevor wir das andere einschalten können
    if (now - relay3.getLastStopTime() >= BOTH_OFF_DELAY && now - relay1.getLastStopTime() >= BOTH_OFF_DELAY)
    {
      relay1.set(true, now, true);
      todo = false;
      return;
    }
  }
}

void requestSpecialRelay(uint8_t index, bool state, unsigned long now)
{
  Relay &relay3 = relays[SPECIAL_RELAY_3PHASE];
  Relay &relay1 = relays[SPECIAL_RELAY_1PHASE];

  if (index == SPECIAL_RELAY_1PHASE)
  {
    phase1wanted = state;
    if (state && !relay1.read())
    {
      todo = true;
    }

    if (!state)
    {
      relay1.set(false, now, true);
    }
  }
  else if (index == SPECIAL_RELAY_3PHASE)
  {
    phase3wanted = state;
    if (state && !relay3.read())
    {
      todo = true;
    }
    else if (!state)
    {
      relay3.set(false, now, true);
      if (phase1wanted)
        todo = true;
    }
  }
}

Relay::Relay(MCP23017Controller &mcp, uint8_t index, char *name, uint16_t watt, uint8_t timeoutMinutes)
{
  // store pointer to the passed controller to avoid copying
  _mcp = &mcp;
  _index = index;
  _mcpPin = index + 8; // Relais auf Port B
  _name = name;
  _watt = watt;
  _timeoutSeconds = timeoutMinutes * 60;
}

void Relay::set(bool state, unsigned long now, bool intern)
{
  if (!intern && (_index == SPECIAL_RELAY_1PHASE || _index == SPECIAL_RELAY_3PHASE))
  {
    requestSpecialRelay(_index, state, now);
    return;
  }

  if (state)
  {
    if (read())
      return; // Bereits an

    _mcp->digitalWrite(_mcpPin, RELAY_ON);
    _lastStartTime = now;
    _timeout = 0;

    if (_watt != 0)
    {
      char bufferTopic[64];
      snprintf(bufferTopic, sizeof(bufferTopic), ADEBAR_TOPIC("garage/relay%d/energy"), _index);
      mqttPublish(bufferTopic, "0");
    }
  }
  else
  {

    if (!read())
      return; // Bereits aus

    _mcp->digitalWrite(_mcpPin, RELAY_OFF);
    _lastStopTime = now;

    // Wenn das Relais ausgeschaltet wird, Energieverbrauch berechnen
    if (_lastStartTime != 0 && _watt != 0)
    {
      unsigned long wasRunningMillis = now - _lastStartTime;
      unsigned long wasRunningSeconds = (wasRunningMillis / 1000);
      float energy = ((float)_watt / 3600) * wasRunningSeconds; // Wattsekunden to Wattstunden

      char bufferSmall[64];
      char bufferTopic[64];
      snprintf(bufferSmall, sizeof(bufferSmall), "%f", energy);
      snprintf(bufferTopic, sizeof(bufferTopic), ADEBAR_TOPIC("garage/relay%d/energy"), _index);
      mqttPublish(bufferTopic, bufferSmall);
    }
    _lastStartTime = 0;
    _timeout = 0;
  }
  mqttPublishState(false);
}

void Relay::triggerOrSet(bool state, unsigned long now)
{
  if (state && _timeoutSeconds)
    triggerTimeout(now);
  else
    set(state, now);
}

bool Relay::read()
{
  return _mcp->digitalRead(_mcpPin) == RELAY_ON;
}

void Relay::triggerTimeout(unsigned long now)
{
  set(true, now);
  _timeout = now + ((unsigned long)_timeoutSeconds * 1000UL);
}

void Relay::loop(unsigned long now)
{
  if(todo && (_index == SPECIAL_RELAY_1PHASE || _index == SPECIAL_RELAY_3PHASE))
  {
    loopSpecialRelays(now);
  }

  // Handle normal timeout (wrap-safe comparison)
  if (_timeout != 0 && (long)(now - _timeout) >= 0)
  {
    set(false, now);
    mqttDebug("Relay timeout triggered");
    _timeout = 0;
  }
}

unsigned long Relay::getLastStopTime()
{
  return _lastStopTime;
}

unsigned long Relay::getLastStartTime()
{
  return _lastStartTime;
}

uint8_t Relay::getIndex()
{
  return _index;
}

void Relay::mqttPublishState(bool full)
{
  char bufferSmall[128];
  char bufferBig[512];

  if (full)
  {
    // see https://www.home-assistant.io/integrations/switch.mqtt/
    JsonDocument discoveryConfig;

    snprintf(bufferSmall, sizeof(bufferSmall), "Garagenschalter %d %s", _index, _name);
    discoveryConfig["name"] = bufferSmall;

    snprintf(bufferSmall, sizeof(bufferSmall), ADEBAR_TOPIC("garage/relay%d/state"), _index);
    discoveryConfig["stat_t"] = bufferSmall;

    snprintf(bufferSmall, sizeof(bufferSmall), ADEBAR_TOPIC("garage/relay%d/set"), _index);
    discoveryConfig["cmd_t"] = bufferSmall;

    snprintf(bufferSmall, sizeof(bufferSmall), ADEBAR_ROOT "_garage_relay%d", _index);
    discoveryConfig["uniq_id"] = bufferSmall;

    JsonObject device = discoveryConfig["dev"].to<JsonObject>();
    device["identifiers"][0] = ADEBAR_ROOT "_garage";
    device["name"] = "Garage";

    serializeJson(discoveryConfig, bufferBig);
    snprintf(bufferSmall, sizeof(bufferSmall), "homeassistant/switch/" ADEBAR_ROOT "_garage_relay%d/config", _index);
    mqttPublish(bufferSmall, bufferBig);

    if (_watt != 0)
    {
      discoveryConfig.clear();

      snprintf(bufferSmall, sizeof(bufferSmall), "Garagenschalter %d %s Verbrauch", _index, _name);
      discoveryConfig["name"] = bufferSmall;

      discoveryConfig["unit_of_meas"] = "Wh";
      discoveryConfig["dev_cla"] = "energy";
      discoveryConfig["stat_cla"] = "total_increasing";

      snprintf(bufferSmall, sizeof(bufferSmall), ADEBAR_TOPIC("garage/relay%d/energy"), _index);
      discoveryConfig["stat_t"] = bufferSmall;

      snprintf(bufferSmall, sizeof(bufferSmall), ADEBAR_ROOT "_garage_relay%d_energy", _index);
      discoveryConfig["uniq_id"] = bufferSmall;

      JsonObject device = discoveryConfig["dev"].to<JsonObject>();
      device["identifiers"][0] = ADEBAR_ROOT "_garage";
      device["name"] = "Garage";

      serializeJson(discoveryConfig, bufferBig);
      snprintf(bufferSmall, sizeof(bufferSmall), "homeassistant/sensor/" ADEBAR_ROOT "_garage_relay%d_energy/config", _index);
      mqttPublish(bufferSmall, bufferBig);
    }
  }

  snprintf(bufferSmall, sizeof(bufferSmall), ADEBAR_TOPIC("garage/relay%d/state"), _index);
  mqttPublish(bufferSmall, read() ? "ON" : "OFF");
}
