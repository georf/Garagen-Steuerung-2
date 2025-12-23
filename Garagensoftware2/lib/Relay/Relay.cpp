#include "Relay.h"
// Forward declaration of global relays array (defined in main.cpp)
extern Relay relays[];

// Special relays that must never be on at the same time and require a 30s both-off delay
static const unsigned long BOTH_OFF_DELAY = 30UL * 1000UL; // 30 seconds
static const int SPECIAL_RELAY_A = 1;                      // Dreiphasen-Kontaktor
static const int SPECIAL_RELAY_B = 2;                      // Einphasen-Kontaktor

// Pending enable request when swap requires both relays to be off for BOTH_OFF_DELAY
static int pendingRelayIndex = -1;
static bool pendingRelayState = false;
static unsigned long pendingExecuteAt = 0;

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

void Relay::set(bool state, unsigned long now)
{
  // If this is one of the special contactor relays, enforce mutual exclusion
  if ((_index == SPECIAL_RELAY_A || _index == SPECIAL_RELAY_B) && state)
  {
    int other = (_index == SPECIAL_RELAY_A) ? SPECIAL_RELAY_B : SPECIAL_RELAY_A;

    // If the other special relay is currently ON, switch it OFF and schedule enabling this one
    if (relays[other].read())
    {
      // Turn other off now
      relays[other].set(false, now);

      // Schedule enabling of this relay after BOTH_OFF_DELAY
      pendingRelayIndex = _index;
      pendingRelayState = true;
      pendingExecuteAt = now + BOTH_OFF_DELAY;
      mqttDebug("Delaying special relay enable until both relays are off for 30s");
      return;
    }

    // If the other relay was turned off recently, ensure both have been off for BOTH_OFF_DELAY
    unsigned long otherLastStop = relays[other].getLastStopTime();
    if (otherLastStop != 0 && (now - otherLastStop) < BOTH_OFF_DELAY && !read())
    {
      // Schedule enabling at the time the 30s off period is finished
      pendingRelayIndex = _index;
      pendingRelayState = true;
      pendingExecuteAt = otherLastStop + BOTH_OFF_DELAY;
      mqttDebug("Delaying special relay enable to satisfy 30s both-off requirement");
      return;
    }
  }

  if (state)
  {
    if (read())
      return; // Bereits an

    _mcp->digitalWrite(_mcpPin, RELAY_ON);
    _lastStartTime = now;
    _timeout = 0;

    if (_watt != 0) {
      char bufferTopic[64];
      snprintf(bufferTopic, sizeof(bufferTopic), "adebar/garage/relay%d/energy", _index);
      mqttPublish(bufferTopic, "0");
    }
  }
  else
  {
    // If there was a pending enable for this relay, cancel it when the relay is explicitly turned off
    if (pendingRelayIndex == _index)
    {
      pendingRelayIndex = -1;
      pendingRelayState = false;
      pendingExecuteAt = 0;
    }

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
      snprintf(bufferTopic, sizeof(bufferTopic), "adebar/garage/relay%d/energy", _index);
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
  // Handle normal timeout
  if (_timeout != 0 && now >= _timeout)
  {
    set(false, now);
    mqttDebug("Relay timeout triggered");
    _timeout = 0;
  }

  // If there is a pending special-relay enable and its time has come, execute it
  if (pendingRelayIndex != -1 && now >= pendingExecuteAt)
  {
    int idx = pendingRelayIndex;
    bool state = pendingRelayState;

    // Clear pending before executing to avoid re-entrance problems
    pendingRelayIndex = -1;
    pendingRelayState = false;
    pendingExecuteAt = 0;

    // Execute the requested state
    relays[idx].set(state, now);
    mqttDebug("Executed pending special-relay enable after 30s off");
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

    snprintf(bufferSmall, sizeof(bufferSmall), "adebar/garage/relay%d/state", _index);
    discoveryConfig["stat_t"] = bufferSmall;

    snprintf(bufferSmall, sizeof(bufferSmall), "adebar/garage/relay%d/set", _index);
    discoveryConfig["cmd_t"] = bufferSmall;

    snprintf(bufferSmall, sizeof(bufferSmall), "adebar_garage_relay%d", _index);
    discoveryConfig["uniq_id"] = bufferSmall;

    JsonObject device = discoveryConfig["dev"].to<JsonObject>();
    device["identifiers"][0] = "adebar_garage";
    device["name"] = "Garage";

    serializeJson(discoveryConfig, bufferBig);
    snprintf(bufferSmall, sizeof(bufferSmall), "homeassistant/switch/adebar_garage_relay%d/config", _index);
    mqttPublish(bufferSmall, bufferBig);

    if (_watt != 0)
    {
      discoveryConfig.clear();

      snprintf(bufferSmall, sizeof(bufferSmall), "Garagenschalter %d %s Verbrauch", _index, _name);
      discoveryConfig["name"] = bufferSmall;

      discoveryConfig["unit_of_meas"] = "Wh";
      discoveryConfig["dev_cla"] = "energy";
      discoveryConfig["stat_cla"] = "total_increasing";

      snprintf(bufferSmall, sizeof(bufferSmall), "adebar/garage/relay%d/energy", _index);
      discoveryConfig["stat_t"] = bufferSmall;

      snprintf(bufferSmall, sizeof(bufferSmall), "adebar_garage_relay%d_energy", _index);
      discoveryConfig["uniq_id"] = bufferSmall;

      JsonObject device = discoveryConfig["dev"].to<JsonObject>();
      device["identifiers"][0] = "adebar_garage";
      device["name"] = "Garage";

      serializeJson(discoveryConfig, bufferBig);
      snprintf(bufferSmall, sizeof(bufferSmall), "homeassistant/sensor/adebar_garage_relay%d_energy/config", _index);
      mqttPublish(bufferSmall, bufferBig);
    }
  }

  snprintf(bufferSmall, sizeof(bufferSmall), "adebar/garage/relay%d/state", _index);
  mqttPublish(bufferSmall, read() ? "ON" : "OFF");
}
