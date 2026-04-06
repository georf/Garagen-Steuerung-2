#include <Arduino.h>
#include "Timer.h"
#include "Relay.h"

#ifdef DEBUG_BUILD
#define DEFAULT_CONDITION_TIME 60000       // 1 Minuten
#define DEFAULT_MINIMUM_CHARGE_TIME  60000 // 1 Minuten
#else
#define DEFAULT_CONDITION_TIME 5 * 60000       // 5 Minuten
#define DEFAULT_MINIMUM_CHARGE_TIME 10 * 60000 // 10 Minuten
#endif

#define TOLERANCE_FACTOR 10      // 10 * 50% (SOC)  => 500 W Toleranz


//  ----------------------------------------------------------
// Automatische Werte für Auto-Laden
enum class ChargeState
{
  IDLE,              // Laden aus
  PRECONDITION,      // Bedingungen prüfen (2 kW, Akku > 30 %)
  CHARGING_MIN_TIME, // Mindest-Ladezeit läuft
  CHARGING_ACTIVE,   // Mindestzeit vorbei, aber Laden läuft weiter
  POSTCONDITION,     // Bedingungen für Ausschalten prüfen (power < Toleranz)
};

class ChargeConfig
{

private:
  float _socThreshold;   // in %
  float _powerThreshold; // W (+ = lädt, - = entlädt)

public:
  ChargeConfig(Relay *relay, float socThreshold, float powerThreshold);

  void loop(unsigned long now, int powerNow, float batterySocNow);
  void setChargeTime(unsigned long now, unsigned long durationMs);
  void stopCharging(unsigned long now);

  bool hardCharge = false;

  Relay *relay;
  ChargeState state = ChargeState::IDLE;
  Timer conditionTimer;
  Timer minimumChargeTimer;
};