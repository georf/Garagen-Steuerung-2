#include "ChargeConfig.h"

ChargeConfig::ChargeConfig(Relay *r, float socThreshold, float powerThreshold)
{
  relay = r;
  _socThreshold = socThreshold;
  _powerThreshold = powerThreshold;
}

void ChargeConfig::setChargeTime(unsigned long now, unsigned long durationMs)
{
  relay->set(true, now);
  hardCharge = true;
  minimumChargeTimer.start(now, durationMs);
  state = ChargeState::CHARGING_MIN_TIME;
}

void ChargeConfig::stopCharging(unsigned long now)
{
  relay->set(false, now);
  hardCharge = false;
  state = ChargeState::IDLE;
}

void ChargeConfig::loop(unsigned long now, int powerNow, float batterySocNow)
{
  switch (state)
  {

  case ChargeState::IDLE:
    if ((batterySocNow > _socThreshold && powerNow < _powerThreshold))
    {
      conditionTimer.start(now, DEFAULT_CONDITION_TIME);
      state = ChargeState::PRECONDITION;
    }
    return;

  case ChargeState::PRECONDITION:
    if (powerNow >= _powerThreshold)
    {
      state = ChargeState::IDLE;
      return;
    }

    if (conditionTimer.elapsed(now))
    {
      relay->set(true, now);
      hardCharge = false;
      minimumChargeTimer.start(now, DEFAULT_MINIMUM_CHARGE_TIME);
      state = ChargeState::CHARGING_MIN_TIME;
    }
    return;

  case ChargeState::CHARGING_MIN_TIME:
    // relay->set(true, now); // Immer wieder versuchern anzuschalten

    if (minimumChargeTimer.elapsed(now))
    {
      if (hardCharge && powerNow > 0)
      {
        // Sofort ausschalten, falls anderer Lader dran ist
        stopCharging(now);
        return;
      }

      hardCharge = false;
      state = ChargeState::CHARGING_ACTIVE;
    }
    return;

  case ChargeState::CHARGING_ACTIVE:
    // relay->set(true, now); // Immer wieder versuchern anzuschalten

    // Laden darf weiterlaufen solange Strom übrig ist
    if (powerNow > TOLERANCE_FACTOR * batterySocNow) // je voller der Akku, desto mehr Toleranz
    {
      conditionTimer.start(now, DEFAULT_CONDITION_TIME);
      state = ChargeState::POSTCONDITION;
    }
    return;

  case ChargeState::POSTCONDITION:
    // Laden darf weiterlaufen solange Strom übrig ist
    if (powerNow < TOLERANCE_FACTOR * batterySocNow) // je voller der Akku, desto mehr Toleranz
    {
      state = ChargeState::CHARGING_ACTIVE;
      return;
    }

    if (conditionTimer.elapsed(now))
    {
      stopCharging(now);
    }
    return;
  }
}