#include "MCP23017Controller.h"

// I2C error thresholds
#define I2C_ERROR_THRESHOLD 8
#define I2C_REINIT_MIN_INTERVAL_MS (60UL * 1000UL) // 60s

void MCP23017Controller::begin(uint8_t addr, void (*resetCallback)(uint8_t addr))
{
  _addr = addr;
  _gpioA = _gpioB = 0;
  _olatA = _olatB = 0;
  _i2cErrorCount = 0;
  _lastReinitAttempt = 0;
  _resetCallback = resetCallback; // Store the reset callback

  // Pins initialisieren (vermeidet undefiniertes Verhalten)
  for (int i = 0; i < 16; i++)
  {
    _pins[i].number = i;
    _pins[i].isInput = true;
    _pins[i].state = HIGH;
    _pins[i].debouncedState = HIGH;
    _pins[i].writeState = LOW;
    _pins[i].pendingWrite = false;
    _pins[i].pullupEnabled = false;
    _pins[i].lastDebounce = 0;
    _pins[i].onClick = NULL;
    _pins[i].onChange = NULL;
  }

  // Default: alle Input
  writeRegister(0x00, 0xFF); // IODIRA
  writeRegister(0x01, 0xFF); // IODIRB

  // Pullups aktivieren
  writeRegister(0x0C, 0xFF); // Port A
  writeRegister(0x0D, 0xFF); // Port B
}

void MCP23017Controller::setPinMode(uint8_t pin, uint8_t mode, int8_t initialState)
{
  bool input = mode != OUTPUT;
  _pins[pin].isInput = input;

  // Keep original pin number for internal array indexing
  uint8_t origPin = pin;
  uint8_t addr = 0x00; // iodirA
  uint8_t pinIndex = pin;
  if (pin >= 8)
  {
    addr = 0x01; // iodirB
    pinIndex = pin - 8;
  }

  uint8_t iodir = readRegister(addr);
  iodir = input ? (iodir | (1 << pinIndex)) : (iodir & ~(1 << pinIndex));
  writeRegister(addr, iodir);

  if (input)
  {
    addr = addr + 0x0C; // A => 0x0C ; B => 0x0D
    bool pullup = mode == INPUT_PULLUP;
    _pins[origPin].pullupEnabled = pullup;
    iodir = readRegister(addr);
    iodir = iodir | (pullup << pinIndex);
    writeRegister(addr, iodir);

    // If an initial state was provided, set internal state/debouncedState
    // to that value so callbacks aren't triggered immediately at startup.
    if (initialState != -1)
    {
      bool st = initialState ? HIGH : LOW;
      _pins[origPin].state = st;
      _pins[origPin].debouncedState = st;
      // Start debounce window from now to avoid immediate callbacks
      _pins[origPin].lastDebounce = millis();
    }
  }
  else
  {
    // Output -> no pullup
    _pins[origPin].pullupEnabled = false;
  }
}

void MCP23017Controller::digitalWrite(uint8_t pin, bool newState)
{
  if (_pins[pin].isInput)
    return;

  _pins[pin].writeState = newState;
  _pins[pin].pendingWrite = true;
}

bool MCP23017Controller::digitalRead(uint8_t pin)
{
  if (_pins[pin].pendingWrite)
    return _pins[pin].writeState;

  return _pins[pin].debouncedState;
}

void MCP23017Controller::setCallbackClick(uint8_t pin, void (*callback)())
{
  _pins[pin].onClick = callback;
}

void MCP23017Controller::setCallbackChange(uint8_t pin, void (*callback)())
{
  _pins[pin].onChange = callback;
}

void MCP23017Controller::configureClick(uint8_t pin, uint8_t mode, void (*callback)(), int8_t initialState)
{
  setPinMode(pin, mode, initialState);
  setCallbackClick(pin, callback);
}

void MCP23017Controller::setOutputModus(uint8_t pin, OutputModus modus)
{
  _pins[pin].outputModus = modus;
}

void MCP23017Controller::loop(unsigned long now)
{
  for (int i = 0; i < 16; i++)
  {
    if (!_pins[i].isInput && _pins[i].outputModus != disabled)
      handleOutputModus(i, now);
  }

  readRegisters(now);
  writeRegisters();
}

void MCP23017Controller::handleOutputModus(uint8_t pin, unsigned long now)
{
  if (_pins[pin].outputModus == off)
  {
    digitalWrite(pin, LOW);
  }
  else if (_pins[pin].outputModus == on)
  {
    digitalWrite(pin, HIGH);
  }
  else if (_pins[pin].outputModus == blink || _pins[pin].outputModus == fastBlink)
  {
    unsigned long interval = (_pins[pin].outputModus == fastBlink) ? 200 : 500; // Blink intervals
    bool state = ((now / interval) % 2) == 0;
    digitalWrite(pin, state);
  }
}

void MCP23017Controller::readRegisters(unsigned long now)
{
  // Read both GPIO registers in a single I2C transaction to get an atomic snapshot
  // and reduce bus traffic/blocking caused by separate reads.
  unsigned long t0 = millis();
  bool readOk = false;
  const int attempts = 2;
  for (int a = 0; a < attempts; a++)
  {
    Wire.beginTransmission(_addr);
    Wire.write((uint8_t)0x12);
    uint8_t endRes = Wire.endTransmission();
    if (endRes != 0)
    {
      _i2cErrorCount++;
      delay(3);
      continue;
    }

    Wire.requestFrom((int)_addr, (int)2);
    if (Wire.available() >= 2)
    {
      _gpioA = Wire.read();
      _gpioB = Wire.read();
      readOk = true;
      _i2cErrorCount = 0;
      break;
    }
    else
    {
      _i2cErrorCount++;
      delay(3);
    }
  }

  unsigned long dt = millis() - t0;
  if (dt > 100)
  {
    // Log unusually long I2C read durations for diagnosis
    Serial.print("MCP23017: gpio read took ms=");
    Serial.println(dt);
  }

  if (!readOk)
  {
    // fallback to previous cached values (prevents ghost-presses)
    // and possibly trigger restore if many errors
    // the resetAndRestoreRegisters call is handled above on threshold
  }

  for (int i = 0; i < 16; i++)
  {
    bool newState = i < 8 ? (_gpioA >> i) & 1 : (_gpioB >> (i - 8)) & 1;
    bool lastState = _pins[i].state;

    _pins[i].state = newState;

    if (!_pins[i].isInput)
      continue;

    if (newState != lastState)
      _pins[i].lastDebounce = now;

    if (_pins[i].debouncedState != newState && now - _pins[i].lastDebounce > DEBOUNCE_MS)
    {
      _pins[i].debouncedState = newState;
      if (_pins[i].onChange)
        _pins[i].onChange();
      if (!newState && _pins[i].onClick) // 0 heißt gedrückt
        _pins[i].onClick();
    }
  }
}

void MCP23017Controller::writeRegisters()
{
  uint8_t outA = 0, outB = 0;
  for (int i = 0; i < 8; i++)
  {
    if (!_pins[i].isInput)
    {
      if (_pins[i].pendingWrite)
      {
        outA |= (_pins[i].writeState << i);
        _pins[i].pendingWrite = false;
        _pins[i].debouncedState = _pins[i].writeState;
      }
      else
      {
        outA |= (_pins[i].state << i);
      }
    }
  }
  for (int i = 0; i < 8; i++)
  {
    if (!_pins[i + 8].isInput)
    {
      if (_pins[i + 8].pendingWrite)
      {
        outB |= (_pins[i + 8].writeState << i);
        _pins[i + 8].pendingWrite = false;
        _pins[i + 8].debouncedState = _pins[i + 8].writeState;
      }
      else
      {
        outB |= (_pins[i + 8].state << i);
      }
    }
  }
  // OLAT-Register für Ausgänge schreiben (besser als direkt GPIO)
  writeRegister(0x14, outA); // OLATA
  writeRegister(0x15, outB); // OLATB
}

void MCP23017Controller::writeRegister(uint8_t reg, uint8_t val)
{
  const int maxAttempts = 3;
  for (int attempt = 0; attempt < maxAttempts; attempt++)
  {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    uint8_t res = Wire.endTransmission();
    if (res == 0)
    {
      return; // Success
    }
    else
    {
      // I2C error - increment error count and attempt recovery if threshold exceeded
      _i2cErrorCount++;
      resetAndRestoreRegisters(); // Attempt immediate recovery on write errors

      delay(1);
    }
  }
}

uint8_t MCP23017Controller::readRegister(uint8_t reg)
{
  // Robust reading: retry a few times on transient I2C errors. If still failing,
  // return a sensible fallback for GPIO registers (use cached _gpioA/_gpioB),
  // otherwise return 0xFF to indicate 'no change' instead of 0 which would be
  // interpreted as a pressed (LOW) input.
  const int maxAttempts = 3;
  for (int attempt = 0; attempt < maxAttempts; attempt++)
  {
    Wire.beginTransmission(_addr);
    // Wire.write returns the number of bytes written but we ignore it here
    Wire.write(reg);
    uint8_t endRes = Wire.endTransmission();
    if (endRes != 0)
    {
      // bus error / NACK - short delay and retry
      _i2cErrorCount++;
      delay(1);
      resetAndRestoreRegisters(); // Attempt recovery on repeated read errors
      continue;
    }

    Wire.requestFrom((int)_addr, (int)1);
    if (Wire.available())
    {
      uint8_t v = Wire.read();
      // success -> clear error counter
      _i2cErrorCount = 0;
      return v;
    }
    // no data available - short delay then retry
    _i2cErrorCount++;
    resetAndRestoreRegisters(); // Attempt recovery on repeated read errors
    delay(1);
  }

  // Wenn wiederholte Leseversuche fehlschlagen, gebe für GPIO-Register den zuletzt
  // gelesenen Wert zurück, damit zumindest die Eingangslogik weiter funktioniert.
  if (reg == 0x12)
    return _gpioA;
  if (reg == 0x13)
    return _gpioB;
  return 0xFF;
}

void MCP23017Controller::resetAndRestoreRegisters(bool forceReset)
{
  if (!(forceReset || (_i2cErrorCount >= I2C_ERROR_THRESHOLD && (millis() - _lastReinitAttempt) > I2C_REINIT_MIN_INTERVAL_MS)))
  {
    return; // Not time to reset yet
  }

  if (_resetCallback)
    _resetCallback(_addr); // Trigger external hardware reset via callback

  _lastReinitAttempt = millis();
  _i2cErrorCount = 0;

  // Restore configuration registers from current _pins[] state. This writes
  // IODIR A/B, GPPU A/B and OLAT A/B based on the in-memory pin configuration.

  uint8_t iodirA = 0, iodirB = 0;
  uint8_t gppuA = 0, gppuB = 0;
  uint8_t olatA = 0, olatB = 0;

  for (int i = 0; i < 8; i++)
  {
    if (_pins[i].isInput)
      iodirA |= (1 << i);
    if (_pins[i].pullupEnabled)
      gppuA |= (1 << i);
    if (!_pins[i].isInput)
      olatA |= (_pins[i].debouncedState << i);
  }
  for (int i = 0; i < 8; i++)
  {
    int idx = i + 8;
    if (_pins[idx].isInput)
      iodirB |= (1 << i);
    if (_pins[idx].pullupEnabled)
      gppuB |= (1 << i);
    if (!_pins[idx].isInput)
      olatB |= (_pins[idx].debouncedState << i);
  }

  // Write these registers back to the device (best-effort)
  writeRegister(0x00, iodirA); // IODIRA
  writeRegister(0x01, iodirB); // IODIRB
  writeRegister(0x0C, gppuA);  // GPPUA
  writeRegister(0x0D, gppuB);  // GPPUB
  writeRegister(0x14, olatA);  // OLATA
  writeRegister(0x15, olatB);  // OLATB

  // Also update cached values so read fallback is consistent
  _olatA = olatA;
  _olatB = olatB;

}
