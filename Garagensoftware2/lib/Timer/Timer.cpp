#include "Timer.h"

void Timer::start(unsigned long now, unsigned long durationMs)
{
  _startTime = now;
  _durationMs = durationMs;
}

unsigned long Timer::left(unsigned long now)
{
  if (_startTime == 0)
    return 0;

  unsigned long elapsed = now - _startTime;
  if (elapsed >= _durationMs)
    return 0;
  return _durationMs - elapsed;
}

bool Timer::elapsed(unsigned long now)
{
  if (_startTime == 0)
    return false;
  return (now - _startTime) >= _durationMs;
}