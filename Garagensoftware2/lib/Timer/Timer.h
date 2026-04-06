#ifndef TIMER_H
#define TIMER_H

class Timer
{

private:
  unsigned long _startTime = 0;
  unsigned long _durationMs = 0;

public:
  void start(unsigned long now, unsigned long durationMs);
  unsigned long left(unsigned long now);
  bool elapsed(unsigned long now);
};

#endif