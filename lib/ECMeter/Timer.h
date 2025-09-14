#ifndef TIMER_H_
#define TIMER_H_

#include <Arduino.h>

typedef uint32_t timer_ticks_t;

// void timer_start(void);
void timer_sleep(uint32_t ticks);
void timer_sleep_5uS(uint32_t ticks);

#endif  // TIMER_H_