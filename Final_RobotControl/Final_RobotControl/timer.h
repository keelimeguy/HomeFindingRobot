#ifndef TIMER_H
#define TIMER_H

#include <avr/interrupt.h>
#include "motor.h"
#include "BT.h"

// Timer Variables
#define MAX_TIMERS 2

void timer_init(void);
void setDelay_ms(int delay);
uint8_t delay_done(void);
void setSweep_ms(int time);
uint8_t sweepReady(void);
void setDebounce_ms(int time);
uint8_t debounceReady(void);
void setBTTimeout_ms(int time);
uint8_t error(void);
void start_counting(int index);
unsigned long timer_ellapsed_micros(int index, uint8_t reset);

#endif
