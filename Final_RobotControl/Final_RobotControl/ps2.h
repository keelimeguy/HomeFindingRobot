#ifndef PS2_h
#define PS2_h

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define _ps2ddr  DDRC
#define _ps2port PORTC
#define _ps2pin  PINC
#define _ps2clk  2
#define _ps2data 3

// PID Variables
// Zeigler-Nichols Tuning:
#define ULTIMATE_GAIN 1.0            // Found through increasing Kp in P controller until oscillates
#define OSCILLATION_PERIOD 3.0      // Recorded once Proportional gain reaches Ku
#define KP (0.60 * ULTIMATE_GAIN)
#define KI (1.8 * KP / OSCILLATION_PERIOD)
#define KD (KP * OSCILLATION_PERIOD / 8.0)
#define KBIAS .001
#define I_SIZE_MAX 20
#define DRIFT_MAX_SIZE 5

void mouse_init(void);
void mouse_pos(char* stat, char* x, char* y);
double driftCorrection(char stat, char x, char y, uint8_t straight);

#endif
