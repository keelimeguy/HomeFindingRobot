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

void mouse_init(void);
void mouse_pos(char* stat, char* x, char* y);

#endif
