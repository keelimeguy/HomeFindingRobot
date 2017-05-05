#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <avr/io.h>
#include "timer.h"

// Robot Direction Variables
#define DIRECTION_UP     1
#define DIRECTION_DOWN   2
#define DIRECTION_RIGHT  3
#define DIRECTION_LEFT   4

// Line Sensor Debounce Vaiables
#define PUSH_STATE_NOPUSH           1
#define PUSH_STATE_MAYBE            2
#define PUSH_STATE_MAYBE_FROM_PUSH  3
#define PUSH_STATE_PUSHED           4

#define DEBOUNCE_WAIT  1 // ms

// Line Sensor ADC Variables
#define ADC_COMPLETE ((ADCSRA & (1<<ADSC)) == 0)
#define V_REF 1.0
#define SENSOR_THRESHOLD 0.4*V_REF
#define ADC_MIDDLE_SENSOR 0
// #define ADC_LEFT_SENSOR   0 // left sensor currently not connected
// #define ADC_RIGHT_SENSOR  0 // right sensor currently not connected

void line_sensor_init(void);
uint8_t getPositionX(void);
uint8_t getPositionY(void);
void setPosition(int x, int y);
void setDirection(int dir);
double updatePosition(void);

#endif
