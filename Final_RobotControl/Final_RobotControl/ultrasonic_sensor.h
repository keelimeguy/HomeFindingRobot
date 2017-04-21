#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void ultrasonic_sensor_init(void);
double getObstacleDistanceCm(void);
// double getObstacleDistanceInch(void);

#endif
