#ifndef SERVO_H
#define SERVO_H

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
#include "ultrasonic_sensor.h"

#define SERVO_PERIOD 20.096 //1024*157*2/16MHz
#define SERVO_MIN .768 // duty:6
#define SERVO_MAX 2.816 // duty:22

void servo_init(void);
void setServoAngle(double angle);
double servoSweepDistanceTask(void);
void setServoSweep(double minAngle, double maxAngle, double angleStep, int timeDelay);
void servoSweepOn(void);
void servoSweepOff(void);

#endif
