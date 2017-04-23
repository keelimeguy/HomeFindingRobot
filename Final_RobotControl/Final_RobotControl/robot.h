#ifndef ROBOT_H
#define ROBOT_H

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "line_sensor.h"
#include "ultrasonic_sensor.h"
#include "motor.h"
#include "servo.h"
#include "timer.h"
#include "mpu9250_lib.h"
#include "ps2.h"
#include "BT.h"

// Robot Mode Variables
#define ROBOT_MODE_CONTROL_AND_AVOID 1
#define ROBOT_MODE_AUTONOMOUS_AVOID 2
#define ROBOT_MODE_AUTONOMOUS_HOME 3

// Robot autonomous state variables
#define BUG_FORWARD_0 0
#define BUG_FORWARD_1 1
#define BUG_CHECK_CORNER_1 2
#define BUG_CHECK_CORNER_2 3
#define BUG_TURN_AT_CORNER 4
#define BUG_TURN_AT_WALL_0 5
#define BUG_TURN_AT_WALL_1 6
#define BUG_TURN_AT_WALL_2 7
#define BUG_TURN_AT_WALL_3 8
#define BUG_FOLLOW_WALL 9

// Remote Variables
#define JOY_X_MAX 910
#define JOY_X_NORM 525
#define JOY_X_MIN 50
#define JOY_Y_MAX 810
#define JOY_Y_NORM 515
#define JOY_Y_MIN 70
#define JOY_DELTA 10

void setup(uint8_t calibrate);

void getHeading(float* Yaw, float* Pitch, float* Roll);
void getDisplacement(char* stat, char* x, char* y);

uint8_t robot_control_and_avoid_task(double correction, float heading, uint8_t* straight);
uint8_t robot_autonomous_avoid_task(double correction, float heading, uint8_t* straight);
uint8_t robot_autonomous_home_task(double correction, float heading, uint8_t* straight);

#endif
