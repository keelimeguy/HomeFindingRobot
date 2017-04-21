#ifndef ROBOT_H
#define ROBOT_H

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
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

// Robot Direction Variables
#define DIRECTION_UP 1
#define DIRECTION_DOWN 2
#define DIRECTION_RIGHT 3
#define DIRECTION_LEFT 4

// Line Sensor Debounce Vaiables
#define PUSH_STATE_NOPUSH 1
#define PUSH_STATE_MAYBE 2
#define PUSH_STATE_MAYBE_FROM_PUSH 3
#define PUSH_STATE_PUSHED 4
#define DEBOUNCE_WAIT 10 // ms

// Line Sensor ADC Variables
#define ADC_COMPLETE ((ADCSRA & (1<<ADSC)) == 0)
#define V_REF 1.0
#define SENSOR_THRESHOLD 0.4*V_REF
#define ADC_MIDDLE_SENSOR 0
#define ADC_LEFT_SENSOR 0 // left sensor currently not connected
#define ADC_RIGHT_SENSOR 0 // right sensor currently not connected

// Remote Variables
#define JOY_X_MAX 910
#define JOY_X_NORM 525
#define JOY_X_MIN 50
#define JOY_Y_MAX 810
#define JOY_Y_NORM 515
#define JOY_Y_MIN 70
#define JOY_DELTA 10

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

void setup(uint8_t calibrate);

uint8_t getPositionX();
uint8_t getPositionY();
void setPosition(int x, int y);

void getHeading(float* Yaw, float* Pitch, float* Roll);
void getDisplacement(char* stat, char* x, char* y);

double driftCorrection(char stat, char x, char y, uint8_t straight);
double readLineSensors(void);

#endif
