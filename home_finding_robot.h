/*
 * HomeFindingRobot: home_finding_robot.h
 * Author : Keelin, Martial
 */

#ifndef ___HOME_FINDING_ROBOT___
#define ___HOME_FINDING_ROBOT___

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Motor Variables
#define MAX_SPEED 1.00
#define MIN_SPEED 0.40
#define LEFT_MOTOR_DIR_CHANGEABLE (leftDutyCycle != 0)
#define RIGHT_MOTOR_DIR_CHANGEABLE (rightDutyCycle != 0)
#define DIR_FORWARD 5     // 0b0101  [L_in1][L_in2][R_in1][R_in2]
#define DIR_BACKWARD 10   // 0b1010
#define DIR_LEFT 9        // 0b1001
#define DIR_RIGHT 6       // 0b0110
#define DIR_STOP 0        // 0b0000

void setup(void);
void moveForward(double speed);
void moveBackward(double speed);
void rotateRight(double speed);
void rotateLeft(double speed);
void stop(void);

#endif
