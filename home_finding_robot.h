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
#include "mpu9250_lib.h"
#include "ps2.h"

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

// Servo Variables
#define SERVO_PERIOD 20.096 //1024*157*2/16MHz
#define SERVO_MIN .768 // duty:6
#define SERVO_MAX 2.816 // duty:22

void setup(void);

void moveForward(void);
void moveBackward(void);
void rotateRight(void);
void rotateLeft(void);
void stop(void);
void setSpeed(double speedL, double speedR);

void setServoAngle(double angle);
void servoSweepTask(void);
void setServoSweep(double minAngle, double maxAngle, double angleStep, int timeDelay);
void servoSweepOn(void);
void servoSweepOff(void);

double getObstacleDistanceCm(void);
// double getObstacleDistanceInch(void);

void getHeading(float* Yaw, float* Pitch, float* Roll);
void getDisplacement(char* stat, char* x, char* y);

void start_counting(void);
unsigned long timer_ellapsed_micros(uint8_t reset);

void setDelay_ms(int delay);
uint8_t delay_done(void);

#endif
