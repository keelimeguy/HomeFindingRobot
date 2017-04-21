#ifndef MOTOR_H
#define MOTOR_H

#define F_CPU 16000000UL
#include <avr/io.h>

#define MAX_SPEED 1.00
#define MIN_SPEED 0.40
#define LEFT_MOTOR_DIR_CHANGEABLE (leftDutyCycle != 0)
#define RIGHT_MOTOR_DIR_CHANGEABLE (rightDutyCycle != 0)
#define DIR_FORWARD 5     // 0b0101  [L_in1][L_in2][R_in1][R_in2]
#define DIR_BACKWARD 10   // 0b1010
#define DIR_LEFT 9        // 0b1001
#define DIR_RIGHT 6       // 0b0110
#define DIR_STOP 0        // 0b0000

void motor_init(void);
void motor_OVFTask(void);
void moveForward(void);
void moveBackward(void);
void rotateRight(void);
void rotateLeft(void);
void stop(void);
void setSpeed(double speedL, double speedR);

#endif
