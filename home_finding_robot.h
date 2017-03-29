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
#include "BT.h"

// Robot Mode Variables
#define ROBOT_MODE_CONTROL_AND_AVOID 1
#define ROBOT_MODE_AUTONOMOUS_AVOID 2

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

// Line Sensor Debounce Vaiables
#define PUSH_STATE_NOPUSH 1
#define PUSH_STATE_MAYBE 2
#define PUSH_STATE_MAYBE_FROM_PUSH 3
#define PUSH_STATE_PUSHED 4
#define DEBOUNCE_WAIT 10 // ms

// Line Sensor ADC Variables
#define ADC_COMPLETE ((ADCSRA & (1<<ADSC)) == 0)
#define V_REF 1.0
#define SENSOR_THRESHOLD 0.8*V_REF
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
#define KI (0.50 * KP / OSCILLATION_PERIOD)
#define KD (KP * OSCILLATION_PERIOD / 8.0)
#define KBIAS 0.13
#define I_SIZE_MAX 50

// Timer Variables
#define MAX_TIMERS 2

void setup(uint8_t calibrate);

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

double readLineSensors(void);

void setBTTimeout(int time);
uint8_t error(void);

void start_counting(int index);
unsigned long timer_ellapsed_micros(int index, uint8_t reset);

void setDelay_ms(int delay);
uint8_t delay_done(void);

#endif
