#include "motor.h"

// The duty cycle variables (speed control) for each set of motors
static uint8_t leftDutyCycle, rightDutyCycle;
// Flag to indicate if we need to duty cycle
static volatile uint8_t dutyUpdateFlag = 1;

static void setLeftMotorDirection(uint8_t dir);
static void setRightMotorDirection(uint8_t dir);
static uint8_t getDutyCycle(double dutyPercent, uint8_t max);
static void setDirection(uint8_t dir);
static void setLeftDutyCycle(double dutyPercent);
// static void motorLeftSet(uint8_t val); // unused
static void setRightDutyCycle(double dutyPercent);
// static void motorRightSet(uint8_t val); // unused

// Initialize the motor library
void motor_init(void) {
    // Motor pins setup
    DDRB |= (1<<DDB2)|(1<<DDB1)|(1<<DDB4); // Sets B4 (N2), B1 (N1), B2 (N4) as output pins
    DDRD |= (1<<DDD5)|(1<<DDD6)|(1<<DDD7); // Sets D5 (ENB), D6 (ENA), D7 (N3) as output pins

    // Start at 0 duty cycle
    leftDutyCycle = 0;
    rightDutyCycle = 0;
}

// Task when motor timer overflows, called from timer.c
void motor_OVFTask(void) {
    if (dutyUpdateFlag == 1) { // Update duty cycles if needed
        // We update duty cycle in timer overflow because we
        //    can only update duty cycles at top of waveform
        OCR0B = leftDutyCycle;
        OCR0A = rightDutyCycle;
        dutyUpdateFlag = 0; // Reset update flag
    }
}

// Find the duty cycle associated with the given duty percentage
static uint8_t getDutyCycle(double dutyPercent, uint8_t max) {
    int duty = (int)(dutyPercent * (double)(max)); // dutyPercent = [duty] / [max];

    // Keep the duty cycle in bound
    if (duty < 0) duty = 0;
    else if (duty > max) duty = max;

    return (uint8_t) duty;
}

// Set direction of left motor
static void setLeftMotorDirection(uint8_t dir) {
    // Set direction only when it is changeable or if it is stopped
    if (LEFT_MOTOR_DIR_CHANGEABLE || dir == DIR_STOP) {
        // turn off left control pins and set to dir
        PORTD = (PORTD & ~(1<<PORTD7)) | ((2 & dir)<<(PORTD7-1));
        PORTB = (PORTB & ~(1<<PORTB2)) | ((1 & dir)<<PORTB2);
    }
}

// Set direction of right motor
static void setRightMotorDirection(uint8_t dir) {
    // Set direction only when it is changeable or if it is stopped
    if (RIGHT_MOTOR_DIR_CHANGEABLE || dir == DIR_STOP) {
        // turn off right control pins and set to dir
        PORTB = (PORTB & ~((1<<PORTB1)|(1<<PORTB4))) | (((2 & dir)<<(PORTB1-1))|((1 & dir)<<PORTB4));
    }
}

// Set direction of robot
static void setDirection(uint8_t dir) {
    setLeftMotorDirection(dir >> 2);
    setRightMotorDirection(dir);
}

// Move robot forward
void moveForward(void) {
    setDirection(DIR_FORWARD);
}

// Move robot backward
void moveBackward(void) {
    setDirection(DIR_BACKWARD);
}

// Turn robot left
void rotateLeft(void) {
    setDirection(DIR_LEFT);
}

// Turn robot right
void rotateRight(void) {
    setDirection(DIR_RIGHT);
}

// Stop the robot
void stop(void) {
    setSpeed(0, 0);
    setDirection(DIR_STOP);
}

// Set speed of robot
void setSpeed(double speedL, double speedR) {
    setLeftDutyCycle(speedL);
    setRightDutyCycle(speedR);
    // Update PWM outside of setDutyCycle functions in order to minimize intermediate delay
    dutyUpdateFlag = 1;
}

// Set the duty cycle of the left motors
static void setLeftDutyCycle(double dutyPercent) {
    uint8_t duty;
    // Keep the duty percent within range
    if (dutyPercent >= MAX_SPEED) duty = MAX_SPEED*0xff;
    else if (dutyPercent <= MIN_SPEED) duty = 0;
    else duty = getDutyCycle(dutyPercent, 0xff); // Get the duty cycle based on the percentage
    // Keep the duty cycle within range
    if (duty > MAX_SPEED*0xff) duty = MAX_SPEED*0xff; // Max speed found through testing
    if (duty <= MIN_SPEED*0xff) { // Min speed found through testing
        duty = 0;
        setLeftMotorDirection(DIR_STOP);
    }
    // Set the duty cycle
    leftDutyCycle = duty;
}

// Turn the left motor on or off
// static void motorLeftSet(uint8_t val) {
//     PORTD = (PORTD & ~(1<<PORTD5)) | ((1 & val)<<PORTD5); // turn off left PWM pin and set to val
// }

// Set the duty cycle of the right motors
static void setRightDutyCycle(double dutyPercent) {
    uint8_t duty;
    if (dutyPercent >= MAX_SPEED) duty = MAX_SPEED*0xff;
    else if (dutyPercent <= MIN_SPEED) duty = 0;
    else duty = getDutyCycle(dutyPercent*1.05, 0xff);
    // Keep the duty cycle within range
    if (duty > MAX_SPEED*0xff) duty = MAX_SPEED*0xff; // Max speed found through testing
    if (duty <= MIN_SPEED*0xff) { // Min speed found through testing
        duty = 0;
        setRightMotorDirection(DIR_STOP);
    }
    // Set the duty cycle
    rightDutyCycle = duty;
}

// Turn the right motor on or off
// static void motorRightSet(uint8_t val) {
//     PORTD = (PORTD & ~(1<<PORTD6)) | ((1 & val)<<PORTD6); // turn off right PWM pin and set to val
// }
