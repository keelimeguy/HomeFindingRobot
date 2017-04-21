#include "motor.h"

static uint8_t leftDutyCycle, rightDutyCycle;
static volatile uint8_t dutyUpdateFlag = 1;

static void setLeftMotorDirection(uint8_t dir);
static void setRightMotorDirection(uint8_t dir);
static uint8_t getDutyCycle(double dutyPercent, uint8_t max);
static void setDirection(uint8_t dir);
static void setLeftDutyCycle(double dutyPercent);
// static void motorLeftSet(uint8_t val); // unused
static void setRightDutyCycle(double dutyPercent);
// static void motorRightSet(uint8_t val); // unused

void motor_init(void) {
    // Motor setup
    DDRB |= (1<<DDB2)|(1<<DDB1)|(1<<DDB4); // Sets B4 (N2), B1 (N1), B2 (N4) as output pins
    DDRD |= (1<<DDD5)|(1<<DDD6)|(1<<DDD7); // Sets D5 (ENB), D6 (ENA), D7 (N3) as output pins

    // Start at 0 duty
    leftDutyCycle = 0;
    rightDutyCycle = 0;
}

void motor_OVFTask(void) {
    if (dutyUpdateFlag == 1) {
        // Update duty cycles at top of waveform
        OCR0B = leftDutyCycle;
        OCR0A = rightDutyCycle;
        dutyUpdateFlag = 0;
    }
}

static uint8_t getDutyCycle(double dutyPercent, uint8_t max) {
    int duty = (int)(dutyPercent * (double)(max)); // dutyPercent = [duty] / [max];

    // Keep the duty cycle in bound
    if (duty < 0) duty = 0;
    else if (duty > max) duty = max;

    return (uint8_t) duty;
}

static void setLeftMotorDirection(uint8_t dir) {
    if (LEFT_MOTOR_DIR_CHANGEABLE || dir == DIR_STOP) {
        // turn off left control pins and set to dir
        PORTD = (PORTD & ~(1<<PORTD7)) | ((2 & dir)<<(PORTD7-1));
        PORTB = (PORTB & ~(1<<PORTB2)) | ((1 & dir)<<PORTB2);
    }
}

static void setRightMotorDirection(uint8_t dir) {
    if (RIGHT_MOTOR_DIR_CHANGEABLE || dir == DIR_STOP) {
        // turn off right control pins and set to dir
        PORTB = (PORTB & ~((1<<PORTB1)|(1<<PORTB4))) | (((2 & dir)<<(PORTB1-1))|((1 & dir)<<PORTB4));
    }
}

static void setDirection(uint8_t dir) {
    setLeftMotorDirection(dir >> 2);
    setRightMotorDirection(dir);
}

void moveForward(void) {
    setDirection(DIR_FORWARD);
}

void moveBackward(void) {
    setDirection(DIR_BACKWARD);
}

void rotateLeft(void) {
    setDirection(DIR_LEFT);
}

void rotateRight(void) {
    setDirection(DIR_RIGHT);
}

void stop(void) {
    setSpeed(0, 0);
    setDirection(DIR_STOP);
}

void setSpeed(double speedL, double speedR) {
    setLeftDutyCycle(speedL);
    setRightDutyCycle(speedR);
    // Update PWM outside of setDutyCycle functions in order to minimize intermediate delay
    dutyUpdateFlag = 1;
}

static void setLeftDutyCycle(double dutyPercent) {
    uint8_t duty;
    if (dutyPercent >= MAX_SPEED) duty = MAX_SPEED*0xff;
    else if (dutyPercent <= MIN_SPEED) duty = 0;
    else duty = getDutyCycle(dutyPercent, 0xff);
    if (duty > MAX_SPEED*0xff) duty = MAX_SPEED*0xff; // Max speed found through testing
    if (duty <= MIN_SPEED*0xff) { // Min speed found through testing
        duty = 0;
        setLeftMotorDirection(DIR_STOP);
    }
    leftDutyCycle = duty;
}

// static void motorLeftSet(uint8_t val) {
//     PORTD = (PORTD & ~(1<<PORTD5)) | ((1 & val)<<PORTD5); // turn off left PWM pin and set to val
// }

static void setRightDutyCycle(double dutyPercent) {
    uint8_t duty;
    if (dutyPercent >= MAX_SPEED) duty = MAX_SPEED*0xff;
    else if (dutyPercent <= MIN_SPEED) duty = 0;
    else duty = getDutyCycle(dutyPercent*1.05, 0xff);
    if (duty > MAX_SPEED*0xff) duty = MAX_SPEED*0xff; // Max speed found through testing
    if (duty <= MIN_SPEED*0xff) { // Min speed found through testing
        duty = 0;
        setRightMotorDirection(DIR_STOP);
    }
    rightDutyCycle = duty;
}

// static void motorRightSet(uint8_t val) {
//     PORTD = (PORTD & ~(1<<PORTD6)) | ((1 & val)<<PORTD6); // turn off right PWM pin and set to val
// }
