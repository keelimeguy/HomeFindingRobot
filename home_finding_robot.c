/*
 * HomeFindingRobot: home_finding_robot.c
 * Author : Keelin, Martial
 */

#include "home_finding_robot.h"

static uint8_t leftDutyCycle, rightDutyCycle;
static uint8_t dutyLeftUpdateFlag, dutyRightUpdateFlag;

static void initTimer0_PWM_10kHz(void);
static void initTimer2_PWM_10kHz(void);
static void setLeftMotorDirection(uint8_t dir);
static void setRightMotorDirection(uint8_t dir);
static uint8_t getDutyCycle(double dutyPercent, uint8_t max);
static void setDirection(uint8_t dir);
static void setLeftDutyCycle(double dutyPercent);
static void motorLeftSet(uint8_t val);
static void setRightDutyCycle(double dutyPercent);
static void motorRightSet(uint8_t val);


void setup(void) {
    // On board LED
    DDRB |= (1<<DDB5); // Sets B5 as output pin
    // Motor setup
    DDRB |= (1<<DDB2)|(1<<DDB1)|(1<<DDB0); // Sets B0, B1, B2 as output pins
    DDRD |= (1<<DDD5)|(1<<DDD6)|(1<<DDD7); // Sets D5, D6, D7 as output pins
    // Servo setup
    DDRD |= (1<<DDD3); // Sets D3 as output pin

    // PWM setup
    initTimer0_PWM_10kHz();
    initTimer2_PWM_10kHz();
    dutyLeftUpdateFlag = 0;
    dutyRightUpdateFlag = 0;

    // Enable interrupts
    sei();
}

/* ------------------------------------------------------------
                            PWM
   ------------------------------------------------------------ */

static void initTimer0_PWM_10kHz(void) {
    // Set Timer0 to Fast PWM mode
    TCCR0B |= (1<<WGM02);
    TCCR0A |= (1<<WGM01)|(1<<WGM00);

    TIMSK0 |= (1<<OCIE0A)|(1<<OCIE0B); // Set compare interrupt enables

    OCR0A = 199; // PWM freq of 10kHz (8/16MHz = 500ns, 500ns*(199+1) = .1ms)
    leftDutyCycle = MIN_SPEED*OCR0A; // Start at minimum duty cycle
    OCR0B = leftDutyCycle;

    TCCR0B |= (1<<CS01); // Set prescalar to 8 and start clock
}

ISR(TIMER0_COMPA_vect) {
    // Turn left motor PWM pin on
    motorLeftSet(1);

    // Set duty cycle to control speed of left motor
    if (dutyLeftUpdateFlag) {
        OCR0B = leftDutyCycle;
        dutyLeftUpdateFlag = 0;
    }
}

ISR(TIMER0_COMPB_vect) {
    // Turn left motor PWM pin off
    motorLeftSet(0);
}

static void initTimer2_PWM_10kHz(void) {
    // Set Timer2 to Fast PWM mode
    TCCR2B |= (1<<WGM22);
    TCCR2A |= (1<<WGM21)|(1<<WGM20);

    TIMSK2 |= (1<<OCIE2A)|(1<<OCIE2B); // Set compare interrupt enables

    OCR2A = 199; // PWM freq of 10kHz (8/16MHz = 500ns, 500ns*(199+1) = .1ms)
    rightDutyCycle = MIN_SPEED*OCR2A; // Start at minimum duty cycle
    OCR2B = rightDutyCycle;

    TCCR2B |= (1<<CS21); // Set prescalar to 8 and start clock
}

ISR(TIMER2_COMPA_vect) {
    // Turn right motor PWM pin on
    motorRightSet(1);
    PORTB|=(1<<PORTB5);
    // Set duty cycle to control speed of right motor
    if (dutyRightUpdateFlag) {
        OCR2B = rightDutyCycle;
        dutyRightUpdateFlag = 0;
    }
}

ISR(TIMER2_COMPB_vect) {
    // Turn right motor PWM pin off
    motorRightSet(0);
    PORTB&=~(1<<PORTB5);
}

static uint8_t getDutyCycle(double dutyPercent, uint8_t max) {
    int duty = (int)(dutyPercent * (double)(max + 1.0) - 1.0); // dutyPercent = OCRB + 1 / OCRA=[max] + 1;

    // Keep the duty cycle in bound
    if (duty < 0) duty = 0;
    else if (duty > max) duty = max;

    return (uint8_t) duty;
}

/* ------------------------------------------------------------
                           Motors
   ------------------------------------------------------------ */

static void setLeftMotorDirection(uint8_t dir) {
    if (LEFT_MOTOR_DIR_CHANGEABLE || dir == DIR_STOP) {
        // turn off left control pins and set to dir
        PORTD = (PORTD & ~((1<<PORTD7)|(1<<PORTD6))) | (((2 & dir)<<(PORTD7-1))|((1 & dir)<<PORTD6));
    }
}

static void setRightMotorDirection(uint8_t dir) {
    if (RIGHT_MOTOR_DIR_CHANGEABLE || dir == DIR_STOP) {
        // turn off right control pins and set to dir
        PORTB = (PORTB & ~((1<<PORTB1)|(1<<PORTB0))) | (((2 & dir)<<(PORTB1-1))|((1 & dir)<<PORTB0));
    }
}

static void setDirection(uint8_t dir) {
    setLeftMotorDirection(dir >> 2);
    setRightMotorDirection(dir);
}

void moveForward(double speed) {
    setLeftDutyCycle(speed);
    setRightDutyCycle(speed);
    setDirection(DIR_FORWARD);
    // Update flags outside of setDutyCycle functions in order to minimize intermediate delay
    dutyLeftUpdateFlag = 1;
    dutyRightUpdateFlag = 1;
}

void moveBackward(double speed) {
    setLeftDutyCycle(speed);
    setRightDutyCycle(speed);
    setDirection(DIR_BACKWARD);
    // Update flags outside of setDutyCycle functions in order to minimize intermediate delay
    dutyLeftUpdateFlag = 1;
    dutyRightUpdateFlag = 1;
}

void rotateLeft(double speed) {
    setLeftDutyCycle(speed);
    setRightDutyCycle(speed);
    setDirection(DIR_LEFT);
    // Update flags outside of setDutyCycle functions in order to minimize intermediate delay
    dutyLeftUpdateFlag = 1;
    dutyRightUpdateFlag = 1;
}

void rotateRight(double speed) {
    setLeftDutyCycle(speed);
    setRightDutyCycle(speed);
    setDirection(DIR_RIGHT);
    // Update flags outside of setDutyCycle functions in order to minimize intermediate delay
    dutyLeftUpdateFlag = 1;
    dutyRightUpdateFlag = 1;
}

void stop(void) {
    setLeftDutyCycle(0);
    setRightDutyCycle(0);
    setDirection(DIR_STOP);
    // Update flags outside of setDutyCycle functions in order to minimize intermediate delay
    dutyLeftUpdateFlag = 1;
    dutyRightUpdateFlag = 1;
}

static void setLeftDutyCycle(double dutyPercent) {
    uint8_t duty;
    if (dutyPercent >= MAX_SPEED) duty = MAX_SPEED*OCR0A;
    else if (dutyPercent <= MIN_SPEED) duty = 0;
    else duty = getDutyCycle(dutyPercent, OCR0A);
    if (duty > MAX_SPEED*OCR0A) duty = MAX_SPEED*OCR0A; // Max speed found through testing
    if (duty <= MIN_SPEED*OCR0A) { // Min speed found through testing
        duty = 0;
        setLeftMotorDirection(DIR_STOP);
    }
    leftDutyCycle = duty;
}

static void motorLeftSet(uint8_t val) {
    PORTD = (PORTD & ~(1<<PORTD5)) | ((1 & val)<<PORTD5); // turn off left PWM pin and set to val
}

static void setRightDutyCycle(double dutyPercent) {
    uint8_t duty;
    if (dutyPercent >= MAX_SPEED) duty = MAX_SPEED*OCR2A;
    else if (dutyPercent <= MIN_SPEED) duty = 0;
    else duty = getDutyCycle(dutyPercent, OCR2A);
    if (duty > MAX_SPEED*OCR2A) duty = MAX_SPEED*OCR2A; // Max speed found through testing
    if (duty <= MIN_SPEED*OCR2A) { // Min speed found through testing
        duty = 0;
        setRightMotorDirection(DIR_STOP);
    }
    rightDutyCycle = duty;
}

static void motorRightSet(uint8_t val) {
    PORTB = (PORTB & ~(1<<PORTB2)) | ((1 & val)<<PORTB2); // turn off right PWM pin and set to val
}
