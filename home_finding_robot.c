/*
 * HomeFindingRobot: home_finding_robot.c
 * Author : Keelin, Martial
 */

#include "home_finding_robot.h"

static uint8_t leftDutyCycle, rightDutyCycle, servoDutyCycle;

static void initTimer0_PWM_61Hz(void);
static void initTimer2_PWM_50Hz(void);
static void setLeftMotorDirection(uint8_t dir);
static void setRightMotorDirection(uint8_t dir);
static uint8_t getDutyCycle(double dutyPercent, uint8_t max);
static double getServoPercent(double angle);
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
    initTimer0_PWM_61Hz(); // motors
    initTimer2_PWM_50Hz(); // servo

    // Enable interrupts
    sei();
}

/* ------------------------------------------------------------
                            PWM
   ------------------------------------------------------------ */

static void initTimer0_PWM_61Hz(void) {
    // Set Timer0 to Fast PWM mode, top at 0xff
    TCCR0A |= (1<<WGM01)|(1<<WGM00);

    // Set OC0A/B outputs to non-inverting mode
    TCCR0A |= (1<<COM0A1)|(1<<COM0B1);

    leftDutyCycle = 0;
    rightDutyCycle = 0;
    OCR0B = leftDutyCycle;
    OCR0A = rightDutyCycle;

    // PWM freq of ~61Hz (1024/16MHz = 64us, 64us*(255+1) = 16.384ms)
    TCCR0B |= (1<<CS02)|(1<<CS00); // Set prescalar to 1024 and start clock
}

ISR(TIMER0_OVF_vect) {
    // Turn off overflow interrupt enable
    TIMSK0 &= ~(1<<TOIE0);
    // Update duty cycles at top of waveform
    OCR0B = leftDutyCycle;
    OCR0A = rightDutyCycle;
}

static void initTimer2_PWM_50Hz(void) {
    // Set Timer0 to Phase Correct PWM mode, top at OCR2A
    TCCR2B |= (1<<WGM22);
    TCCR2A |= (1<<WGM20);

    // Set OC2B output to non-inverting mode
    TCCR2A |= (1<<COM2B1);

    OCR2A = 157;
    servoDutyCycle = (uint8_t)(getServoPercent(0)*OCR2A); // Start servo at middle (0 degrees)
    OCR2B = servoDutyCycle;

    // PWM freq of ~50Hz (1024/16MHz = 64us, 64us*157*2 = 20.096ms)
    TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20); // Set prescalar to 1024 and start clock
}

ISR(TIMER2_COMPA_vect) {
    // Turn off overflow interrupt enable
    TIMSK2 &= ~(1<<OCIE2A);
    // Update duty cycle at top of waveform
    OCR2B = servoDutyCycle;
}

static uint8_t getDutyCycle(double dutyPercent, uint8_t max) {
    int duty = (int)(dutyPercent * (double)(max)); // dutyPercent = [duty] / [max];

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
        PORTD = (PORTD & ~(1<<PORTD7)) | ((2 & dir)<<(PORTD7-1));
        PORTB = (PORTB & ~(1<<PORTB2)) | ((1 & dir)<<PORTB2);
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
    // Update PWM outside of setDutyCycle functions in order to minimize intermediate delay
    TIMSK0 |= (1<<TOIE0); // Set overflow interrupt enable to update
}

void moveBackward(double speed) {
    setLeftDutyCycle(speed);
    setRightDutyCycle(speed);
    setDirection(DIR_BACKWARD);
    // Update PWM outside of setDutyCycle functions in order to minimize intermediate delay
    TIMSK0 |= (1<<TOIE0); // Set overflow interrupt enable to update
}

void rotateLeft(double speed) {
    setLeftDutyCycle(speed);
    setRightDutyCycle(speed);
    setDirection(DIR_LEFT);
    // Update PWM outside of setDutyCycle functions in order to minimize intermediate delay
    TIMSK0 |= (1<<TOIE0); // Set overflow interrupt enable to update
}

void rotateRight(double speed) {
    setLeftDutyCycle(speed);
    setRightDutyCycle(speed);
    setDirection(DIR_RIGHT);
    // Update PWM outside of setDutyCycle functions in order to minimize intermediate delay
    TIMSK0 |= (1<<TOIE0); // Set overflow interrupt enable to update
}

void stop(void) {
    setLeftDutyCycle(0);
    setRightDutyCycle(0);
    setDirection(DIR_STOP);
    // Update PWM outside of setDutyCycle functions in order to minimize intermediate delay
    TIMSK0 |= (1<<TOIE0); // Set overflow interrupt enable to update
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

static void motorLeftSet(uint8_t val) {
    PORTD = (PORTD & ~(1<<PORTD5)) | ((1 & val)<<PORTD5); // turn off left PWM pin and set to val
}

static void setRightDutyCycle(double dutyPercent) {
    uint8_t duty;
    if (dutyPercent >= MAX_SPEED) duty = MAX_SPEED*0xff;
    else if (dutyPercent <= MIN_SPEED) duty = 0;
    else duty = getDutyCycle(dutyPercent, 0xff);
    if (duty > MAX_SPEED*0xff) duty = MAX_SPEED*0xff; // Max speed found through testing
    if (duty <= MIN_SPEED*0xff) { // Min speed found through testing
        duty = 0;
        setRightMotorDirection(DIR_STOP);
    }
    rightDutyCycle = duty;
}

static void motorRightSet(uint8_t val) {
    PORTD = (PORTD & ~(1<<PORTD6)) | ((1 & val)<<PORTD6); // turn off right PWM pin and set to val
}

/* ------------------------------------------------------------
                           Servo
   ------------------------------------------------------------ */


static double getServoPercent(double angle) {
    double pulse = (angle+90.0)/180.0*(SERVO_MAX-SERVO_MIN)+SERVO_MIN; // ms

    // Keep the servo pulse in bound
    if (pulse < SERVO_MIN) pulse = SERVO_MIN;
    else if (pulse > SERVO_MAX) pulse = SERVO_MAX;

    return pulse/SERVO_PERIOD;
}

void setServoAngle(double angle) {
    servoDutyCycle = getServoPercent(angle)*OCR2A;
    TIMSK2 |= (1<<OCIE2A); // Set compare A interrupt enable to update
}
