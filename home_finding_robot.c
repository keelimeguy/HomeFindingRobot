/*
 * HomeFindingRobot: home_finding_robot.c
 * Author : Keelin, Martial
 */

#include "home_finding_robot.h"

static uint8_t leftDutyCycle, rightDutyCycle, servoDutyCycle;
static volatile double obstacle_distance_cm;
// static volatile double obstacle_distance_inch;

static void initTimer1_Capture_1s(void);
static void initTimer0_PWM_61Hz(void);
static void initTimer2_PWM_50Hz(void);
static void setLeftMotorDirection(uint8_t dir);
static void setRightMotorDirection(uint8_t dir);
static uint8_t getDutyCycle(double dutyPercent, uint8_t max);
static double getServoPercent(double angle);
static void setDirection(uint8_t dir);
static void setLeftDutyCycle(double dutyPercent);
static void motorLeftSet(uint8_t val); // unused
static void setRightDutyCycle(double dutyPercent);
static void motorRightSet(uint8_t val); // unused


void setup(void) {
    // On board LED
    DDRB |= (1<<DDB5); // Sets B5 as output pin
    // Motor setup
    DDRB |= (1<<DDB2)|(1<<DDB1)|(1<<DDB4); // Sets B4, B1, B2 as output pins
    DDRD |= (1<<DDD5)|(1<<DDD6)|(1<<DDD7); // Sets D5, D6, D7 as output pins
    // Servo setup
    DDRD |= (1<<DDD3); // Sets D3 as output pin
    // Ultrasonic Sensor setup
    DDRB &= ~(1<<PORTB0); // Set ECHO (PB0) as input
    DDRC |= (1<<PORTC1); // Set TRIG (PC1) as output
    obstacle_distance_cm = 0;
    // obstacle_distance_inch = 0;

    // Timer Capture Setup
    initTimer1_Capture_1s();
    // Enable Pin Change Interrupt
    PCMSK0 |= (1<<PCINT0);
    PCICR |= (1<<PCIE0);

    // PWM setup
    initTimer0_PWM_61Hz(); // motors
    initTimer2_PWM_50Hz(); // servo

    // Enable interrupts
    sei();
}

/* ------------------------------------------------------------
                        Timer Capture
   ------------------------------------------------------------ */

static void initTimer1_Capture_1s(void) {
    // Set to overflow timer at 1s
    TCCR1B |= (1<<WGM12); // Set to CTC mode, clear timer on compare match at OCR1A
    TCCR1B |= (1<<CS12);  // Set pre-scalar to divide by 256
    OCR1A = 62499;        // Compare match A after 1s ( 256*(62499+1)/16Mhz = 1s )
    // Enable timer1 capture and overflow interrupts
    TIMSK1 |= (1<<ICIE1)|(1<<OCIE1A);
    // Captures on negative edge of ICP1 (PB0)
}

ISR(PCINT0_vect) {
    if ((PINB & (1<<PINB0)) != 0) { // Rising edge
        TCNT1 = 0;
        PCICR &= ~(1<<PCIE0); // Disable interrupt
    } else {
        // distance invalid
    }
}

ISR(TIMER1_CAPT_vect) {
    // Capture obstacle distance
    double time = ICR1;
    time *= 16; // one tick = 256/16MHz = 16us
    obstacle_distance_cm = time/58; // d(cm) = t(us)/58
    // obstacle_distance_inch = time/148; // d(inch) = t(us)/148

    PCICR |= (1<<PCIE0); // Enable pin change interrupt
    // trigger next pulse
    PORTC |= (1<<PORTC1); // turn on trigger
    _delay_us(10); // hold trigger for 10us
    PORTC &= ~(1<<PORTC1); // turn off trigger
}

ISR(TIMER1_COMPA_vect) {
    PCICR |= (1<<PCIE0); // Enable pin change interrupt
    // trigger next pulse
    PORTC |= (1<<PORTC1); // turn on trigger
    _delay_us(10); // hold trigger for 10us
    PORTC &= ~(1<<PORTC1); // turn off trigger
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

/* ------------------------------------------------------------
                      Ultrasonic Sensor
   ------------------------------------------------------------ */

double getDistanceCm(void) {
    return obstacle_distance_cm;
}

// double getDistanceInch(void) {
//     return obstacle_distance_inch;
// }
