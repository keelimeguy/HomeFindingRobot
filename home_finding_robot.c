/*
 * HomeFindingRobot: home_finding_robot.c
 * Author : Keelin, Martial
 */

#include "home_finding_robot.h"

static uint8_t leftDutyCycle, rightDutyCycle, servoDutyCycle;
static volatile uint8_t dutyUpdateFlag = 0;
static volatile double obstacle_distance_cm = 0;
// static volatile double obstacle_distance_inch = 0;
static volatile int timer0_overflows[MAX_TIMERS]; // Allows for about 17.8 minutes max timer
static uint8_t start_time[MAX_TIMERS];
static uint8_t countingFlag[MAX_TIMERS];
static volatile int delayTimer;
static volatile int sweepTimer;
static float yaw = 0, pitch = 0, roll = 0;
static uint8_t sweepEnabled = 0;
static int sweepDelay = 0, sweepDirection = 1;
static double sweepMin = 0, sweepMax = 0, sweepAngle = 0, sweepStep = 0;
static volatile int debounceTimer = 0;
static uint8_t pushState = PUSH_STATE_NOPUSH;
static uint8_t pushFlag_Debounce = 0;
static uint8_t pushHandledFlag = 0;
static volatile unsigned int Ain;
static volatile int timeoutTimer = 0;
static volatile uint8_t connectionError = 1;

static void initADC(void);
static void ADC_convert(uint8_t ADC_mux);
static double readADC(double Aref);
static void initTimer1_Capture_1s(void);
static void initTimer0_PWM_1kHz(void);
static void initTimer2_PWM_50Hz(void);
static void setLeftMotorDirection(uint8_t dir);
static void setRightMotorDirection(uint8_t dir);
static uint8_t getDutyCycle(double dutyPercent, uint8_t max);
static double getServoPercent(double angle);
static void setDirection(uint8_t dir);
static void setLeftDutyCycle(double dutyPercent);
// static void motorLeftSet(uint8_t val); // unused
static void setRightDutyCycle(double dutyPercent);
// static void motorRightSet(uint8_t val); // unused
static uint8_t switchDebounce(uint8_t switchPressed);

void setup(uint8_t calibrate) {
    // On board LED
    DDRB |= (1<<DDB5); // Sets B5 (LED) as output pin
    // Motor setup
    DDRB |= (1<<DDB2)|(1<<DDB1)|(1<<DDB4); // Sets B4 (N2), B1 (N1), B2 (N4) as output pins
    DDRD |= (1<<DDD5)|(1<<DDD6)|(1<<DDD7); // Sets D5 (ENB), D6 (ENA), D7 (N3) as output pins
    // Servo setup
    DDRD |= (1<<DDD3); // Sets D3 (SIG) as output pin
    // Ultrasonic Sensor setup
    DDRB &= ~(1<<PORTB0); // Set ECHO (PB0) as input
    DDRC |= (1<<PORTC1); // Set TRIG (PC1) as output
    // Line Sensor Setup
    DDRB &= ~(1<<DDB3); // Sets B3 (lLine) as input pin
    DDRD &= ~(1<<DDD2); // Sets D2 (rLine) as input pin
    DDRC &= ~(1<<DDC0); // Sets C0 (mLine) as input pin

    // Clear software timer variables
    delayTimer = 0;
    sweepTimer = 0;
    for (int i = 0; i < MAX_TIMERS; i++) {
        start_time[i] = 0;
        timer0_overflows[i] = 0;
        countingFlag[i] = 0;
    }

    // Timer Capture Setup
    initTimer1_Capture_1s();
    // Enable Pin Change Interrupt
    PCMSK0 |= (1<<PCINT0);
    PCICR |= (1<<PCIE0);

    // PWM setup
    initTimer0_PWM_1kHz(); // motors
    initTimer2_PWM_50Hz(); // servo

    // IMU setup
    mpu9250_init(calibrate);

    // Mouse odometer setup
    mouse_init();

    // ADC setup
    initADC();

    // Bluetooth setup
    BT_init();

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

static void initTimer0_PWM_1kHz(void) {
    // Set Timer0 to Fast PWM mode, top at 0xff
    TCCR0A |= (1<<WGM01)|(1<<WGM00);

    // Set OC0A/B outputs to non-inverting mode
    TCCR0A |= (1<<COM0A1)|(1<<COM0B1);

    leftDutyCycle = 0;
    rightDutyCycle = 0;
    OCR0B = leftDutyCycle;
    OCR0A = rightDutyCycle;

    TIMSK0 |= (1<<TOIE0);

    // PWM freq of ~1kHz (64/16MHz = 4us, 4us*(255+1) = 1.024ms)
    TCCR0B |= (1<<CS01)|(1<<CS00); // Set prescalar to 64 and start clock
}

ISR(TIMER0_OVF_vect) {
    if (delayTimer>0) delayTimer --;
    if (sweepTimer>0) sweepTimer--;
    if (debounceTimer>0) debounceTimer--;
    if (timeoutTimer > 0) {
        timeoutTimer--;
        if (timeoutTimer == 0) {
            UCSR0B &= ~(1<<RXCIE0); // Disable Rx interrupt
            stop();
            connectionError = 1;
            BT_prepare_for_pkt();
        }
    }
    for (int i = 0; i < MAX_TIMERS; i++)
        if (countingFlag[i] == 1)
            timer0_overflows[i]++;
    if (dutyUpdateFlag == 1) {
        // Update duty cycles at top of waveform
        OCR0B = leftDutyCycle;
        OCR0A = rightDutyCycle;
        dutyUpdateFlag = 0;
    }
}

void setBTTimeout(int time) {
    connectionError = 0;
    timeoutTimer = time;
}

uint8_t error(void) {
    return connectionError;
}

void start_counting(int index) {
    start_time[index] = TCNT0;
    timer0_overflows[index] = 0;
    countingFlag[index] = 1;
    // Retry if timer overflow occurred within execution of above
    if (TCNT0 < start_time[index] && timer0_overflows[index] == 0)
        start_counting(index);
}

unsigned long timer_ellapsed_micros(int index, uint8_t reset) {
    uint8_t now = TCNT0;
    if (start_time[index] <= now)
        now -= start_time[index];
    else {
        now = (uint8_t) (0x0100 - (uint16_t)start_time[index] + (uint16_t)now);
        timer0_overflows[index]--;
    }
    unsigned long ret = (unsigned long)now*4UL+(unsigned long)timer0_overflows[index]*0x100UL*4UL;
    if (reset) start_counting(index);
    else countingFlag[index] = 0;
    return ret;
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


void setDelay_ms(int delay) {
    delayTimer = delay;
}

uint8_t delay_done(void) {
    return (delayTimer==0 ? 1 : 0);
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
    else duty = getDutyCycle(dutyPercent, 0xff);
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
    sweepAngle = angle;
    servoDutyCycle = getServoPercent(angle)*OCR2A;
    TIMSK2 |= (1<<OCIE2A); // Set compare A interrupt enable to update
}

// Use software timer to minimize costly execution delays
void servoSweepTask(void) {
    if (sweepEnabled) {
        if (sweepTimer == 0) {
            sweepAngle+=sweepStep*sweepDirection;
            if (sweepAngle >= sweepMax) {
                sweepAngle = sweepMax;
                sweepDirection = -sweepDirection;
            }
            if (sweepAngle <= sweepMin) {
                sweepAngle = sweepMin;
                sweepDirection = -sweepDirection;
            }
            sweepTimer = sweepDelay;
            setServoAngle(sweepAngle);
        }
    }
}

void setServoSweep(double minAngle, double maxAngle, double angleStep, int timeDelay) {
    sweepMin = minAngle;
    sweepMax = maxAngle;
    sweepStep = angleStep;
    sweepDelay = timeDelay/16;
    sweepTimer = 0;
}

void servoSweepOn(void) {
    sweepEnabled = 1;
}

void servoSweepOff(void) {
    sweepEnabled = 0;
}

/* ------------------------------------------------------------
                      Ultrasonic Sensor
   ------------------------------------------------------------ */

double getObstacleDistanceCm(void) {
    return obstacle_distance_cm;
}

// double getObstacleDistanceInch(void) {
//     return obstacle_distance_inch;
// }

/* ------------------------------------------------------------
                            IMU
   ------------------------------------------------------------ */

void getHeading(float* Yaw, float* Pitch, float* Roll) {
    if (mpu9250_dataReady())
        mpu9250_readData(&yaw, &pitch, &roll);
    *Yaw = yaw;
    *Pitch = pitch;
    *Roll = roll;
}

/* ------------------------------------------------------------
                        Mouse Odometer
   ------------------------------------------------------------ */

void getDisplacement(char* stat, char* x, char* y) {
    mouse_pos(stat, x, y);
}

/* ---------------------------------------------------------
                        ADC setup
   --------------------------------------------------------- */

static void initADC(void) {
    ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Set prescalar 128 (128/16MHz * 13cycles = 104us wait time)
    ADCSRA |= (1<<ADEN); // Enable ADC
}

static void ADC_convert(uint8_t ADC_mux) {
    ADMUX = (1<<REFS0)|ADC_mux;
    ADCSRA |= (1<<ADSC); // Set ADSC to 1 to begin a conversion.
}

static double readADC(double Aref) {
    Ain = ADCL;
    Ain |= ADCH<<8;
    double val = (double)Ain/1024.0 * Aref;
    return val;
}

/* --------------------------------------------------------
                        Debounce setup
   -------------------------------------------------------- */

static uint8_t switchDebounce(uint8_t switchPressed) {
    switch (pushState) {
        case PUSH_STATE_NOPUSH:
            if(debounceTimer == 0) {
                if (switchPressed) pushState = PUSH_STATE_MAYBE;
                else pushState = PUSH_STATE_NOPUSH;
                debounceTimer = DEBOUNCE_WAIT;
            }
            break;
        case PUSH_STATE_MAYBE:
            if(debounceTimer == 0) {
                if (switchPressed) {
                    pushState = PUSH_STATE_PUSHED;
                    pushFlag_Debounce = 1;
                    pushHandledFlag = 0;
                } else {
                    pushState = PUSH_STATE_NOPUSH;
                    pushFlag_Debounce = 0;
                }
                debounceTimer = DEBOUNCE_WAIT;
            }
            break;
        case PUSH_STATE_MAYBE_FROM_PUSH:
            if(debounceTimer == 0) {
                if (switchPressed) {
                    pushState = PUSH_STATE_PUSHED;
                } else {
                    pushState = PUSH_STATE_NOPUSH;
                    pushFlag_Debounce = 0;
                }
                debounceTimer = DEBOUNCE_WAIT;
            }
            break;
        case PUSH_STATE_PUSHED:
            if(debounceTimer == 0) {
                if (switchPressed) pushState = PUSH_STATE_PUSHED;
                else pushState = PUSH_STATE_MAYBE_FROM_PUSH;
                debounceTimer = DEBOUNCE_WAIT;
            }
            break;
    }
    return pushFlag_Debounce;
}

/* ------------------------------------------------------------
                        Line Sensors
   ------------------------------------------------------------ */

double readLineSensors(void) {
    ADC_convert(ADC_MIDDLE_SENSOR);
    while(!ADC_COMPLETE);
    double middleVal = readADC(V_REF);

    // When directed towards a white surface, the voltage output will be lower than that on a black surface
    if (switchDebounce(middleVal > SENSOR_THRESHOLD) && !pushHandledFlag) {
        pushHandledFlag = 1;
        PORTB ^= (1<<PORTB5);
    }
    return middleVal;
}
