#include "servo.h"

static uint8_t servoDutyCycle;
static uint8_t sweepEnabled = 0;
static int sweepDelay = 0, sweepDirection = 1;
static double sweepMin = 0, sweepMax = 0, sweepAngle = 0, sweepStep = 0;

static void initTimer2_PWM_50Hz(void);
static double getServoPercent(double angle);

void servo_init(void) {
    // Servo setup
    DDRD |= (1<<DDD3); // Sets D3 (SIG) as output pin
    // PWM Setup
    initTimer2_PWM_50Hz();
}

/* ------------------------------------------------------------
                            PWM
   ------------------------------------------------------------ */

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
// double sweepDistance = 0;
// double sweepDistance2 = 0;
// uint8_t sweepDistanceCount2 = 0;
double sweepDistance[50];
uint8_t first = 2;
uint8_t sweepDistanceCount = 0;
int sweepMinIndex = 0;
int sweepDistanceIndex = 0;
double servoSweepDistanceTask(void) {
    if (sweepEnabled) {
        if (sweepReady()) {
            sweepAngle+=sweepStep*sweepDirection;
            if (sweepAngle >= sweepMax) {
                sweepAngle = sweepMax;
                sweepDirection = -sweepDirection;
            }
            if (sweepAngle <= sweepMin) {
                sweepAngle = sweepMin;
                sweepDirection = -sweepDirection;
                if (first > 1)
                    first = 1;
                else if (first == 1) first = 0;
                sweepDistanceIndex = 0;
            }
            if (first == 1) {
                sweepDistanceCount++;
            }
            sweepDistance[sweepDistanceIndex] = getObstacleDistanceCm();
            if (sweepMinIndex == sweepDistanceIndex) {
                sweepMinIndex = 0;
                for (int i = 0 ; i < sweepDistanceCount; i++)
                    if (sweepDistance[i] < sweepDistance[sweepMinIndex])
                        sweepMinIndex = i;
            } else if (sweepDistance[sweepDistanceIndex] < sweepDistance[sweepMinIndex])
                sweepMinIndex = sweepDistanceIndex;
            sweepDistanceIndex++;
            if (sweepDistanceIndex >= 50) sweepDistanceIndex = 0;
            setSweep_ms(sweepDelay);
            setServoAngle(sweepAngle);
        }
        return sweepDistance[sweepMinIndex];
    } else
        return getObstacleDistanceCm();
}

void setServoSweep(double minAngle, double maxAngle, double angleStep, int timeDelay) {
    sweepMin = minAngle;
    sweepMax = maxAngle;
    sweepStep = angleStep;
    sweepDelay = timeDelay/16;
    setSweep_ms(0);
}

void servoSweepOn(void) {
    sweepEnabled = 1;
}

void servoSweepOff(void) {
    sweepEnabled = 0;
    first = 2;
    sweepDistanceCount = 0;
    sweepMinIndex = 0;
    sweepDistanceIndex = 0;
}
