#include "servo.h"

// Duty cycle (angle) of servo
static uint8_t servoDutyCycle;

// State variables and flags for sweep functionality (moving servo back and forth between given angles)
static uint8_t sweepEnabled = 0;
static int sweepDelay = 0, sweepDirection = 1;
static double sweepMin = 0, sweepMax = 0, sweepAngle = 0, sweepStep = 0;
static double sweepDistance[SERVO_MAX_DISTANCE_BUFFER];
static uint8_t first = 2;
static uint8_t sweepDistanceCount = 0;
static int sweepMinIndex = 0;
static int sweepDistanceIndex = 0;

static void initTimer2_PWM_50Hz(void);
static double getServoPercent(double angle);

// Initialize the servo library
void servo_init(void) {
    // Servo setup
    DDRD |= (1<<DDD3); // Sets D3 (SIG) as output pin
    // PWM Setup
    initTimer2_PWM_50Hz();
}

/* ------------------------------------------------------------
                            PWM
   ------------------------------------------------------------ */

// Setupup timer 2 for PWM mode at 50Hz
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

// ISR for timer2 compare A match
ISR(TIMER2_COMPA_vect) {
    // Turn off overflow interrupt enable
    TIMSK2 &= ~(1<<OCIE2A);
    // Update duty cycle at top of waveform
    OCR2B = servoDutyCycle;
}

/* ------------------------------------------------------------
                           Servo
   ------------------------------------------------------------ */

// Convert a given angle to the percent duty cycle of the servo
static double getServoPercent(double angle) {
    // Calculate the pulse based on percentage within specific min/max range
    double pulse = (angle+90.0)/180.0*(SERVO_MAX-SERVO_MIN)+SERVO_MIN; // ms

    // Keep the servo pulse in bound
    if (pulse < SERVO_MIN) pulse = SERVO_MIN;
    else if (pulse > SERVO_MAX) pulse = SERVO_MAX;

    // Return the fraction of the servo pulse over the max servo period
    return pulse/SERVO_PERIOD;
}

// Set the angle of the servo
void setServoAngle(double angle) {
    // Set the current sweep angle to this angle
    sweepAngle = angle;

    // Updtae the duty cycle based on the angle and associated duty percent
    servoDutyCycle = getServoPercent(angle)*OCR2A;
    TIMSK2 |= (1<<OCIE2A); // Set compare A interrupt enable to update duty cycle
}

// Calculate the distance read by the servo, taking note of minimum distance found within a sweep
double servoSweepDistanceTask(void) {
    // Only sweep if enabled
    if (sweepEnabled) {
        // Use simulated software timer to minimize costly execution delays
        if (sweepReady()) { // Continue if timer ready (timer.c)
            // Increment the angle of the servo
            sweepAngle+=sweepStep*sweepDirection;

            // If angle reaches max, set to max and switch direction
            if (sweepAngle >= sweepMax) {
                sweepAngle = sweepMax;
                sweepDirection = -sweepDirection;
            }

            // If angle reaches min, set to min and switch direction
            if (sweepAngle <= sweepMin) {
                sweepAngle = sweepMin;
                sweepDirection = -sweepDirection;
                // Keep track of first (and second) time reaching this point
                if (first > 1)
                    first = 1;
                else if (first == 1) first = 0;
                // Reset the index into the distance readings
                sweepDistanceIndex = 0;
            }
            if (first == 1) { // Increment number of readings if first
                sweepDistanceCount++;
            }
            // Record recent distance, by reading directly from ultrasonic sensor
            sweepDistance[sweepDistanceIndex] = getObstacleDistanceCm(); // (ultrasonic_sensor.c)
            if (sweepMinIndex == sweepDistanceIndex) { // If we reach a minimum index
                sweepMinIndex = 0; // Reset minimum index to zero
                for (int i = 0 ; i < sweepDistanceCount; i++) // For each measured distance
                    // If that distance is less than minimum, then set it as the new minimum
                    if (sweepDistance[i] < sweepDistance[sweepMinIndex])
                        sweepMinIndex = i;
            // Otherwise no need to check all values, only see if recent measurement is less than minimum
            } else if (sweepDistance[sweepDistanceIndex] < sweepDistance[sweepMinIndex])
                // If that distance is less than minimum, then set it as the new minimum
                sweepMinIndex = sweepDistanceIndex;
            sweepDistanceIndex++; // Increment index into distance measurement buffer
            // If we reach a maximum size into the buffer, then loop back to zero
            if (sweepDistanceIndex >= SERVO_MAX_DISTANCE_BUFFER) sweepDistanceIndex = 0;
            // Reset sweep delay timer
            setSweep_ms(sweepDelay); // (timer.c)
            setServoAngle(sweepAngle); //Update the servo angle
        }
        return sweepDistance[sweepMinIndex]; // Return the minimum distance
    } else
        // If not sweeping, just return the distance mesured directly from the ultrasonic sensor
        return getObstacleDistanceCm(); // (ultrasonic_sensor.c)
}

// Setup the servo sweep variables
void setServoSweep(double minAngle, double maxAngle, double angleStep, int timeDelay) {
    // Set the minimum angle in the sweep
    sweepMin = minAngle;
    // Set the maximum angle in the sweep
    sweepMax = maxAngle;
    // Set the change in the angle at each update
    sweepStep = angleStep;
    // Set the delay between each update
    sweepDelay = timeDelay;
    // Clear the sweep delay timer (to indicate we should update sweep when enabled)
    setSweep_ms(0); // (timer.c)
}

// Turn on sweep
void servoSweepOn(void) {
    sweepEnabled = 1;
}

// Turn off sweep
void servoSweepOff(void) {
    sweepEnabled = 0;
    // Reset variables involved in finding minimum distance of sweep
    first = 2;
    sweepDistanceCount = 0;
    sweepMinIndex = 0;
    sweepDistanceIndex = 0;
}
