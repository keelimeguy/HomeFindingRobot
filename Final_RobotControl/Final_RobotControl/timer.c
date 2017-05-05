#include "timer.h"

// Variables to keep track of multiple simulated stopwatch timers (used to measure ellapsed time)
static volatile int timer0_overflows[MAX_TIMERS]; // Allows for about 17.8 minutes max timer
static uint8_t start_time[MAX_TIMERS]; // The time when the associated timer started
static uint8_t countingFlag[MAX_TIMERS]; // Indicates this timer is active

// Timer variables used by other libraries
static volatile int delayTimer;
static volatile int sweepTimer;
static volatile int debounceTimer;
static volatile int timeoutTimer;
static volatile uint8_t connectionError = 1;

static void initTimer0_PWM_1kHz(void);

// Initialize the timer
void timer_init(void) {
    // Start timer, timer0 PWM at 1kHZ frequency
    // (Timer0 also used for controlling left and right motor speeds by PWM)
    initTimer0_PWM_1kHz();

    // Clear software timer variables
    delayTimer = 0;
    sweepTimer = 0;
    debounceTimer = 0;
    timeoutTimer = 0;
    for (int i = 0; i < MAX_TIMERS; i++) {
        start_time[i] = 0;
        timer0_overflows[i] = 0;
        countingFlag[i] = 0;
    }
}

// Initialize timer 0 for PWM mode at 1kHz
static void initTimer0_PWM_1kHz(void) {
    // Set Timer0 to Fast PWM mode, top at 0xff
    TCCR0A |= (1<<WGM01)|(1<<WGM00);

    // Set OC0A/B outputs to non-inverting mode (left and right motor control)
    // (Timer0 also used for controlling left and right motor speeds by PWM)
    TCCR0A |= (1<<COM0A1)|(1<<COM0B1);

    OCR0B = 0;
    OCR0A = 0;

    TIMSK0 |= (1<<TOIE0); // Enable overflow interrupt

    // PWM freq of ~1kHz (64/16MHz = 4us, 4us*(255+1) = 1.024ms)
    TCCR0B |= (1<<CS01)|(1<<CS00); // Set prescalar to 64 and start clock
}

// Timer0 overflow interrupt
ISR(TIMER0_OVF_vect) {
    // Decrement software timers if they are positive
    if (delayTimer>0) delayTimer --;
    if (sweepTimer>0) sweepTimer--;
    if (debounceTimer>0) debounceTimer--;
    if (timeoutTimer > 0) {
        timeoutTimer--;
        if (timeoutTimer == 0) {
            // If timeout timer reaches 0, indicate bluetooth connection error
            connectionError = 1;
            BT_TimeoutTask(); // (BT.c)
        }
    }
    // Call task for overflowing motor timer
    // (Timer0 also used for controlling left and right motor speeds by PWM)
    motor_OVFTask(); // (motor.c)
    // CIncrement all active stopwatch timers
    for (int i = 0; i < MAX_TIMERS; i++)
        if (countingFlag[i] == 1)
            timer0_overflows[i]++;
}

// Set the delay timer
void setDelay_ms(int delay) {
    delayTimer = delay;
}

// Indicate that the delay is complete
uint8_t delay_done(void) {
    return delayTimer==0;
}

// Set the sweep delay timer
void setSweep_ms(int time) {
    sweepTimer = time;
}

// Indicate that the servo sweep is ready to continue
uint8_t sweepReady(void) {
    return (sweepTimer==0 ? 1 : 0);
}

// Set the sweep delay timer
void setDebounce_ms(int time) {
    debounceTimer = time;
}

// Indicate that the debounce is ready
uint8_t debounceReady(void) {
    return (debounceTimer==0 ? 1 : 0);
}

// Set the timeout timer for the bluetooth
void setBTTimeout_ms(int time) {
    connectionError = 0;
    timeoutTimer = time;
}

// Indicate that the bluetooth timed out
uint8_t error(void) {
    return connectionError;
}

// Start the indicated stopwatch timer
void start_counting(int index) {
    // Set the start time to the current time
    start_time[index] = TCNT0;
    // Reset the overflows
    timer0_overflows[index] = 0;
    // Set the timer as active (counting)
    countingFlag[index] = 1;
    // Retry if timer overflow occurred within execution of above
    if (TCNT0 < start_time[index] && timer0_overflows[index] == 0)
        start_counting(index);
}

// Get how much time has ellapsed on indicated stopwatch timer
unsigned long timer_ellapsed_micros(int index, uint8_t reset) {
    // Find the difference between now and the start time
    uint8_t now = TCNT0;
    if (start_time[index] <= now)
        now -= start_time[index];
    else { // Account for extra overflow if applicable
        now = (uint8_t) (0x0100 - (uint16_t)start_time[index] + (uint16_t)now);
        timer0_overflows[index]--;
    }
    // Add together the time difference and the time accounted by timer overflows
    unsigned long ret = (unsigned long)now*4UL+(unsigned long)timer0_overflows[index]*0x100UL*4UL;
    // Reset the timer if we indicate to do so in parameters
    if (reset) start_counting(index);
    else countingFlag[index] = 0; // Otherwise disable the timer
    // Return the ellapsed time
    return ret;
}
