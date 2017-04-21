#include "timer.h"

static volatile int timer0_overflows[MAX_TIMERS]; // Allows for about 17.8 minutes max timer
static uint8_t start_time[MAX_TIMERS];
static uint8_t countingFlag[MAX_TIMERS];
static volatile int delayTimer;
static volatile int sweepTimer;
static volatile int debounceTimer = 0;
static volatile int timeoutTimer = 0;
static volatile uint8_t connectionError = 1;

static void initTimer0_PWM_1kHz(void);

void timer_init(void) {
    // Start timer
    initTimer0_PWM_1kHz();

    // Clear software timer variables
    delayTimer = 0;
    sweepTimer = 0;
    for (int i = 0; i < MAX_TIMERS; i++) {
        start_time[i] = 0;
        timer0_overflows[i] = 0;
        countingFlag[i] = 0;
    }
}

static void initTimer0_PWM_1kHz(void) {
    // Set Timer0 to Fast PWM mode, top at 0xff
    TCCR0A |= (1<<WGM01)|(1<<WGM00);

    // Set OC0A/B outputs to non-inverting mode
    TCCR0A |= (1<<COM0A1)|(1<<COM0B1);

    OCR0B = 0;
    OCR0A = 0;

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
            connectionError = 1;
            BT_TimeoutTask();
        }
    }
    motor_OVFTask();
    for (int i = 0; i < MAX_TIMERS; i++)
        if (countingFlag[i] == 1)
            timer0_overflows[i]++;
}

void setDelay_ms(int delay) {
    delayTimer = delay;
}

uint8_t delay_done(void) {
    return delayTimer==0;
}

void setSweep_ms(int time) {
    sweepTimer = time;
}

uint8_t sweepReady(void) {
    return (sweepTimer==0 ? 1 : 0);
}

void setDebounce_ms(int time) {
    debounceTimer = time;
}

uint8_t debounceReady(void) {
    return (debounceTimer==0 ? 1 : 0);
}

void setBTTimeout_ms(int time) {
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
