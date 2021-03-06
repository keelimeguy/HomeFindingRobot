#include "ultrasonic_sensor.h"

// Variable to stroe the measured distance in specified unit
static volatile double obstacle_distance_cm;
// static volatile double obstacle_distance_inch;

static void initTimer1_Capture_1s(void);

// Initialize the ultrasonic sensor library
void ultrasonic_sensor_init(void) {
    // Ultrasonic Sensor setup
    DDRB &= ~(1<<PORTB0); // Set ECHO (PB0) as input
    DDRC |= (1<<PORTC1); // Set TRIG (PC1) as output
    obstacle_distance_cm = 0; // Reser the distance to zero

    // Timer Capture Setup
    initTimer1_Capture_1s();
    // Enable pin change interrupt
    PCMSK0 |= (1<<PCINT0);
    PCICR |= (1<<PCIE0);
}

/* ------------------------------------------------------------
                        Timer Capture
   ------------------------------------------------------------ */

// Setup timer 1 for interrupt capture at 1s overflow
static void initTimer1_Capture_1s(void) {
    // Set to overflow timer at 1s
    TCCR1B |= (1<<WGM12); // Set to CTC mode, clear timer on compare match at OCR1A
    TCCR1B |= (1<<CS12);  // Set pre-scalar to divide by 256
    OCR1A = 62499;        // Compare match A after 1s ( 256*(62499+1)/16Mhz = 1s )
    // Enable timer1 capture and overflow interrupts
    TIMSK1 |= (1<<ICIE1)|(1<<OCIE1A);
    // Captures on negative edge of ICP1 (PB0)
}

// Pin change interrupt (PB0)
ISR(PCINT0_vect) {
    if ((PINB & (1<<PINB0)) != 0) { // Rising edge
        TCNT1 = 0; // Reset time
        PCICR &= ~(1<<PCIE0); // Disable interrupt
    } else {
        // distance invalid
    }
}

// Timer capture interrupt (falling edge)
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

// Timer compare A match interrupt
ISR(TIMER1_COMPA_vect) {
    PCICR |= (1<<PCIE0); // Enable pin change interrupt
    // trigger next pulse
    PORTC |= (1<<PORTC1); // turn on trigger
    _delay_us(10); // hold trigger for 10us
    PORTC &= ~(1<<PORTC1); // turn off trigger
}

/* ------------------------------------------------------------
                      Ultrasonic Sensor
   ------------------------------------------------------------ */

// Get the distance in cm
double getObstacleDistanceCm(void) {
    return obstacle_distance_cm;
}

// // Get the distance in inches
// double getObstacleDistanceInch(void) {
//     return obstacle_distance_inch;
// }
