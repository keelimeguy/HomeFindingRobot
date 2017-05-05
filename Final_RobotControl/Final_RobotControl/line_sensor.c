#include "line_sensor.h"

// Stored position and orientation of robot
static int direction = DIRECTION_UP, positionX = 0, positionY = 0;

// Debounce variables for the line sensor debounce
static uint8_t pushState = PUSH_STATE_NOPUSH;
static uint8_t pushFlag_Debounce = 0;
static uint8_t pushHandledFlag = 0;

// Store the measured line sensor ADC value
static volatile unsigned int Ain;

static void initADC(void);
static void ADC_convert(uint8_t ADC_mux);
static double readADC(double Aref);
static uint8_t switchDebounce(uint8_t switchPressed);

// Return X position as pure signed int
int getIntPositionX() {
    return positionX;
}

// Return Y position as pure signed int
int getIntPositionY() {
    return positionY;
}

// Return X position in signed magnitude form
uint8_t getPositionX() {
    int posX = positionX;
    uint8_t ret = 0;
    // Set first bit to 1 if negative
    if (positionX<0) {
        ret = 0x80;
        posX = -positionX;
    }
    // If position exceeds max value, set to the max value
    if (posX>0x7f) posX = 0x7f;
    // Set the magnitude
    ret |= posX&0x7f;
    return ret;
}

// Return Y position in signed magnitude form
uint8_t getPositionY() {
    int posY = positionY;
    uint8_t ret = 0;
    // Set first bit to 1 if negative
    if (positionY<0) {
        ret = 0x80;
        posY = -positionY;
    }
    // If position exceeds max value, set to the max value
    if (posY>0x7f) posY = 0x7f;
    // Set the magnitude
    ret |= posY&0x7f;
    return ret;
}

// Set the position of the robot
void setPosition(int x, int y) {
    positionX = x;
    positionY = y;
}

// Set the direction of the robot
void setDirection(int dir) {
    direction = dir;
}

// Get the direction of the robot
int getDirection() {
    return direction;
}

// Update the direction of the robot by turning in the direction given
void updateDirection(int dir) {
    // Update direction for turning right case.
    if (dir == DIRECTION_RIGHT) {
        switch (direction) {
            case DIRECTION_UP:
                direction = DIRECTION_RIGHT;
                break;

            case DIRECTION_RIGHT:
                direction = DIRECTION_DOWN;
                break;

            case DIRECTION_DOWN:
                direction = DIRECTION_LEFT;
                break;

            case DIRECTION_LEFT:
                direction = DIRECTION_UP;
                break;
        }
    }
    // Update direction for turning left case.
    else if (dir == DIRECTION_LEFT) {
        switch (direction) {
            case DIRECTION_UP:
                direction = DIRECTION_LEFT;
                break;

            case DIRECTION_LEFT:
                direction = DIRECTION_DOWN;
                break;

            case DIRECTION_DOWN:
                direction = DIRECTION_RIGHT;
                break;

            case DIRECTION_RIGHT:
                direction = DIRECTION_UP;
                break;
        }
    }
}

// Initialize the line senosr library
void line_sensor_init(void) {
    initADC(); // init the ADC
}

// Initialize the ADC
static void initADC(void) {
    ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Set prescalar 128 (128/16MHz * 13cycles = 104us wait time)
    ADCSRA |= (1<<ADEN); // Enable ADC
}

// Start measuring the value through ADC at the given multiplexed input
static void ADC_convert(uint8_t ADC_mux) {
    ADMUX = (1<<REFS0)|ADC_mux;
    ADCSRA |= (1<<ADSC); // Set ADSC to 1 to begin a conversion.
}

// Convert the measured ADC value to a value based on the given reference voltage
static double readADC(double Aref) {
    Ain = ADCL;
    Ain |= ADCH<<8;
    double val = (double)Ain/1024.0 * Aref;
    return val;
}

// Debounce for given switch value
static uint8_t switchDebounce(uint8_t switchPressed) {
    switch (pushState) {
        case PUSH_STATE_NOPUSH:
            // Always check if debounce wait time is over (timer.c)
            if(debounceReady()) {
                if (switchPressed) pushState = PUSH_STATE_MAYBE;
                else pushState = PUSH_STATE_NOPUSH;
                // Reset the debounce wait time (timer.c)
                setDebounce_ms(DEBOUNCE_WAIT);
            }
            break;
        case PUSH_STATE_MAYBE:
            // Always check if debounce wait time is over (timer.c)
            if(debounceReady()) {
                if (switchPressed) {
                    pushState = PUSH_STATE_PUSHED;
                    pushFlag_Debounce = 1;
                    pushHandledFlag = 0;
                } else {
                    pushState = PUSH_STATE_NOPUSH;
                    pushFlag_Debounce = 0;
                }
                // Reset the debounce wait time (timer.c)
                setDebounce_ms(DEBOUNCE_WAIT);
            }
            break;
        case PUSH_STATE_MAYBE_FROM_PUSH:
            // Always check if debounce wait time is over (timer.c)
            if(debounceReady()) {
                if (switchPressed) {
                    pushState = PUSH_STATE_PUSHED;
                } else {
                    pushState = PUSH_STATE_NOPUSH;
                    pushFlag_Debounce = 0;
                }
                // Reset the debounce wait time (timer.c)
                setDebounce_ms(DEBOUNCE_WAIT);
            }
            break;
        case PUSH_STATE_PUSHED:
            // Always check if debounce wait time is over (timer.c)
            if(debounceReady()) {
                if (switchPressed) pushState = PUSH_STATE_PUSHED;
                else pushState = PUSH_STATE_MAYBE_FROM_PUSH;
                // Reset the debounce wait time (timer.c)
                setDebounce_ms(DEBOUNCE_WAIT);
            }
            break;
    }
    return pushFlag_Debounce;
}

// Update the position based on value read by line sensor
double updatePosition(void) {
    // Read middle line sensor value
    ADC_convert(ADC_MIDDLE_SENSOR);
    while(!ADC_COMPLETE); // Wait to convert, blocking
    double middleVal = readADC(V_REF); // Read the value

    // When directed towards a white surface, the voltage output will be lower than that on a black surface
    // Debounce the value and handle case where we read a line, if not already handled
    if (switchDebounce(middleVal > SENSOR_THRESHOLD) && !pushHandledFlag) { // Debounce the measured value
        // Indicate that this line transition was handled
        pushHandledFlag = 1;
        PORTB ^= (1<<PORTB5); // Prompt user with LED toggle
        switch(direction) { // Update position according to direction robot is facing
            case DIRECTION_UP:
                positionY++;
                break;

            case DIRECTION_DOWN:
                positionY--;
                break;

            case DIRECTION_LEFT:
                positionX--;
                break;

            case DIRECTION_RIGHT:
                positionX++;
                break;

            default: // Reset position if invalid direction
                positionX = 0;
                positionY = 0;
        }
    }
    // Return the line sensor value for inspection
    return middleVal;
}
