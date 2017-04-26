#include "line_sensor.h"

static int direction = DIRECTION_UP, positionX = 0, positionY = 0;
static uint8_t pushState = PUSH_STATE_NOPUSH;
static uint8_t pushFlag_Debounce = 0;
static uint8_t pushHandledFlag = 0;
static volatile unsigned int Ain;

static void initADC(void);
static void ADC_convert(uint8_t ADC_mux);
static double readADC(double Aref);
static uint8_t switchDebounce(uint8_t switchPressed);

int getIntPositionX() {
    return positionX;
}

int getIntPositionY() {
    return positionY;
}

uint8_t getPositionX() {
    int posX = positionX;
    uint8_t ret = 0;
    if (positionX<0) {
        ret = 0x80;
        posX = -positionX;
    }
    if (posX>0x7f) posX = 0x7f;
    ret |= posX&0x7f;
    return ret;
}

uint8_t getPositionY() {
    int posY = positionY;
    uint8_t ret = 0;
    if (positionY<0) {
        ret = 0x80;
        posY = -positionY;
    }
    if (posY>0x7f) posY = 0x7f;
    ret |= posY&0x7f;
    return ret;
}

void setPosition(int x, int y) {
    positionX = x;
    positionY = y;
}

void setDirection(int dir) {
    direction = dir;
}

int getDirection() {
    return direction;
}

void updateDirection(int dir) {
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

void line_sensor_init(void) {
    initADC();
}

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

static uint8_t switchDebounce(uint8_t switchPressed) {
    switch (pushState) {
        case PUSH_STATE_NOPUSH:
            if(debounceReady()) {
                if (switchPressed) pushState = PUSH_STATE_MAYBE;
                else pushState = PUSH_STATE_NOPUSH;
                setDebounce_ms(DEBOUNCE_WAIT);
            }
            break;
        case PUSH_STATE_MAYBE:
            if(debounceReady()) {
                if (switchPressed) {
                    pushState = PUSH_STATE_PUSHED;
                    pushFlag_Debounce = 1;
                    pushHandledFlag = 0;
                } else {
                    pushState = PUSH_STATE_NOPUSH;
                    pushFlag_Debounce = 0;
                }
                setDebounce_ms(DEBOUNCE_WAIT);
            }
            break;
        case PUSH_STATE_MAYBE_FROM_PUSH:
            if(debounceReady()) {
                if (switchPressed) {
                    pushState = PUSH_STATE_PUSHED;
                } else {
                    pushState = PUSH_STATE_NOPUSH;
                    pushFlag_Debounce = 0;
                }
                setDebounce_ms(DEBOUNCE_WAIT);
            }
            break;
        case PUSH_STATE_PUSHED:
            if(debounceReady()) {
                if (switchPressed) pushState = PUSH_STATE_PUSHED;
                else pushState = PUSH_STATE_MAYBE_FROM_PUSH;
                setDebounce_ms(DEBOUNCE_WAIT);
            }
            break;
    }
    return pushFlag_Debounce;
}

double updatePosition(void) {
    ADC_convert(ADC_MIDDLE_SENSOR);
    while(!ADC_COMPLETE);
    double middleVal = readADC(V_REF);

    // When directed towards a white surface, the voltage output will be lower than that on a black surface
    if (switchDebounce(middleVal > SENSOR_THRESHOLD) && !pushHandledFlag) {
        pushHandledFlag = 1;
        PORTB ^= (1<<PORTB5);
        switch(direction) {
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

            default:
                positionX = 0;
                positionY = 0;
        }
    }
    return middleVal;
}
