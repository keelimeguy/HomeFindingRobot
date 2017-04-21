#include "robot.h"

static uint8_t pushState = PUSH_STATE_NOPUSH;
static uint8_t pushFlag_Debounce = 0;
static uint8_t pushHandledFlag = 0;
static int direction = DIRECTION_UP, positionX = 0, positionY = 0;
static volatile unsigned int Ain;
static float yaw, pitch, roll;

static void initADC(void);
static void ADC_convert(uint8_t ADC_mux);
static double readADC(double Aref);
static uint8_t switchDebounce(uint8_t switchPressed);

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

void setup(uint8_t calibrate) {
     // On board LED
    DDRB |= (1<<DDB5); // Sets B5 (LED) as output pin

    // Timer setup (timer0)
    timer_init();
    // Motor setup
    motor_init();
    // Servo setup (timer2)
    servo_init();
    // Ultrasonic Sensor setup (timer1)
    ultrasonic_sensor_init();

    // Line Sensor Setup
    DDRB &= ~(1<<DDB3); // Sets B3 (lLine) as input pin
    DDRD &= ~(1<<DDD2); // Sets D2 (rLine) as input pin
    DDRC &= ~(1<<DDC0); // Sets C0 (mLine) as input pin

    // Bluetooth setup
    BT_init();
    // IMU setup
    mpu9250_init(calibrate);
    // Mouse odometer setup
    mouse_init();
    // ADC setup
    initADC();

    // Enable interrupts
    sei();
}

/* ------------------------------------------------------------
                    PID Straight Control
   ------------------------------------------------------------ */
static int drift = 0;
static int driftArr[DRIFT_MAX_SIZE];
static int driftSize = 0;
static int driftIndex = 0;
static int iSize = 0, iPos = 0;
static double errorProportional = 0, errorIntegral = 0, errorDerivative = 0;
static double sampleDelay = 0;
static double errorArr[I_SIZE_MAX];
double driftCorrection(char stat, char x, char y, uint8_t straight) {
    // stat & (1<<4) => x sign
    // stat & (1<<6) => x overflow
    // PID Control to move straight
    if (straight) {
        if ((stat & (1<<6)) != 0) {
            if (stat & (1<<4))
                x = -127;
            else
                x = 127;
        }
        drift*=driftSize;
        if (driftSize>= DRIFT_MAX_SIZE)
            drift -= driftArr[driftIndex];
        if (stat & (1<<4)) {
            x = (x^0xff) + 1;
            if (x > 10)
                driftArr[driftIndex] = -x;
        } else if (x > 10) driftArr[driftIndex] = x;
        if (driftSize < DRIFT_MAX_SIZE)
            driftSize++;
        drift += driftArr[driftIndex];
        drift /= driftSize;
        driftIndex++;
        if (driftIndex>=DRIFT_MAX_SIZE) driftIndex = 0;
    } else {
        driftSize = 0;
        drift = 0;
    }

    // Perform PID control to keep robot straight
    sampleDelay = (double)(timer_ellapsed_micros(0, 1)/1000000.0);
    if (iSize < I_SIZE_MAX){
        errorIntegral += (double)drift;
        iSize++;
    } else {
        errorIntegral += ((double)drift-errorArr[iPos]);
    }
    errorArr[iPos] = (double)drift;
    iPos++;
    if (iPos>=I_SIZE_MAX) iPos = 0;
    errorDerivative = (drift - errorProportional)/sampleDelay;
    errorProportional = drift;
    return KBIAS*(KP*errorProportional + KI*errorIntegral*sampleDelay + KD*errorDerivative);
}

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
