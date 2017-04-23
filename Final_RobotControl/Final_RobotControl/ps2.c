#include "ps2.h"

static void ps2_init(void);
static void ps2_golo(int pin);
static void ps2_gohi(int pin);
static void ps2_write(unsigned char data);
static unsigned char ps2_read(void);

static int drift = 0;
static int driftArr[DRIFT_MAX_SIZE];
static int driftSize = 0;
static int driftIndex = 0;
static int iSize = 0, iPos = 0;
static double errorProportional = 0, errorIntegral = 0, errorDerivative = 0;
static double sampleDelay = 0;
static double errorArr[I_SIZE_MAX];

static void ps2_init(void) {
    ps2_gohi(_ps2clk);
    ps2_gohi(_ps2data);
}

static void ps2_gohi(int pin) {
    _ps2ddr &= ~(1<<pin); // Set to input
    _ps2port |= 1<<pin; // Turn on
}

static void ps2_golo(int pin) {
    _ps2ddr |= 1<<pin; // Set to output
    _ps2port &= ~(1<<pin); // Turn off
}

static void ps2_write(unsigned char data) {
    unsigned char parity=1;

    cli();

    ps2_gohi(_ps2data);
    ps2_gohi(_ps2clk);
    _delay_us(300);
    ps2_golo(_ps2clk);
    _delay_us(300);
    ps2_golo(_ps2data);
    _delay_us(10);
    ps2_gohi(_ps2clk);

    // sei();

    while((_ps2pin & (1<<_ps2clk)) != 0);

    for (int i=0; i<8; i++){
        if(data&0x01) ps2_gohi(_ps2data);
        else ps2_golo(_ps2data);
        while((_ps2pin & (1<<_ps2clk)) == 0);
        while((_ps2pin & (1<<_ps2clk)) != 0);
        parity^=(data&0x01);
        data=data>>1;
    }

    if(parity) ps2_gohi(_ps2data);
    else ps2_golo(_ps2data);

    while((_ps2pin & (1<<_ps2clk)) == 0);
    while((_ps2pin & (1<<_ps2clk)) != 0);

    // cli();

    ps2_gohi(_ps2data);
    _delay_us(50);

    while((_ps2pin & (1<<_ps2clk)) != 0);
    while(((_ps2pin & (1<<_ps2clk)) == 0)||((_ps2pin & (1<<_ps2data)) == 0));

    ps2_golo(_ps2clk);

    sei();
}

static unsigned char ps2_read(void) {
    unsigned char data=0, bit=1;

    cli();

    ps2_gohi(_ps2clk);
    ps2_gohi(_ps2data);
    _delay_us(50);
    while((_ps2pin & (1<<_ps2clk))!=0);

    _delay_us(5);

    // sei();

    while((_ps2pin & (1<<_ps2clk))==0);

    for(int i=0; i<8; i++){
        while((_ps2pin & (1<<_ps2clk))!=0);
        if((_ps2pin & (1<<_ps2data))!=0) data|=bit;
        while((_ps2pin & (1<<_ps2clk))==0);
        bit=bit<<1;
    }

    while((_ps2pin & (1<<_ps2clk))!=0);
    while((_ps2pin & (1<<_ps2clk))==0);
    while((_ps2pin & (1<<_ps2clk))!=0);
    while((_ps2pin & (1<<_ps2clk))==0);

    ps2_golo(_ps2clk);

    sei();

    return data;
}

void mouse_init(void) {
    ps2_init();

    ps2_write(0xFF); // Reset
    for(int i=0; i<3; i++) ps2_read();

    ps2_write(0xF3); // Set sampling rate
    ps2_write(0x0A); // 10/s
    ps2_read();

    ps2_write(0xE7); // Set 2:1 scaling
    ps2_read();

    ps2_write(0xE8); // Set resolution
    ps2_write(0x00); // 1 count/mm
    ps2_read();

    ps2_write(0xF0); // Remote mode
    ps2_read();
    _delay_us(100);
}

void mouse_pos(char* stat, char* x, char* y) {
    ps2_write(0xEB);
    ps2_read();
    *stat = ps2_read();
    *x = ps2_read();
    *y = ps2_read();
}

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
