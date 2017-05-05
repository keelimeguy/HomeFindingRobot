

// Modified for personal project by Keelin Wheeler in March 2017:
//  This code is converted to C from an arduino library at http://playground.arduino.cc/ComponentLib/Ps2mouse
//  The details of this code need not be discussed or commented in much detail, except for the driftCorrection() function.
//    What the code does is get readings from the PS/2 Mouse, acting as an odometer, and also manages the PID control for side to side drift


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

// Calculates the drift correction according to a PID controller on the error in side to side drift
double driftCorrection(char stat, char x, char y, uint8_t straight) {
    // stat & (1<<4) => x sign
    // stat & (1<<6) => x overflow

    // PID Control only when moving straight
    if (straight) {
        if ((stat & (1<<6)) != 0) { // If overflow
            if (stat & (1<<4)) // If negative
                x = -127; // Set to minimum value
            else
                x = 127; // Set to maimum value
        }
        // Multiply to recover accumulated drift, i,e, undo the average [ drift = sum(x) / driftSize ]
        drift*=driftSize;
        if (driftSize>= DRIFT_MAX_SIZE)
            // If we exceed size limit, subtract earliest measurement from sum
            drift -= driftArr[driftIndex];

        if (stat & (1<<4)) { // If negative
            x = (x^0xff) + 1; // Convert to postitive magnitude by 2's compliment
            if (x > 10) // Only consider errors with magnitude greater than 10
                driftArr[driftIndex] = -x; // Add negative error to list
        } else if (x > 10) driftArr[driftIndex] = x;  // Add negative error to list if greater than 10
        // Increase the size if we didn't reach the limit
        if (driftSize < DRIFT_MAX_SIZE)
            driftSize++;
        // Add current measurement to sum accumulated drift
        drift += driftArr[driftIndex];
        // Take the average of all measured drifts
        drift /= driftSize;
        driftIndex++; // Increase the index into the buffer of measurements
        // If index exceeds max size of buffer, loop back to zero
        if (driftIndex>=DRIFT_MAX_SIZE) driftIndex = 0;
    } else {
        // We are not going straight, so let's take this time to reset the PID measurements (to zero)
        driftSize = 0;
        drift = 0;
    }

    // Perform PID control to keep robot straight
    // Calculate delay between measurement samples by reading from simulated timer 0, flag to reset timer as well
    sampleDelay = (double)(timer_ellapsed_micros(0, 1)/1000000.0); // (timer.c)
    // Calculate the integral term in the PID equation
    if (iSize < I_SIZE_MAX){ // If we have not reached the limit on the integral buffer
        errorIntegral += (double)drift; // Add the current drift to the integral buffer
        iSize++; // Update size of data in integral buffer
    } else {
        // Max size reached, so replace earliest measured drift (by subtracting it out) with latest drift (by adding it in)
        errorIntegral += ((double)drift-errorArr[iPos]);
    }
    // Update buffer of drift values
    errorArr[iPos] = (double)drift;
    iPos++; // Increment index into error buffer
    if (iPos>=I_SIZE_MAX) iPos = 0; // If index exceeds max, loop back to zero
    // Calculate the derivative term in the PID equation
    errorDerivative = (drift - errorProportional)/sampleDelay;
    // Calculate the proportional term in the PID equation (equal to current drift error)
    errorProportional = drift;

    // Return the correction (the biased sum of weighted terms)
    return KBIAS*(KP*errorProportional + KI*errorIntegral*sampleDelay + KD*errorDerivative);
}
