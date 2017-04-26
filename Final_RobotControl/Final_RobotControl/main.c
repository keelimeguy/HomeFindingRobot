/*
 * Final_RobotControl.c
 *
 * Created: 4/3/2017 10:49:26 AM
 * Author : Keelin
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "robot.h"

static uint8_t straight = 1;
static uint8_t robotMode = ROBOT_MODE_CONTROL_AND_AVOID;
static char stat, x, y;
static float yaw, pitch, roll;
static double correction;

int main(void) {
    setup(0);

    // Start servo facing forward
    setServoAngle(0);

    // Blink the on-board LED to indicate startup
    // (Don't want to start moving immediately when turned on)
    _delay_ms(1000);
    for (int i = 0; i < 2; i ++) {
        PORTB|=(1<<PORTB5);
        _delay_ms(500);
        PORTB&=~(1<<PORTB5);
        _delay_ms(500);
    }
    PORTB|=(1<<PORTB5);
    _delay_ms(1000);
    PORTB&=~(1<<PORTB5);

    BT_send_pkt((BT_packet_t){PKT_STRING, 5, (uint8_t*)"begin"});
    BT_prepare_for_pkt();

    start_counting(0);
    driftCorrection(0, 0, 0, straight);

    setServoSweep(-30, 30, 5, 600);
    servoSweepOn();

    setDelay_ms(1000);

    while(1) {

        // Get mouse odometer position
        getDisplacement(&stat, &x, &y);
        correction = driftCorrection(stat, x, y, straight);

        // Get IMU orientation
        getHeading(&yaw, &pitch, &roll);

        // Read line sensor
        updatePosition();

        switch(robotMode) {
            case ROBOT_MODE_CONTROL_AND_AVOID:
                robotMode = robot_control_and_avoid_task(correction, yaw, &straight);
                break;

            case ROBOT_MODE_AUTONOMOUS_AVOID:
                robotMode = robot_autonomous_avoid_task(correction, yaw, &straight);
                break;

            case ROBOT_MODE_AUTONOMOUS_HOME:
                robotMode = robot_autonomous_home_task(correction, yaw, &straight);
                break;

            default:
                stop();
        }
    }
}
