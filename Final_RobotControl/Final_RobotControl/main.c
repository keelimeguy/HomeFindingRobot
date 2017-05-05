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

static uint8_t straight = 1; // Flag to indicate if robot is moving straight
static uint8_t robotMode = ROBOT_MODE_CONTROL_AND_AVOID; // State of robot
static char stat, x, y; // Mouse sensor values
static float yaw, pitch, roll; // IMU orientation values
static double correction; // PID correction variable for mouse x value (side to side drift)

int main(void) {
    // Setup the robot, pass a 1 to calibrate the IMU
    setup(0); // (robot.c)

    // Start servo facing forward
    setServoAngle(0); // (servo.c)

    // Blink the on-board LED to indicate startup
    // (Don't want to start moving immediately when turned on)
    _delay_ms(1000);
    // Blink twice
    for (int i = 0; i < 2; i ++) {
        PORTB|=(1<<PORTB5);
        _delay_ms(500);
        PORTB&=~(1<<PORTB5);
        _delay_ms(500);
    }
    // Blink once more slightly longer
    PORTB|=(1<<PORTB5);
    _delay_ms(1000);
    PORTB&=~(1<<PORTB5);

    // Send begin packet over bluetooth
    BT_send_pkt((BT_packet_t){PKT_STRING, 5, (uint8_t*)"begin"}); // (BT.c)
    BT_prepare_for_pkt(); // Prepare to receive over bluetooth, (BT.c)

    // Start the simulated timer 0
    start_counting(0); // (timer.c)

    // Initiate drift PID by calling with zero parameter
    driftCorrection(0, 0, 0, straight);// (ps2.c)

    // Start servo sweeping with given parameters
    setServoSweep(-30, 30, 5, 600/16); // (servo.c)
    servoSweepOn(); // (servo.c)

    setDelay_ms(1000); // Wait a bit more before starting

    while(1) {

        // Get mouse odometer position
        getDisplacement(&stat, &x, &y); // (robot.c)
        // Calculate drift PID correction given mouse displacement info
        correction = driftCorrection(stat, x, y, straight); // (ps2.c)

        // Get IMU orientation
        getHeading(&yaw, &pitch, &roll); // (robot.c)

        // Read line sensor and update position when robot crosses a line
        updatePosition(); // (line_sensor.c)

        // Control robot based on mode
        switch(robotMode) {
            case ROBOT_MODE_CONTROL_AND_AVOID:
                // The remote controlled mode
                robotMode = robot_control_and_avoid_task(correction, yaw, &straight); // (robot.c)
                break;

            case ROBOT_MODE_AUTONOMOUS_AVOID:
                // The obstacle avoiding mode (tries to moves parallel to obstacles it meets)
                robotMode = robot_autonomous_avoid_task(correction, yaw, &straight); // (robot.c)
                break;

            case ROBOT_MODE_AUTONOMOUS_HOME:
                // The autonomous home returning mode (requires coordinate system on ground made up of black taped lines)
                robotMode = robot_autonomous_home_task(correction, yaw, &straight); // (robot.c)
                break;

            default:
                // Stop the robot if invalid mode reached
                stop(); // (motor.c)
        }

        // Read line sensor and update position when robot crosses a line
        updatePosition(); // (line_sensor.c)
    }
}
