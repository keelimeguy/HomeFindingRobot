/*
 * HomeFindingRobot: main.c
 *
 * Created: 2/26/2017 6:20:40 PM
 * Author : Keelin, Martial
 */

#include "home_finding_robot.h"

int main(void) {
    setup();

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

    uint8_t stopped = 0;
    char stat, x, y;
    float yaw, pitch, roll;

    while(1) {
        // Get mouse odometer position
        getDisplacement(&stat, &x, &y);
        // Get IMU orientation
        getHeading(&yaw, &pitch, &roll);

        // Avoid obstacles by turning right when an object is too close
        // (Simple implementation, not perfect. Keep robot under supervision)
        if (getObstacleDistanceCm() <= 25) {
            setDelay_ms(0);
            stopped = 1;
            setSpeed(MIN_SPEED+.01, MIN_SPEED+.01);
            rotateRight();
        } else if (stopped == 1 && delay_done()) { // Rotate right until object not seen, then extra period of rotation as overhead
            setDelay_ms(320);
            stopped = 0;
        } else if (delay_done() == 1) {
            setSpeed(MIN_SPEED+.01, MIN_SPEED+.01);
            moveForward();
            stopped = 0;
        }
    }
}
