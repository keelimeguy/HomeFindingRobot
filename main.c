/*
 * HomeFindingRobot: main.c
 *
 * Created: 2/26/2017 6:20:40 PM
 * Author : Keelin, Martial
 */

#include "home_finding_robot.h"

int main(void) {
    setup();

    // Keep servo facing forward
    setServoAngle(0);

    // Blink the on-board LED to indicate startup
    // (Don't want to start moving immediately when turned on)
    _delay_ms(1000);
    PORTB|=(1<<PORTB5);
    _delay_ms(500);
    PORTB&=~(1<<PORTB5);
    _delay_ms(500);
    PORTB|=(1<<PORTB5);
    _delay_ms(500);
    PORTB&=~(1<<PORTB5);
    _delay_ms(500);
    PORTB|=(1<<PORTB5);
    _delay_ms(1000);
    PORTB&=~(1<<PORTB5);

    while(1) {
        // Avoid obstacles by turning right when an object is too close
        // (Simple implementation, not perfect. Keep robot under supervision)
        if (getDistanceCm() < 15) {
            stop();
            _delay_ms(50);
            setSpeed(MIN_SPEED+.01, MIN_SPEED+.01);
            rotateRight();
            _delay_ms(400);
            stop();
        } else {
            setSpeed(MIN_SPEED+.01, MIN_SPEED+.01);
            moveForward();
        }
    }
}
