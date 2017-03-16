/*
 * HomeFindingRobot: main.c
 *
 * Created: 2/26/2017 6:20:40 PM
 * Author : Keelin, Martial
 */

#include "home_finding_robot.h"

int main(void) {
    setup();
    while(1) {
        rotateRight(MIN_SPEED+.01);
        _delay_ms(1500);
        rotateRight(MIN_SPEED+.01+MAX_SPEED/2);
        _delay_ms(500);
        stop();
        _delay_ms(1000);
        rotateLeft(MAX_SPEED);
        _delay_ms(2000);
        stop();
        _delay_ms(1000);
        moveForward(MIN_SPEED+.01);
        _delay_ms(500);
        stop();
        _delay_ms(500);
        moveBackward(MIN_SPEED+.01);
        _delay_ms(500);
        stop();
        _delay_ms(1000);
    }
}
