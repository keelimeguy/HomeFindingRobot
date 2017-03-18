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
        setServoAngle(-90);
        rotateRight(MIN_SPEED+.01);
        _delay_ms(1500);
        setServoAngle(90);
        rotateRight((MIN_SPEED+.01+MAX_SPEED)*2.0/3.0);
        _delay_ms(500);
        setServoAngle(0);
        stop();
        _delay_ms(1000);
        setServoAngle(-45);
        rotateLeft(MAX_SPEED);
        _delay_ms(2000);
        stop();
        _delay_ms(1000);
        setServoAngle(45);
        moveForward(MIN_SPEED+.01);
        _delay_ms(500);
        stop();
        _delay_ms(500);
        setServoAngle(30);
        moveBackward(MIN_SPEED+.01);
        _delay_ms(500);
        setServoAngle(-30);
        stop();
        _delay_ms(1000);
    }
}
