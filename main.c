/*
 * HomeFindingRobot: main.c
 *
 * Created: 2/26/2017 6:20:40 PM
 * Author : Keelin, Martial
 */

#include "home_finding_robot.h"

int main(void) {
    while(1) {
        rotateRight();
        _delay_ms(1000);
        stop();
        _delay_ms(1000);
        rotateLeft();
        _delay_ms(1000);
        stop();
        _delay_ms(1000);
    }
}
