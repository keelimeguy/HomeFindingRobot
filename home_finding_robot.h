/*
 * HomeFindingRobot: home_finding_robot.h
 * Author : Keelin, Martial
 */

#ifndef ___HOME_FINDING_ROBOT___
#define ___HOME_FINDING_ROBOT___

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

void moveForward(void);
void moveBackwards(void);
void rotateRight(void);
void rotateLeft(void);
void stop(void);

#endif
