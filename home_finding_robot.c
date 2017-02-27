/*
 * HomeFindingRobot: home_finding_robot.c
 * Author : Keelin, Martial
 */

#include "home_finding_robot.h"

void moveForward(void) {
    // Right motor
    DDRB |= (1<<DDB2)|(1<<DDB1)|(1<<DDB0); // Sets B0, B1, B2 as output pins
    PORTB |= (1<<PORTB2)|(1<<PORTB0); // Turn right motors on, forward
    PORTB &= ~(1<<PORTB1); // Turn B1 off
    // Left motor
    DDRD |= (1<<DDD5)|(1<<DDD6)|(1<<DDD7); // Sets D5, D6, D7 as output pins
    PORTD |= (1<<PORTD5)|(1<<PORTD6); // Turn left motors on, forward
    PORTD &= ~(1<<PORTD7); // Turn D7 off
}

void moveBackwards(void) {
    // Right motor
    DDRB |= (1<<DDB2)|(1<<DDB1)|(1<<DDB0); // Sets B0, B1, B2 as output pins
    PORTB |= (1<<PORTB2)|(1<<PORTB1); // Turn right motors on, reverse
    PORTB &= ~(1<<PORTB0); // Turn B0 off
    // Left motor
    DDRD |= (1<<DDD5)|(1<<DDD6)|(1<<DDD7); // Sets D5, D6, D7 as output pins
    PORTD |= (1<<PORTD5)|(1<<PORTD7); // Turn left motors on, reverse
    PORTD &= ~(1<<PORTD6); // Turn D6 off
}

void rotateRight(void) {
    // Right motor
    DDRB |= (1<<DDB2)|(1<<DDB1)|(1<<DDB0); // Sets B0, B1, B2 as output pins
    PORTB |= (1<<PORTB2)|(1<<PORTB1); // Turn right motors on, reverse
    PORTB &= ~(1<<PORTB0); // Turn B0 off
    // Left motor
    DDRD |= (1<<DDD5)|(1<<DDD6)|(1<<DDD7); // Sets D5, D6, D7 as output pins
    PORTD |= (1<<PORTD5)|(1<<PORTD6); // Turn left motors on, forward
    PORTD &= ~(1<<PORTD7); // Turn D7 off
}

void rotateLeft(void) {
    // Right motor
    DDRB |= (1<<DDB2)|(1<<DDB1)|(1<<DDB0); // Sets B0, B1, B2 as output pins
    PORTB |= (1<<PORTB2)|(1<<PORTB0); // Turn right motors on, forward
    PORTB &= ~(1<<PORTB1); // Turn B1 off
    // Left motor
    DDRD |= (1<<DDD5)|(1<<DDD6)|(1<<DDD7); // Sets D5, D6, D7 as output pins
    PORTD |= (1<<PORTD5)|(1<<PORTD7); // Turn left motors on, reverse
    PORTD &= ~(1<<PORTD6); // Turn D6 off
}

void stop(void) {
    PORTD &= ~(1<<PORTD5); // Turn left motors off
    PORTB &= ~(1<<PORTB2); // Turn right motors off
}
