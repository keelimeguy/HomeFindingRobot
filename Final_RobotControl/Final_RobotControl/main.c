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

void robot_control_and_avoid_task(void);
void robot_autonomous_avoid_task(void);
void robot_autonomous_home_task(void);

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

    // To convert double to string for displaying
    static char outstr[15];

    double lineVal;
    double correction;
    char stat, x, y;
    uint8_t stopped = 0, straight = 1;
    float yaw, pitch, roll;

    double baseSpeed = 0, rightBias = 0, leftBias = 0;
    int bugState = BUG_FORWARD_0;
    double bugDist;

    uint8_t robotMode = ROBOT_MODE_CONTROL_AND_AVOID;

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
        lineVal = readLineSensors();

        switch(robotMode) {
            case ROBOT_MODE_CONTROL_AND_AVOID:
                robot_control_and_avoid_task();
                break;

            case ROBOT_MODE_AUTONOMOUS_AVOID:
                robot_autonomous_avoid_task();
                break;

            case ROBOT_MODE_AUTONOMOUS_HOME:
                robot_autonomous_home_task();
                break;

            default:
                stop()
        }
    }
}

void robot_control_and_avoid_task(void) {
    if (servoSweepDistanceTask() < 30 && stopped == 0) {
        stop();
        stopped = 1;
    }
    if ((BT_has_pkt()&2)!=0) PORTB|=1<<PORTB5; // If packet overflow, turn on LED
    if (((BT_has_pkt()&1)==1)) { // If a packet is received
        BT_packet_t pkt = BT_get_pkt(); // Get the received packet

        // Update motor speeds for Joystick type data
        if (pkt.pkt_type == PKT_STOP) {
            stop();
            servoSweepOff();
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 2;
        } else if (pkt.pkt_type == PKT_JOYSTICK_Y && pkt.pkt_len==2) {
            uint16_t val = pkt.pkt_val[0];
            val |= pkt.pkt_val[1]<<8;
            if (val>=JOY_Y_MAX) {
                if (straight == 0) {
                    straight = 1;
                    start_counting(0);
                }
                baseSpeed = -MAX_SPEED;
            } else if (val<=JOY_Y_MIN) {
                if (straight == 0) {
                    straight = 1;
                    start_counting(0);
                }
                baseSpeed = MAX_SPEED;
            } else if (val <= JOY_Y_NORM - JOY_DELTA) {
                baseSpeed = (double)(JOY_Y_NORM  - JOY_DELTA - val)/(double)(JOY_Y_NORM - JOY_DELTA - JOY_Y_MIN)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
            } else if (val >= JOY_Y_NORM + JOY_DELTA) {
                baseSpeed = (double)(val - JOY_Y_NORM - JOY_DELTA)/(double)(JOY_Y_MAX - JOY_Y_NORM - JOY_DELTA)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
                baseSpeed = -baseSpeed;
            } else {
                baseSpeed = 0.0;
            }
            if (straight) {
                if (correction > 0) {
                    rightBias = correction;
                    leftBias = 0.0;
                } else if (correction < 0) {
                    leftBias = -correction;
                    rightBias = 0.0;
                } else {
                    leftBias = 0.0;
                    rightBias = 0.0;
                }
            }
        } else if (pkt.pkt_type == PKT_JOYSTICK_X && pkt.pkt_len==2) {
            uint16_t val = pkt.pkt_val[0];
            val |= pkt.pkt_val[1]<<8;
            if (val>=JOY_X_MAX) {
                leftBias = MAX_SPEED;
                rightBias = 0;
                straight = 0;
            } else if (val<=JOY_X_MIN){
                rightBias = MAX_SPEED;
                leftBias = 0;
                straight = 0;
            } else if (val >= JOY_X_NORM + JOY_DELTA) {
                leftBias = (double)(val - JOY_X_NORM - JOY_DELTA)/(double)(JOY_X_MAX - JOY_X_NORM - JOY_DELTA)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
                rightBias = 0;
                straight = 0;
            } else if (val <= JOY_X_NORM - JOY_DELTA) {
                rightBias = (double)(JOY_X_NORM - JOY_DELTA - val)/(double)(JOY_X_NORM - JOY_DELTA - JOY_X_MIN)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
                leftBias = 0;
                straight = 0;
            } else  {
                if (straight == 0) {
                    straight = 1;
                    start_counting(0);
                }
                if (straight && correction > 0) {
                    rightBias = correction;
                    leftBias = 0.0;
                } else if (straight && correction < 0) {
                    leftBias = -correction;
                    rightBias = 0.0;
                } else {
                    rightBias = 0.0;
                    leftBias = 0.0;
                }
            }
        } else if (pkt.pkt_type == PKT_AUTO_MODE) {
            stop();
            setServoAngle(0);
            servoSweepOn();
            robotMode = ROBOT_MODE_AUTONOMOUS_AVOID;
            bugState = BUG_FORWARD_0;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
        } else if (pkt.pkt_type == PKT_CONTROL_MODE) {
            stop();
            setServoAngle(0);
            servoSweepOn();
            BT_send_pkt((BT_packet_t){PKT_STRING, 5, (uint8_t*)"begin"});
            robotMode = ROBOT_MODE_CONTROL_AND_AVOID;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
        } else if (pkt.pkt_type == PKT_SET_HOME) {
            stop();
            setPosition(0,0);
        } else if (pkt.pkt_type == PKT_GO_HOME) {
            stop();
            servoSweepOff();
            setServoAngle(0);
            robotMode = ROBOT_MODE_AUTONOMOUS_HOME;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
        }

        BT_send_pkt(pkt); // Respond by sending the received packet over bluetooth

        for (int i = 0; i < 15; i++)
            outstr[i] = ' ';
        dtostrf(yaw, 3, 0, outstr);
        BT_send_pkt((BT_packet_t){PKT_HEADING, 15, outstr});
        dtostrf(correction, 5, 0, outstr);
        BT_send_pkt((BT_packet_t){PKT_DRIFT, 15, outstr});
        BT_send_pkt((BT_packet_t){PKT_POSITION, 2, (uint8_t[2]){getPositionX(), getPositionY()}});
        int dist = (int)servoSweepDistanceTask();
        BT_send_pkt((BT_packet_t){PKT_DISTANCE, 2, (uint8_t[2]){(dist>>8) & 0xff, dist & 0xff}});
    }
    double dist = servoSweepDistanceTask();
    if (error() == 0 && stopped < 2) {
        if (stopped == 0 && dist < 40) {
            stop();
            stopped = 1;
            setDelay_ms(50);
        }
        if (error() == 0 && delay_done()) {
            if (dist >= 40) stopped = 0;
            if (baseSpeed < 0) {
                setSpeed(sqrt(baseSpeed*baseSpeed + leftBias*leftBias) - rightBias/2, sqrt(baseSpeed*baseSpeed + rightBias*rightBias) - leftBias/2);
                moveBackward();
            } else if (stopped == 0) {
                setSpeed(sqrt(baseSpeed*baseSpeed + leftBias*leftBias)  - rightBias/2, sqrt(baseSpeed*baseSpeed + rightBias*rightBias) - leftBias/2);
                moveForward();
            } else stop();
        }
    } else stop();
}

void robot_autonomous_avoid_task(void) {
    // Add correction to account for drift bias
    correction=-correction;
    if (correction > 0) {
        rightBias = correction;
        leftBias = 0;
    } else if (correction < 0) {
        leftBias = -correction;
        rightBias = 0;
    }

    if ((BT_has_pkt()&2)!=0) PORTB|=1<<PORTB5; // If packet overflow, turn on LED
    if (((BT_has_pkt()&1)==1)) { // If a packet is received
        BT_packet_t pkt = BT_get_pkt(); // Get the received packet

        // Update motor speeds for Joystick type data
        if (pkt.pkt_type == PKT_STOP) {
            servoSweepOff();
            stop();
            stopped = 2;
        } else if (pkt.pkt_type == PKT_AUTO_MODE) {
            stop();
            setServoAngle(0);
            servoSweepOn();
            robotMode = ROBOT_MODE_AUTONOMOUS_AVOID;
            bugState = BUG_FORWARD_0;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
        } else if (pkt.pkt_type == PKT_CONTROL_MODE) {
            stop();
            setServoAngle(0);
            servoSweepOn();
            BT_send_pkt((BT_packet_t){PKT_STRING, 5, (uint8_t*)"begin"});
            robotMode = ROBOT_MODE_CONTROL_AND_AVOID;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
        } else if (pkt.pkt_type == PKT_SET_HOME) {
            stop();
            setPosition(0,0);
        } else if (pkt.pkt_type == PKT_GO_HOME) {
            stop();
            servoSweepOff();
            setServoAngle(0);
            robotMode = ROBOT_MODE_AUTONOMOUS_HOME;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
        }

        BT_send_pkt(pkt); // Respond by sending the received packet over bluetooth

        for (int i = 0; i < 15; i++)
            outstr[i] = ' ';
        dtostrf(correction, 5, 0, outstr);
        BT_send_pkt((BT_packet_t){PKT_DRIFT, 15, outstr});
        for (int i = 0; i < 15; i++)
            outstr[i] = ' ';
        dtostrf(yaw, 3, 0, outstr);
        BT_send_pkt((BT_packet_t){PKT_HEADING, 15, outstr});
        BT_send_pkt((BT_packet_t){PKT_POSITION, 2, (uint8_t[2]){getPositionX(), getPositionY()}});
        int dist = (int)servoSweepDistanceTask();
        BT_send_pkt((BT_packet_t){PKT_DISTANCE, 2, (uint8_t[2]){(dist>>8) & 0xff, dist & 0xff}});
    }

    // Avoid obstacles by turning right when an object is too close
    // (Simple implementation, not perfect. Keep robot under supervision)
    if (error() == 0 && stopped < 2) {
        switch(bugState) {
            case BUG_FORWARD_0:
                if (delay_done() && servoSweepDistanceTask() <= 35) {
                    bugState = BUG_FORWARD_1;
                    stopped = 1;
                    stop();
                    setDelay_ms(150);
                } else if (delay_done() && servoSweepDistanceTask() > 35) {
                    setSpeed((MAX_SPEED-MIN_SPEED)*0.2 + MIN_SPEED + leftBias, (MAX_SPEED-MIN_SPEED)*0.2 + MIN_SPEED + rightBias);
                    moveForward();
                    stopped = 0;
                }
                break;

            case BUG_FORWARD_1:
                if (delay_done() && servoSweepDistanceTask() <= 20) {
                    bugState = BUG_CHECK_CORNER_1;
                    stopped = 1;
                    servoSweepOff();
                    setServoAngle(90);
                    stop();
                    setDelay_ms(300);
                } else if (delay_done() && servoSweepDistanceTask() > 20) {
                    setSpeed((MAX_SPEED-MIN_SPEED)*0.075 + MIN_SPEED, (MAX_SPEED-MIN_SPEED)*0.075 + MIN_SPEED);
                    moveForward();
                    stopped = 0;
                }
                break;

            case BUG_CHECK_CORNER_1:
                if (delay_done() && getObstacleDistanceCm() <= 50) {
                    setServoAngle(-15);
                    setDelay_ms(300);
                    bugState = BUG_TURN_AT_CORNER;
                } else if (delay_done() && getObstacleDistanceCm() > 50) {
                    bugState = BUG_CHECK_CORNER_2;
                    setServoAngle(60);
                    setDelay_ms(300);
                }
                break;

            case BUG_CHECK_CORNER_2:
                if (delay_done() && getObstacleDistanceCm() <= 50) {
                    setServoAngle(-30);
                    setDelay_ms(300);
                    bugState = BUG_TURN_AT_CORNER;
                } else if (delay_done() && getObstacleDistanceCm() > 50)
                    bugState = BUG_TURN_AT_WALL_0;
                break;

            case BUG_TURN_AT_CORNER:
                if (delay_done() && getObstacleDistanceCm() <= 50) {
                    setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                    rotateRight();
                    setDelay_ms(30);
                } else if (delay_done() && getObstacleDistanceCm() > 50) {
                    stop();
                    setServoAngle(60);
                    setDelay_ms(300);
                    bugState = BUG_TURN_AT_WALL_0;
                }
                break;

            case BUG_TURN_AT_WALL_0:
                if (delay_done() && getObstacleDistanceCm() > 50) {
                    setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                    rotateRight();
                    setDelay_ms(30);
                } else if (delay_done() && getObstacleDistanceCm() <= 50) {
                    stop();
                    bugDist = getObstacleDistanceCm();
                    setServoAngle(45);
                    setDelay_ms(300);
                    bugState = BUG_TURN_AT_WALL_1;
                }
                break;

            case BUG_TURN_AT_WALL_1:
                if(delay_done()) {
                    if (bugDist + 10 > getObstacleDistanceCm()) {
                        setServoAngle(90);
                        setDelay_ms(300);
                        bugState = BUG_TURN_AT_WALL_2;
                    } else {
                        bugState = BUG_FOLLOW_WALL;
                    }
                }
                break;

            case BUG_TURN_AT_WALL_2:
                if(delay_done()) {
                    setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                    rotateRight();
                    setDelay_ms(100);
                    bugState = BUG_TURN_AT_WALL_3;
                }
                break;

            case BUG_TURN_AT_WALL_3:
                if (delay_done()) {
                    stop();
                    bugDist = getObstacleDistanceCm();
                    setServoAngle(45);
                    setDelay_ms(300);
                    bugState = BUG_TURN_AT_WALL_1;
                }
                break;

            case BUG_FOLLOW_WALL:
                setServoAngle(0);
                servoSweepOn();
                setDelay_ms(300);
                bugState = BUG_FORWARD_0;
                break;
        }
    } else {
        stop();
        stopped = 2;
    }
}

void robot_autonomous_home_task(void){
    // Add correction to account for drift bias
    correction=-correction;
    if (correction > 0) {
        rightBias = correction;
        leftBias = 0;
    } else if (correction < 0) {
        leftBias = -correction;
        rightBias = 0;
    }

    if ((BT_has_pkt()&2)!=0) PORTB|=1<<PORTB5; // If packet overflow, turn on LED
    if (((BT_has_pkt()&1)==1)) { // If a packet is received
        BT_packet_t pkt = BT_get_pkt(); // Get the received packet

        // Update motor speeds for Joystick type data
        if (pkt.pkt_type == PKT_STOP) {
            stop();
            servoSweepOff();
            stopped = 2;
       } else if (pkt.pkt_type == PKT_AUTO_MODE) {
            stop();
            setServoAngle(0);
            servoSweepOn();
            robotMode = ROBOT_MODE_AUTONOMOUS_AVOID;
            bugState = BUG_FORWARD_0;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
        } else if (pkt.pkt_type == PKT_CONTROL_MODE) {
            stop();
            setServoAngle(0);
            servoSweepOn();
            BT_send_pkt((BT_packet_t){PKT_STRING, 5, (uint8_t*)"begin"});
            robotMode = ROBOT_MODE_CONTROL_AND_AVOID;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
        } else if (pkt.pkt_type == PKT_SET_HOME) {
            stop();
            setPosition(0,0);
        } else if (pkt.pkt_type == PKT_GO_HOME) {
            stop();
            servoSweepOff();
            setServoAngle(0);
            robotMode = ROBOT_MODE_AUTONOMOUS_HOME;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
        }
        BT_send_pkt(pkt); // Respond by sending the received packet over bluetooth
    } else {
        stop();
        stopped = 2;
    }

    stop();
}
