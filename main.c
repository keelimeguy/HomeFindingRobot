/*
 * HomeFindingRobot: main.c
 *
 * Created: 2/26/2017 6:20:40 PM
 * Author : Keelin, Martial
 */

#include "home_finding_robot.h"
#include <stdlib.h>
// #include "uart.h"

// File stream for UART. Used for Transmission to demonstrate the fprintf function.
// FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int main(void) {
    // uart_init();
    // stdout = stdin = stderr = &uart_str; // Set File outputs to point to UART stream
    // printf("Hello.\n");

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

    int iSize = 0;
    double lineVal;
    double correction;
    double errorProportional = 0, errorIntegral = 0, errorDerivative = 0;
    double sampleDelay = 0;
    char stat, x, y;
    int drift = 0;
    uint8_t stopped = 0, straight = 1;
    float yaw, pitch, roll;

    double baseSpeed = 0, rightBias = 0, leftBias = 0;

    // uint8_t robotMode = ROBOT_MODE_CONTROL_AND_AVOID;
    uint8_t robotMode = ROBOT_MODE_AUTONOMOUS_AVOID;

    BT_send_pkt((BT_packet_t){PKT_STRING, 5, (uint8_t*)"begin"});
    BT_prepare_for_pkt();
    start_counting(0);

    while(1) {

        // Get mouse odometer position
        getDisplacement(&stat, &x, &y);
        // printf("x: %d, y: %d\t", x, y);
        // stat & (1<<4) => x sign
        // stat & (1<<6) => x overflow
        // PID Control to move straight
        if (straight) {
            if ((stat & (1<<6)) != 0)
                x = 127;
            if ((stat & (1<<4)) == 0)
                drift += x;
            else
                drift -= ((x^0xff)+1);
        }

        // Perform PID control to keep robot straight
        sampleDelay = (double)(timer_ellapsed_micros(0, 1)/1000000.0);
        // if(iSize < I_SIZE_MAX){
        //     errorIntegral += drift*sampleDelay;
        //     iSize++;
        // } else {
        //     errorIntegral += (drift-errorProportional)*sampleDelay;
        // }
        // errorDerivative = (drift - errorProportional)/sampleDelay;
        // errorProportional = drift;
        // correction = KBIAS*(KP*errorProportional + KI*errorIntegral + KD*errorDerivative);
        correction = drift;
        dtostrf(correction, 5, 3, outstr);
        BT_send_pkt((BT_packet_t){PKT_STRING, 10, outstr});

        // Get IMU orientation
        getHeading(&yaw, &pitch, &roll);
        // dtostrf(yaw, 7, 3, outstr);
        // printf("Yaw: %s  ", outstr);
        // dtostrf(pitch, 7, 3, outstr);
        // printf("Pitch: %s  ", outstr);
        // dtostrf(roll, 7, 3, outstr);
        // printf("Roll: %s\t", outstr);
        dtostrf(yaw, 3, 0, outstr);
        BT_send_pkt((BT_packet_t){PKT_STRING, 4, outstr});

        // Read line sensor
        lineVal = readLineSensors();
        // dtostrf(lineVal, 3, 3, outstr);
        // printf("%s", outstr);

        // printf("\n");

        switch(robotMode) {
            case ROBOT_MODE_CONTROL_AND_AVOID:
                if (getObstacleDistanceCm() < 25) {
                    stop();
                    stopped = 1;
                }
                if ((BT_has_pkt()&2)!=0) PORTB|=1<<PORTB5; // If packet overflow, turn on LED
                if (((BT_has_pkt()&1)==1)) { // If a packet is received
                    BT_packet_t pkt = BT_get_pkt(); // Get the received packet

                    if (straight == 0) {
                        straight = 1;
                        start_counting(0);
                    }
                    // Update motor speeds for Joystick type data
                    if (pkt.pkt_type == PKT_JOYSTICK_Y && pkt.pkt_len==2) {
                        uint16_t val = pkt.pkt_val[0];
                        val |= pkt.pkt_val[1]<<8;
                        if (val>=JOY_Y_MAX) {
                            baseSpeed = -MAX_SPEED;
                        } else if (val<=JOY_Y_MIN) {
                            baseSpeed = MAX_SPEED;
                        } else if (val <= JOY_Y_NORM - JOY_DELTA) {
                            baseSpeed = (double)(JOY_Y_NORM  - JOY_DELTA - val)/(double)(JOY_Y_NORM - JOY_DELTA - JOY_Y_MIN)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
                        } else if (val >= JOY_Y_NORM + JOY_DELTA) {
                            baseSpeed = (double)(val - JOY_Y_NORM - JOY_DELTA)/(double)(JOY_Y_MAX - JOY_Y_NORM - JOY_DELTA)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
                            baseSpeed = -baseSpeed;
                        } else {
                            baseSpeed = 0.0;
                        }
                        if (correction > 0) {
                            rightBias = correction;
                            leftBias = 0;
                        } else if (correction < 0) {
                            leftBias = -correction;
                            rightBias = 0;
                        }
                    } else if (pkt.pkt_type == PKT_JOYSTICK_X && pkt.pkt_len==2) {
                        uint16_t val = pkt.pkt_val[0];
                        val |= pkt.pkt_val[1]<<8;
                        if (val>=JOY_X_MAX) {
                            leftBias = MAX_SPEED;
                            rightBias = -leftBias;
                            straight = 0;
                        } else if (val<=JOY_X_MIN){
                            rightBias = MAX_SPEED;
                            leftBias = -rightBias;
                            straight = 0;
                        } else if (val >= JOY_X_NORM + JOY_DELTA) {
                            leftBias = (double)(val - JOY_X_NORM - JOY_DELTA)/(double)(JOY_X_MAX - JOY_X_NORM - JOY_DELTA)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
                            rightBias = -leftBias/2.0;
                            straight = 0;
                        } else if (val <= JOY_X_NORM - JOY_DELTA) {
                            rightBias = (double)(JOY_X_NORM - JOY_DELTA - val)/(double)(JOY_X_NORM - JOY_DELTA - JOY_X_MIN)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
                            leftBias = -rightBias/2.0;
                            straight = 0;
                        } else  {
                            if (straight == 0) {
                                straight = 1;
                                start_counting(0);
                            }
                            if (correction > 0) {
                                rightBias = correction;
                                leftBias = 0;
                            } else if (correction < 0) {
                                leftBias = -correction;
                                rightBias = 0;
                            } else {
                                rightBias = 0.0;
                                leftBias = 0.0;
                            }
                        }
                    }

                    BT_send_pkt(pkt); // Respond by sending the received packet over bluetooth
                    int dist = (int)getObstacleDistanceCm();
                    BT_send_pkt((BT_packet_t){PKT_DISTANCE, 2, (uint8_t[2]){(dist>>8) & 0xff, dist & 0xff}});
                }
                double dist = getObstacleDistanceCm();
                if (stopped == 0 && dist < 25) {
                    stop();
                    stopped = 1;
                } else if (error() == 0) {
                    if (dist >= 25) stopped = 0;
                    if (baseSpeed < 0) {
                        setSpeed((-baseSpeed + leftBias), (-baseSpeed + rightBias));
                        moveBackward();
                    } else if (stopped == 0) {
                        setSpeed((baseSpeed + leftBias), (baseSpeed + rightBias));
                        moveForward();
                    }
                }
                break;

            case ROBOT_MODE_AUTONOMOUS_AVOID:
                // Add correction to account for drift bias
                if (correction > 0) {
                    rightBias = correction;
                    leftBias = 0;
                } else if (correction < 0) {
                    leftBias = -correction;
                    rightBias = 0;
                }

                // Avoid obstacles by turning right when an object is too close
                // (Simple implementation, not perfect. Keep robot under supervision)
                if (error() == 0) {
                    if (getObstacleDistanceCm() <= 25) {
                        setDelay_ms(0);
                        stopped = 1;
                        setSpeed((MAX_SPEED+MIN_SPEED)/2.0 + leftBias, (MAX_SPEED+MIN_SPEED)/2.0 + rightBias);
                        rotateRight();
                    } else if (stopped == 1 && delay_done()) { // Rotate right until object not seen, then extra period of rotation as overhead
                        setDelay_ms(320);
                        stopped = 0;
                    } else if (delay_done() == 1) {
                        setSpeed((MAX_SPEED+MIN_SPEED)/2.0 + leftBias, (MAX_SPEED+MIN_SPEED)/2.0 + rightBias);
                        moveForward();
                        stopped = 0;
                    }
                }
                break;
        }
    }
}
