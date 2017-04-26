#include "robot.h"

static float yaw, pitch, roll;

// To convert double to string for displaying
static char outstr[15];

static uint8_t stopped = 0;
static double baseSpeed = 0, rightBias = 0, leftBias = 0;
static int bugState = BUG_FORWARD_0;
static int searchState = SEARCH_START;
static int lastX = 0, lastY = 0;
static double bugDist;

void setup(uint8_t calibrate) {
     // On board LED
    DDRB |= (1<<DDB5); // Sets B5 (LED) as output pin

    // Timer setup (timer0)
    timer_init();
    // Motor setup
    motor_init();
    // Servo setup (timer2)
    servo_init();
    // Ultrasonic Sensor setup (timer1)
    ultrasonic_sensor_init();

    // Line Sensor Setup
    DDRB &= ~(1<<DDB3); // Sets B3 (lLine) as input pin
    DDRD &= ~(1<<DDD2); // Sets D2 (rLine) as input pin
    DDRC &= ~(1<<DDC0); // Sets C0 (mLine) as input pin

    // Bluetooth setup
    BT_init();
    // IMU setup
    mpu9250_init(calibrate);
    // Mouse odometer setup
    mouse_init();
    // Line Sensor setup
    line_sensor_init();

    // Enable interrupts
    sei();
}

void getHeading(float* Yaw, float* Pitch, float* Roll) {
    if (mpu9250_dataReady())
        mpu9250_readData(&yaw, &pitch, &roll);
    *Yaw = yaw;
    *Pitch = pitch;
    *Roll = roll;
}

void getDisplacement(char* stat, char* x, char* y) {
    mouse_pos(stat, x, y);
}

uint8_t robot_control_and_avoid_task(double correction, float heading, uint8_t* straight) {
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
                    *straight = 1;
                    start_counting(0);
                }
                baseSpeed = -MAX_SPEED;
            } else if (val<=JOY_Y_MIN) {
                if (straight == 0) {
                    *straight = 1;
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
                *straight = 0;
            } else if (val<=JOY_X_MIN){
                rightBias = MAX_SPEED;
                leftBias = 0;
                *straight = 0;
            } else if (val >= JOY_X_NORM + JOY_DELTA) {
                leftBias = (double)(val - JOY_X_NORM - JOY_DELTA)/(double)(JOY_X_MAX - JOY_X_NORM - JOY_DELTA)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
                rightBias = 0;
                *straight = 0;
            } else if (val <= JOY_X_NORM - JOY_DELTA) {
                rightBias = (double)(JOY_X_NORM - JOY_DELTA - val)/(double)(JOY_X_NORM - JOY_DELTA - JOY_X_MIN)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
                leftBias = 0;
                *straight = 0;
            } else  {
                if (straight == 0) {
                    *straight = 1;
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
            bugState = BUG_FORWARD_0;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
            return ROBOT_MODE_AUTONOMOUS_AVOID;
        } else if (pkt.pkt_type == PKT_CONTROL_MODE) {
            stop();
            setServoAngle(0);
            servoSweepOn();
            BT_send_pkt((BT_packet_t){PKT_STRING, 5, (uint8_t*)"begin"});
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
            return ROBOT_MODE_CONTROL_AND_AVOID;
        } else if (pkt.pkt_type == PKT_SET_HOME) {
            stop();
            setPosition(0,0);
            setDirection(DIRECTION_UP);
        } else if (pkt.pkt_type == PKT_GO_HOME) {
            stop();
            servoSweepOff();
            setServoAngle(0);
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
            searchState = SEARCH_START;
            return ROBOT_MODE_AUTONOMOUS_HOME;
        } else if (pkt.pkt_type == PKT_RIGHT) {
            updateDirection(DIRECTION_RIGHT);
        } else if (pkt.pkt_type == PKT_LEFT) {
            updateDirection(DIRECTION_LEFT);
        }

        BT_send_pkt(pkt); // Respond by sending the received packet over bluetooth

        for (int i = 0; i < 15; i++)
            outstr[i] = ' ';
        dtostrf(heading, 3, 0, outstr);
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
    return ROBOT_MODE_CONTROL_AND_AVOID;
}

uint8_t robot_autonomous_avoid_task(double correction, float heading, uint8_t* straight) {
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
            bugState = BUG_FORWARD_0;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
            return ROBOT_MODE_AUTONOMOUS_AVOID;
        } else if (pkt.pkt_type == PKT_CONTROL_MODE) {
            stop();
            setServoAngle(0);
            servoSweepOn();
            BT_send_pkt((BT_packet_t){PKT_STRING, 5, (uint8_t*)"begin"});
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
            return ROBOT_MODE_CONTROL_AND_AVOID;
        } else if (pkt.pkt_type == PKT_SET_HOME) {
            stop();
            setPosition(0,0);
            setDirection(DIRECTION_UP);
        } else if (pkt.pkt_type == PKT_GO_HOME) {
            stop();
            servoSweepOff();
            setServoAngle(0);
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
            searchState = SEARCH_START;
            return ROBOT_MODE_AUTONOMOUS_HOME;
        } else if (pkt.pkt_type == PKT_RIGHT) {
            updateDirection(DIRECTION_RIGHT);
        } else if (pkt.pkt_type == PKT_LEFT) {
            updateDirection(DIRECTION_LEFT);
        }

        BT_send_pkt(pkt); // Respond by sending the received packet over bluetooth

        for (int i = 0; i < 15; i++)
            outstr[i] = ' ';
        dtostrf(correction, 5, 0, outstr);
        BT_send_pkt((BT_packet_t){PKT_DRIFT, 15, outstr});
        for (int i = 0; i < 15; i++)
            outstr[i] = ' ';
        dtostrf(heading, 3, 0, outstr);
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
    return ROBOT_MODE_AUTONOMOUS_AVOID;
}

uint8_t robot_autonomous_home_task(double correction, float heading, uint8_t* straight){
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
            bugState = BUG_FORWARD_0;
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
            return ROBOT_MODE_AUTONOMOUS_AVOID;
        } else if (pkt.pkt_type == PKT_CONTROL_MODE) {
            stop();
            setServoAngle(0);
            servoSweepOn();
            BT_send_pkt((BT_packet_t){PKT_STRING, 5, (uint8_t*)"begin"});
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
            return ROBOT_MODE_CONTROL_AND_AVOID;
        } else if (pkt.pkt_type == PKT_SET_HOME) {
            stop();
            setPosition(0,0);
            setDirection(DIRECTION_UP);
        } else if (pkt.pkt_type == PKT_GO_HOME) {
            stop();
            servoSweepOff();
            setServoAngle(0);
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 1;
            searchState = SEARCH_START;
            return ROBOT_MODE_AUTONOMOUS_HOME;
        } else if (pkt.pkt_type == PKT_RIGHT) {
            updateDirection(DIRECTION_RIGHT);
        } else if (pkt.pkt_type == PKT_LEFT) {
            updateDirection(DIRECTION_LEFT);
        }

        BT_send_pkt(pkt); // Respond by sending the received packet over bluetooth
    } else {
        stop();
        stopped = 2;
    }

    int x = getIntPositionX();
    int abs_x = (x < 0)? -x : x;
    int y = getIntPositionY();
    int abs_y = (y < 0)? -y : y;
    switch (searchState) {
        case SEARCH_START:
            if (delay_done() && (x!=0 || y!=0)) {
                if (abs_x > abs_y) {
                    if (x > 0) {
                        searchState = SEARCH_TURN_LEFT;
                    } else {
                        searchState = SEARCH_TURN_RIGHT;
                    }
                } else {
                    if (y > 0) {
                        searchState = SEARCH_TURN_DOWN;
                    } else {
                        searchState = SEARCH_TURN_UP;
                    }
                }
            }
            break;

        case SEARCH_TURN_UP:
            if (delay_done() && getDirection() != DIRECTION_UP) {
                setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                rotateRight();
                setDelay_ms(100);
                updateDirection(DIRECTION_RIGHT);
            } else if (delay_done() && getDirection() == DIRECTION_UP) {
                stop();
                setDelay_ms(1000);
                lastX = x;
                lastY = y;
                searchState = SEARCH_UP;
            }
            break;

        case SEARCH_TURN_RIGHT:
            if (delay_done() && getDirection() != DIRECTION_RIGHT) {
                setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                rotateRight();
                setDelay_ms(100);
                updateDirection(DIRECTION_RIGHT);
            } else if (delay_done() && getDirection() == DIRECTION_RIGHT) {
                stop();
                setDelay_ms(1000);
                lastX = x;
                lastY = y;
                searchState = SEARCH_RIGHT;
            }
            break;

        case SEARCH_TURN_DOWN:
            if (delay_done() && getDirection() != DIRECTION_DOWN) {
                setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                rotateRight();
                setDelay_ms(100);
                updateDirection(DIRECTION_RIGHT);
            } else if (delay_done() && getDirection() == DIRECTION_DOWN) {
                stop();
                setDelay_ms(1000);
                lastX = x;
                lastY = y;
                searchState = SEARCH_DOWN;
            }
            break;

        case SEARCH_TURN_LEFT:
            if (delay_done() && getDirection() != DIRECTION_LEFT) {
                setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                rotateRight();
                setDelay_ms(100);
                updateDirection(DIRECTION_RIGHT);
            } else if (delay_done() && getDirection() == DIRECTION_LEFT) {
                stop();
                setDelay_ms(1000);
                lastX = x;
                lastY = y;
                searchState = SEARCH_LEFT;
            }
            break;

        case SEARCH_UP:
        case SEARCH_DOWN:
            if (delay_done() && x == lastX) {
                setSpeed((MAX_SPEED+MIN_SPEED)*.6, (MAX_SPEED+MIN_SPEED)*.6);
                moveForward();
                setDelay_ms(100);
            } else if (delay_done() && x != lastX) {
                stop();
                setDelay_ms(1000);
                searchState = SEARCH_START;
            }
            break;

        case SEARCH_LEFT:
        case SEARCH_RIGHT:
            if (delay_done() && y == lastY) {
                setSpeed((MAX_SPEED+MIN_SPEED)*.6, (MAX_SPEED+MIN_SPEED)*.6);
                moveForward();
                setDelay_ms(100);
            } else if (delay_done() && y != lastY) {
                stop();
                setDelay_ms(1000);
                searchState = SEARCH_START;
            }
            break;
    }

    stop();
    return ROBOT_MODE_AUTONOMOUS_HOME;
}
