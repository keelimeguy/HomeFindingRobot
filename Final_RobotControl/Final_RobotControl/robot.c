#include "robot.h"


// Contains code for the three robot modes as well as other helpful functions:
//     The remote controlled mode - robot_control_and_avoid_task()
//     The obstacle avoiding mode - robot_autonomous_avoid_task()
//     The autonomous home returning mode - robot_autonomous_home_task()
// This is a long file, it may help to ctrl-f (or your text editor's equivalent to "find") for the functions of interest


// Variables to hold orientation of robot
static float yaw, pitch, roll;

// To convert double to string for displaying
static char outstr[15];

// Flag variables for the state of the robot
static uint8_t stopped = 0, verbose = 0;
static int bugState = BUG_FORWARD_0;
static int searchState = SEARCH_START;

// The last measured x and y coordinate positions and obstacle distance,
//   used in autonomous home return mode
static int lastX = 0, lastY = 0;
static double bugDist;

// The current base speed and motor bias speed variables
static double baseSpeed = 0, rightBias = 0, leftBias = 0;

static void yJOYTask(BT_packet_t pkt, double correction, uint8_t* straight);
static void xJoyTask(BT_packet_t pkt, double correction, uint8_t* straight);
static uint8_t void handleOtherPKT(BT_packet_t pkt, double correction, float heading);

// Setup teh robot by initializing everything
void setup(uint8_t calibrate) {
     // On board LED
    DDRB |= (1<<DDB5); // Sets B5 (LED) as output pin

    // Timer setup (timer0)
    timer_init(); // (timer.c)

    // Motor setup
    motor_init(); // (motor.c)

    // Servo setup (timer2)
    servo_init(); // (servo.c)

    // Ultrasonic Sensor setup (timer1)
    ultrasonic_sensor_init(); // (ultrasonic_sensor.c)

    // Line Sensor Setup
    DDRB &= ~(1<<DDB3); // Sets B3 (lLine) as input pin
    DDRD &= ~(1<<DDD2); // Sets D2 (rLine) as input pin
    DDRC &= ~(1<<DDC0); // Sets C0 (mLine) as input pin

    // Bluetooth setup
    BT_init(); // (BT.c)

    // IMU setup, calibrate if indicated
    mpu9250_init(calibrate); // (mpu9250_lib.c)

    // Mouse odometer setup
    mouse_init(); // (ps2.c)

    // Line Sensor setup
    line_sensor_init(); // (line_sensor.c)

    // Enable interrupts
    sei();
}

// Get the orientation from the IMU
void getHeading(float* Yaw, float* Pitch, float* Roll) {
    if (mpu9250_dataReady()) // Read only if IMU is ready to be read (mpu9250_lib.c)
        mpu9250_readData(&yaw, &pitch, &roll); // (mpu9250_lib.c)
    // Return values by reference
    *Yaw = yaw;
    *Pitch = pitch;
    *Roll = roll;
}

// Get the displacement data from the PS/2 mouse (odometer)
void getDisplacement(char* stat, char* x, char* y) {
    // Return values by reference
    mouse_pos(stat, x, y); // (ps2.c)
}

// Handles Y-Joystick bluetooth packets (forward/backward control)
static void yJOYTask(BT_packet_t pkt, double correction, uint8_t* straight) {
    // Parse packet data
    uint16_t val = pkt.pkt_val[0];
    val |= pkt.pkt_val[1]<<8;
    if (val>=JOY_Y_MAX) { // Positive Y means moving backwards
        if (straight == 0) { // If we weren't going straight, start now
            // Return variable by reference
            *straight = 1;
            // Start the simulated timer 0
            start_counting(0); // (timer.c)
        }
        // Limit minimum speed
        baseSpeed = -MAX_SPEED;
    } else if (val<=JOY_Y_MIN) { // Negative Y means moving backwards
        if (straight == 0) { // If we weren't going straight, start now
            // Seturn variable by reference
            *straight = 1;
            // Start the simulated timer 0
            start_counting(0); // (timer.c)
        }
        // Limit maximum speed
        baseSpeed = MAX_SPEED;
    } else if (val <= JOY_Y_NORM - JOY_DELTA) { // If joystick value passed a certain negative threshold
        // Calculate the base speed of the robot based on percentage of the way the joystick is from maximum
        baseSpeed = (double)(JOY_Y_NORM  - JOY_DELTA - val)/(double)(JOY_Y_NORM - JOY_DELTA - JOY_Y_MIN)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
    } else if (val >= JOY_Y_NORM + JOY_DELTA) { // If joystick value passed a certain positive threshold
        // Calculate the base speed of the robot based on percentage of the way the joystick is from minimum
        baseSpeed = (double)(val - JOY_Y_NORM - JOY_DELTA)/(double)(JOY_Y_MAX - JOY_Y_NORM - JOY_DELTA)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
        // Negative Y means moving backwards
        baseSpeed = -baseSpeed;
    } else { // If joystick did not pass threshold, reset speed to zero
        baseSpeed = 0.0;
    }
    // Handle PID to have robot going straight
    if (straight) {
        // Adjust speed correction based on robot speed
        correction = correction*baseSpeed;
        if (correction > 0) { // Positive correction means drifted right
            // Increase right bias to adjust towards left
            rightBias = correction;
            leftBias = 0.0;
        } else if (correction < 0) { // Negative correction means drifted left
            // Increase left bias to adjust towards right
            leftBias = -correction;
            rightBias = 0.0;
        } else { // No correction needed, reset bias speeds
            leftBias = 0.0;
            rightBias = 0.0;
        }
    }
}

// Handles Y-Joystick bluetooth packets (forward/backward control)
static void xJoyTask(BT_packet_t pkt, double correction, uint8_t* straight) {
    // Parse packet data
    uint16_t val = pkt.pkt_val[0];
    val |= pkt.pkt_val[1]<<8;
    if (val>=JOY_X_MAX) { // Positive X means moving right
        leftBias = MAX_SPEED;
        rightBias = 0;
        // Set value by reference
        *straight = 0; // If we are turning, we are not going straight
    } else if (val<=JOY_X_MIN){ // Negative X means moving left
        rightBias = MAX_SPEED;
        leftBias = 0;
        // Set value by reference
        *straight = 0; // If we are turning, we are not going straight
    } else if (val >= JOY_X_NORM + JOY_DELTA) { // If joystick value passed a certain positive threshold (turning right)
        // Calculate the left bias speed of the robot based on percentage of the way the joystick is from maximum
        leftBias = (double)(val - JOY_X_NORM - JOY_DELTA)/(double)(JOY_X_MAX - JOY_X_NORM - JOY_DELTA)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
        rightBias = 0;
        // Set value by reference
        *straight = 0; // If we are turning, we are not going straight
    } else if (val <= JOY_X_NORM - JOY_DELTA) { // If joystick value passed a certain negative threshold (turning left)
        // Calculate the right bias speed of the robot based on percentage of the way the joystick is from minimum
        rightBias = (double)(JOY_X_NORM - JOY_DELTA - val)/(double)(JOY_X_NORM - JOY_DELTA - JOY_X_MIN)*(MAX_SPEED-MIN_SPEED)+MIN_SPEED;
        leftBias = 0;
        // Set value by reference
        *straight = 0; // If we are turning, we are not going straight
    } else  {
        if (straight == 0) {
            // If we are not turning, we must be going straight
            *straight = 1; // Set value by reference
            // Start the simulated timer 0
            start_counting(0); // (timer.c)
        }
        // Adjust speed correction based on robot speed
        correction = correction*baseSpeed;
        if (straight && correction > 0) { // Positive correction means drifted right
            // Increase right bias to adjust towards left
            rightBias = correction;
            leftBias = 0.0;
        } else if (straight && correction < 0) { // Negative correction means drifted left
            // Increase left bias to adjust towards right
            leftBias = -correction;
            rightBias = 0.0;
        } else { // No correction needed, reset bias speeds
            rightBias = 0.0;
            leftBias = 0.0;
        }
    }
}

// Handle other specific command packets
static uint8_t void handleOtherPKT(BT_packet_t pkt, double correction, float heading) {
    // Handle command to enter autonomous avoid mode
    if (pkt.pkt_type == PKT_AUTO_MODE) {
        stop(); // (motor.c)
        // Set servo facing forward
        setServoAngle(0); // (servo.c)
        // Start servo sweep
        servoSweepOn(); // (servo.c)
        // Set bug state to initial state (moving forward)
        bugState = BUG_FORWARD_0;
        // Reset speeds and flags
        baseSpeed = 0;
        leftBias = 0;
        rightBias = 0;
        stopped = 1;
        verbose = 1;
        // Change mode of robot to autonomus obstacle avoidance mode
        return ROBOT_MODE_AUTONOMOUS_AVOID;

    // Handle command to enter remote control mode
    } else if (pkt.pkt_type == PKT_CONTROL_MODE) {
        stop(); // (motor.c)
        // Set servo facing forward
        setServoAngle(0); // (servo.c)
        // Start servo sweep
        servoSweepOn(); // (servo.c)
        // Send a begin packet over bluetooth, indicating start of remote control mode
        BT_send_pkt((BT_packet_t){PKT_STRING, 5, (uint8_t*)"begin"}); // (BT.c)
        // Reset speeds and flags
        baseSpeed = 0;
        leftBias = 0;
        rightBias = 0;
        stopped = 1;
        verbose = 1;
        // Change mode of robot to remote control mode
        return ROBOT_MODE_CONTROL_AND_AVOID;

    // Handle command to set new home position
    } else if (pkt.pkt_type == PKT_SET_HOME) {
        stop(); // (motor.c)
        // Set home by setting position to origin, facing up
        setPosition(0,0); // (line_sensor.c)
        setDirection(DIRECTION_UP); // (line_sensor.c)

    // Handle command to enter autonomous home return mode
    } else if (pkt.pkt_type == PKT_GO_HOME) {
        stop(); // (motor.c)
        // Stop servo sweep
        servoSweepOff(); // (servo.c)
        // Set servo facing forward
        setServoAngle(0); // (servo.c)
        // Reset speeds and flags
        baseSpeed = 0;
        leftBias = 0;
        rightBias = 0;
        stopped = 1;
        verbose = 1;
        // Set home finding state to initial state
        searchState = SEARCH_START;
        // Change mode of robot to autonomous home return mode
        return ROBOT_MODE_AUTONOMOUS_HOME;

    // Handle command to update direction by right turn
    } else if (pkt.pkt_type == PKT_RIGHT) {
        // Update direction by right turn
        updateDirection(DIRECTION_RIGHT); // (line_sensor.c)
    // Handle command to update direction by left turn
    } else if (pkt.pkt_type == PKT_LEFT) {
        // Update direction by left turn
        updateDirection(DIRECTION_LEFT); // (line_sensor.c)
    }

    BT_send_pkt(pkt); // Respond by sending the received packet over bluetooth (BT.c)

    if (verbose) { // If we have flag set to display extra information
        // Clear string buffer (to empty spaces)
        for (int i = 0; i < 15; i++)
            outstr[i] = ' ';
        dtostrf(heading, 3, 0, outstr); // Convert heading data to string
        // Send heading over bluetooth
        BT_send_pkt((BT_packet_t){PKT_HEADING, 15, outstr}); // (BT.c)

        // Clear string buffer (to empty spaces)
        for (int i = 0; i < 15; i++)
            outstr[i] = ' ';
        dtostrf((double)((int)correction*1000), 7, 0, outstr); // Convert correction data to string
        // Send correction over bluetooth
        BT_send_pkt((BT_packet_t){PKT_DRIFT, 15, outstr}); // (BT.c)

        // Send position over bluetooth
        BT_send_pkt((BT_packet_t){PKT_POSITION, 2, (uint8_t[2]){getPositionX(), getPositionY()}}); // (BT.c), (line_sensor.c)
    }

    // Read the distance through servo sweep
    int dist = (int)servoSweepDistanceTask(); // (servo.c)
    // Send distance over bluetooth
    BT_send_pkt((BT_packet_t){PKT_DISTANCE, 2, (uint8_t[2]){(dist>>8) & 0xff, dist & 0xff}}); // (BT.c)

    // Indicate no update in mode
    return 0;
}




// The main robot task code for the remote controlled mode
uint8_t robot_control_and_avoid_task(double correction, float heading, uint8_t* straight) {
    // Read the distance through servo sweep
    if (servoSweepDistanceTask() < 30 && stopped == 0) { // (servo.c)
        // Stop robot if threshold reached and we are not already stopped
        stop(); // (motor.c)
        stopped = 1; // Set stopped flag
    }
    if ((BT_has_pkt()&2)!=0) PORTB|=1<<PORTB5; // If packet overflow, turn on LED (BT.c)
    if (((BT_has_pkt()&1)==1)) { // If a packet is received (BT.c)
        BT_packet_t pkt = BT_get_pkt(); // Get the received packet (BT.c)

        // Handle receive STOP command
        if (pkt.pkt_type == PKT_STOP) {
            // Stop moving, including servo
            stop(); // (motor.c)
            servoSweepOff(); // (servo.c)
            // Make sure speeds are reset to zero
            baseSpeed = 0;
            leftBias = 0;
            rightBias = 0;
            stopped = 2; // Set flag to indicate forced stop

        // Handle receive Y Joystick
        } else if (pkt.pkt_type == PKT_JOYSTICK_Y && pkt.pkt_len==2) {
            yJoyTask(pkt, correction, straight);

        // Handle receive X Joystick
        } else if (pkt.pkt_type == PKT_JOYSTICK_X && pkt.pkt_len==2) {
            xJoyTask(pkt, correction, straight);

        // Handle other types of packet commands
        } else {
            uint8_t nextMode = handleOtherPKT(pkt, correction, heading);
            if (nextMode) // If we updated our mode, quit from current mode and return the change
                return nextMode;
        }
    }

    // Read the distance through servo sweep
    double dist = servoSweepDistanceTask(); // (servo.c)
    if (error() == 0 && stopped < 2) { // If no BT error and not forced stopped (timer.c)
        if (stopped == 0 && dist < 40) { // If stop not handled and obstacle within threshold then stop
            stop(); // (motor.c)
            stopped = 1; // Set stop handle flag
            // Wait a bit after stopping
            setDelay_ms(50); // Set delay through simulated timer (timer.c)
        }

        if (error() == 0 && delay_done()) { // If no BT error and delay finished (timer.c)
            if (dist >= 40) stopped = 0; // Reset stop flag if necessary
            // Handle negative speed, moving backwards
            if (baseSpeed < 0) {
                // Update speed and direction of robot
                setSpeed(sqrt(baseSpeed*baseSpeed + leftBias*leftBias) - rightBias/2, sqrt(baseSpeed*baseSpeed + rightBias*rightBias) - leftBias/2); // (motor.c)
                moveBackward(); // (motor.c)

            // Handle positive speed, moving forwards, but don;t allow movement if stopped
            } else if (stopped == 0) {
                // Update speed and direction of robot
                setSpeed(sqrt(baseSpeed*baseSpeed + leftBias*leftBias)  - rightBias/2, sqrt(baseSpeed*baseSpeed + rightBias*rightBias) - leftBias/2); // (motor.c)
                moveForward(); // (motor.c)
            } else stop(); // (motor.c)
        }
    } else stop(); // (motor.c)

    // Maintain that we are in remote controlled mode
    return ROBOT_MODE_CONTROL_AND_AVOID;
}




// The main robot task code for the autonomous obstacle avoidance mode
uint8_t robot_autonomous_avoid_task(double correction, float heading, uint8_t* straight) {
    // Add correction to account for drift bias
    correction=-correction;
    if (correction > 0) { // Positive correction means drifted right
        // Increase right bias to adjust towards left
        rightBias = correction;
        leftBias = 0.0;
    } else if (correction < 0) { // Negative correction means drifted left
        // Increase left bias to adjust towards right
        leftBias = -correction;
        rightBias = 0.0;
    }

    if ((BT_has_pkt()&2)!=0) PORTB|=1<<PORTB5; // If packet overflow, turn on LED (BT.c)
    if (((BT_has_pkt()&1)==1)) { // If a packet is received (BT.c)
        BT_packet_t pkt = BT_get_pkt(); // Get the received packet (BT.c)

        // Handle receive STOP command
        if (pkt.pkt_type == PKT_STOP) {
            // Stop moving, including servo
            stop(); // (motor.c)
            servoSweepOff(); // (servo.c)
            stopped = 2; // Set flag to indicate forced stop
        } else {
            uint8_t nextMode = handleOtherPKT(pkt, correction, heading);
            if (nextMode) // If we updated our mode, quit from current mode and return the change
                return nextMode;
        }
    }

    // Avoid obstacles by turning right when an object is too close
    // (Simple implementation, not perfect. Keep robot under supervision)
    if (error() == 0 && stopped < 2) { // (timer.c)

        // Bug algorithm state transition (incomplete bug algorithm, only does moves parallel to obstacle)
        switch(bugState) {

            // Move forward state
            case BUG_FORWARD_0:
                // Read the distance through servo sweep, always wait for delay to finish
                if (delay_done() && servoSweepDistanceTask() <= 35) { // (timer.c), (servo.c)
                    // If obstacle is there, transition to next state
                    bugState = BUG_FORWARD_1;
                    // Stop before changing state
                    stop(); // (motor.c)
                    stopped = 1; // Set stop flag
                    setDelay_ms(150); // (timer.c)

                // Read the distance through servo sweep, always wait for delay to finish
                } else if (delay_done() && servoSweepDistanceTask() > 35) { // (timer.c), (servo.c)
                    // If no obstacle is there, move the robot forward
                    setSpeed((MAX_SPEED-MIN_SPEED)*0.2 + MIN_SPEED + leftBias, (MAX_SPEED-MIN_SPEED)*0.2 + MIN_SPEED + rightBias);
                    moveForward(); // (motor.c)
                    stopped = 0; // Reset stop flag
                }
                break;

            // Obstacle detected, move a bit closer to obstacle state
            case BUG_FORWARD_1:
                // Read the distance through servo sweep, always wait for delay to finish
                if (delay_done() && servoSweepDistanceTask() <= 20) { // (timer.c), (servo.c)
                    // If obstacle is there, transition to next state
                    bugState = BUG_CHECK_CORNER_1;
                    // Turn off servo sweep and set angle to full left (90 degrees)
                    servoSweepOff(); // (servo.c)
                    setServoAngle(90); // (servo.c)
                    setDelay_ms(300); // Wait a bit for servo to turn (timer.c)
                    // Stop before changing state
                    stop(); // (motor.c)
                    stopped = 1; // Set stop flag

                // Read the distance through servo sweep, always wait for delay to finish
                } else if (delay_done() && servoSweepDistanceTask() > 20) { // (timer.c), (servo.c)
                    // If no obstacle is there, move forward
                    setSpeed((MAX_SPEED-MIN_SPEED)*0.075 + MIN_SPEED, (MAX_SPEED-MIN_SPEED)*0.075 + MIN_SPEED);
                    moveForward(); // (motor.c)
                    stopped = 0; // Reset stop flag
                }
                break;

            // At obstacle, look to left to see if another obstacle (a corner)
            case BUG_CHECK_CORNER_1:
                // Read the distance directly through ultrasonic sensor, always wait for delay to finish
                if (delay_done() && getObstacleDistanceCm() <= 50) { // (timer.c), (ultrasonic_sensor.c)
                    // If obstacle is there, then it is a corner
                    // Move servo to look slightly right
                    setServoAngle(-15); // (servo.c)
                    setDelay_ms(300); // Wait a bit for servo to turn (timer.c)
                    // Transition to next state
                    bugState = BUG_TURN_AT_CORNER;

                // Read the distance directly through ultrasonic sensor, always wait for delay to finish
                } else if (delay_done() && getObstacleDistanceCm() > 50) { // (timer.c), (ultrasonic_sensor.c)
                    // No obstacle is there, not a corner
                    // Move servo to look slightly less than full left
                    setServoAngle(60); // (servo.c)
                    setDelay_ms(300); // Wait a bit for servo to turn (timer.c)
                    // Transition to next state
                    bugState = BUG_CHECK_CORNER_2;
                }
                break;

            // At corner, checking to see if we are at odd angle to wall
            case BUG_CHECK_CORNER_2:
                // Read the distance directly through ultrasonic sensor, always wait for delay to finish
                if (delay_done() && getObstacleDistanceCm() <= 50) { // (timer.c), (ultrasonic_sensor.c)
                    // An obstacle is detected, this must be a corner
                    // Turn servo to look a towards the front right
                    setServoAngle(-30); // (servo.c)
                    setDelay_ms(300); // Wait a bit for servo to turn timer.c)
                    // Transition to next state
                    bugState = BUG_TURN_AT_CORNER;

                // Read the distance directly through ultrasonic sensor, always wait for delay to finish
                } else if (delay_done() && getObstacleDistanceCm() > 50) // (timer.c), (ultrasonic_sensor.c)
                    // No obstacle is there, continue to next state normally
                    bugState = BUG_TURN_AT_WALL_0;
                break;

            // At corner, turning right until mostly parallel
            case BUG_TURN_AT_CORNER:
                // Read the distance directly through ultrasonic sensor, always wait for delay to finish
                if (delay_done() && getObstacleDistanceCm() <= 50) { // (timer.c), (ultrasonic_sensor.c)
                    // Keep turning while we see the obstacle (corner)
                    setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                    rotateRight(); // (motor.c)
                    setDelay_ms(30); // (timer.c)

                // Read the distance directly through ultrasonic sensor, always wait for delay to finish
                } else if (delay_done() && getObstacleDistanceCm() > 50) { // (timer.c), (ultrasonic_sensor.c)
                    // No more obstacle, we can stop turning
                    // Turn servo to slightly less than full left
                    setServoAngle(60); // (servo.c)
                    setDelay_ms(300); // Wait a bit for servo to turn (timer.c)
                    // Stop before changing state
                    stop(); // (motor.c)
                    stopped = 1; // Set stop flag
                    // Transition the state
                    bugState = BUG_TURN_AT_WALL_0;
                }
                break;

            // At Wall, turning right until we see obstacle at our left
            case BUG_TURN_AT_WALL_0:
                // Read the distance directly through ultrasonic sensor, always wait for delay to finish
                if (delay_done() && getObstacleDistanceCm() > 50) { // (timer.c), (ultrasonic_sensor.c)
                    // While no obstacle is there, turn right
                    setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                    rotateRight(); // (motor.c)
                    setDelay_ms(30); // (timer.c)

                // Read the distance directly through ultrasonic sensor, always wait for delay to finish
                } else if (delay_done() && getObstacleDistanceCm() <= 50) { // (timer.c), (ultrasonic_sensor.c)
                    // Obstacle detected, stop turning
                    // Stop before changing state
                    stop(); // (motor.c)
                    stopped = 1
                    // Read the distance directly through ultrasonic sensor, store it in bugDist variable (last distance seen)
                    bugDist = getObstacleDistanceCm(); // (ultrasonic_sensor.c)
                    // Turn servo to middle left
                    setServoAngle(45); // (servo.c)
                    setDelay_ms(300); // Wait a bit for servo to turn (timer.c)
                    // Transition to next state
                    bugState = BUG_TURN_AT_WALL_1;
                }
                break;

            // At Wall, comparing current distance with last distance to see if we need to keep turning
            case BUG_TURN_AT_WALL_1:
                // Always wait for delay to finish
                if(delay_done()) { // (timer.c)
                    // Read the distance directly through ultrasonic sensor
                    if (bugDist + 10 > getObstacleDistanceCm()) { // (ultrasonic_sensor.c)
                        // If distance is greater than last seen distance (plus fudge factor) then continue turning at wall
                        setServoAngle(90); // (servo.c)
                        setDelay_ms(300); // Wait a bit for servo to turn (timer.c)
                        bugState = BUG_TURN_AT_WALL_2;
                    } else {
                        // The distance is less than the last seen distance, we must be parallel to wall (or close to it)
                        bugState = BUG_FOLLOW_WALL;
                    }
                }
                break;

            // At Wall, turn slightly to right to get close to parallel
            case BUG_TURN_AT_WALL_2:
                // Always wait for delay to finish
                if(delay_done()) { // (timer.c)
                    // Turn right a small amount
                    setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                    rotateRight(); // (motor.c)
                    setDelay_ms(100); // We will turn right for this amount of time then stop (timer.c)
                    // Transition to next state
                    bugState = BUG_TURN_AT_WALL_3;
                }
                break;

            // At Wall, recording distance then adjusting servo before comparing
            case BUG_TURN_AT_WALL_3:
                // Always wait for delay to finish
                if (delay_done()) { // (timer.c)
                    // Stop turning
                    stop(); // (motor.c)
                    // Read the distance directly through ultrasonic sensor, store it in bugDist variable (last distance seen)
                    bugDist = getObstacleDistanceCm(); // (ultrasonic_sensor.c)
                    // Turn servo to middle left
                    setServoAngle(45); // (servo.c)
                    setDelay_ms(300); // (timer.c)
                    // Transition to next state
                    bugState = BUG_TURN_AT_WALL_1;
                }
                break;

            case BUG_FOLLOW_WALL:
                // Turn servo sweep back on and return to moving forward (initial state)
                setServoAngle(0); // (servo.c)
                servoSweepOn(); // (servo.c)
                setDelay_ms(300); // Delay a bit before starting again (timer.c)
                // Transition to next state
                bugState = BUG_FORWARD_0;
                break;
        }
    } else {
        // Stop the robot
        stop(); // (motor.c)
        stopped = 2; // Set the forced stop flag
    }

    // Maintain that we are in autonomous obstacle avoidance mode
    return ROBOT_MODE_AUTONOMOUS_AVOID;
}




// The main robot task code for the autonomous home return mode
uint8_t robot_autonomous_home_task(double correction, float heading, uint8_t* straight){
    // Add correction to account for drift bias
    correction=-correction;
    if (correction > 0) { // Positive correction means drifted right
        // Increase right bias to adjust towards left
        rightBias = correction;
        leftBias = 0.0;
    } else if (correction < 0) { // Negative correction means drifted left
        // Increase left bias to adjust towards right
        leftBias = -correction;
        rightBias = 0.0;
    }

    if ((BT_has_pkt()&2)!=0) PORTB|=1<<PORTB5; // If packet overflow, turn on LED (BT.c)
    if (((BT_has_pkt()&1)==1)) { // If a packet is received (BT.c)
        BT_packet_t pkt = BT_get_pkt(); // Get the received packet (BT.c)

        // Handle receive STOP command
        if (pkt.pkt_type == PKT_STOP) {
            // Stop moving, including servo
            stop(); // (motor.c)
            servoSweepOff(); // (servo.c)
            // Make sure speeds are reset to zero
            stopped = 2; // Set flag to indicate forced stop
       } else {
            uint8_t nextMode = handleOtherPKT(pkt, correction, heading);
            if (nextMode) // If we updated our mode, quit from current mode and return the change
                return nextMode;
        }

    } else {
        // Stop the robot
        stop(); // (motor.c)
        stopped = 2; // Set the forced stop flag
    }

    // Get the position of the robot and record distance from axis (magnitude of position)
    int x = getIntPositionX();
    int abs_x = (x < 0)? -x : x;
    int y = getIntPositionY();
    int abs_y = (y < 0)? -y : y;

    // If not in forced stop state
    if (stopped != 2) {

        // Home finding state transition (incomplete algorithm, does not move around obstacles, requires taped coordinate grid on ground)
        switch (searchState) {

            // The initial state, find out where we need to go
            case SEARCH_START:
                // Always wait for delay to finish
                if (delay_done() && (x!=0 || y!=0)) { // (timer.c)
                    // Go the direction where we have the most to go
                    if (abs_x > abs_y) {
                        if (x > 0) { // For positive x, go down
                            searchState = SEARCH_TURN_LEFT;
                        } else { // For positive x, go down
                            searchState = SEARCH_TURN_RIGHT;
                        }
                    } else {
                        if (y > 0) { // For positive y, go down
                            searchState = SEARCH_TURN_DOWN;
                        } else { // For positive y, go down
                            searchState = SEARCH_TURN_UP;
                        }
                    }
                }
                break;

            // Turn until we are facing the up direction
            case SEARCH_TURN_UP:
                // Always wait for delay to finish
                if (delay_done() && getDirection() != DIRECTION_UP) { // (timer.c), (line_sensor.c)
                    // As long as we are not facing up, turn right
                    setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                    rotateRight(); // (motor.c)
                    setDelay_ms(600); // (timer.c)
                    updateDirection(DIRECTION_RIGHT); // (line_sensor.c)
                    // Transition to next state
                    searchState = SEARCH_WAIT_UP;
                } else if (delay_done() && getDirection() == DIRECTION_UP) { // (timer.c), (line_sensor.c)
                    // We are facing the correct direstion, so stop
                    stop(); // (motor.c)
                    setDelay_ms(2000); // (timer.c)
                    // Record last measured position before moving
                    lastX = x;
                    lastY = y;
                    // Transition to next state
                    searchState = SEARCH_UP;
                }
                break;

            // Wait a bit before continuing search
            case SEARCH_WAIT_UP:
                // Always wait for delay to finish
                if (delay_done()) {
                    stop(); // (motor.c)
                    setDelay_ms(2000); // (timer.c)
                    // Transition to next state
                    searchState = SEARCH_TURN_UP;
                }
                break;

            // Turn until we are facing the right direction
            case SEARCH_TURN_RIGHT:
                // Always wait for delay to finish
                if (delay_done() && getDirection() != DIRECTION_RIGHT) { // (timer.c), (line_sensor.c)
                    // As long as we are not facing right, turn right
                    setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                    rotateRight(); // (motor.c)
                    setDelay_ms(600); // (timer.c)
                    updateDirection(DIRECTION_RIGHT);
                    // Transition to next state
                    searchState = SEARCH_WAIT_RIGHT;
                } else if (delay_done() && getDirection() == DIRECTION_RIGHT) { // (timer.c), (line_sensor.c)
                    // We are facing the correct direstion, so stop
                    stop(); // (motor.c)
                    setDelay_ms(2000); // (timer.c)
                    // Record last measured position before moving
                    lastX = x;
                    lastY = y;
                    // Transition to next state
                    searchState = SEARCH_RIGHT;
                }
                break;

            // Wait a bit before continuing search
            case SEARCH_WAIT_RIGHT:
                // Always wait for delay to finish
                if (delay_done()) { // (timer.c)
                    stop(); // (motor.c)
                    setDelay_ms(2000); // (timer.c)
                    // Transition to next state
                    searchState = SEARCH_TURN_RIGHT;
                }
                break;

            // Turn until we are facing the down direction
            case SEARCH_TURN_DOWN:
                // Always wait for delay to finish
                if (delay_done() && getDirection() != DIRECTION_DOWN) { // (timer.c), (line_sensor.c)
                    // As long as we are not facing down, turn right
                    setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                    rotateRight(); // (motor.c)
                    setDelay_ms(600); // (timer.c)
                    updateDirection(DIRECTION_RIGHT);
                    // Transition to next state
                    searchState = SEARCH_WAIT_DOWN;
                } else if (delay_done() && getDirection() == DIRECTION_DOWN) { // (timer.c), (line_sensor.c)
                    // We are facing the correct direstion, so stop
                    stop(); // (motor.c)
                    setDelay_ms(2000); // (timer.c)
                    // Record last measured position before moving
                    lastX = x;
                    lastY = y;
                    // Transition to next state
                    searchState = SEARCH_DOWN;
                }
                break;

            // Wait a bit before continuing search
            case SEARCH_WAIT_DOWN:
                // Always wait for delay to finish
                if (delay_done()) { // (timer.c)
                    stop(); // (motor.c)
                    setDelay_ms(2000); // (timer.c)
                    // Transition to next state
                    searchState = SEARCH_TURN_DOWN;
                }
                break;

            // Turn until we are facing the left direction
            case SEARCH_TURN_LEFT:
                // Always wait for delay to finish
                if (delay_done() && getDirection() != DIRECTION_LEFT) { // (timer.c), (line_sensor.c)
                    // As long as we are not facing left, turn right
                    setSpeed((MAX_SPEED+MIN_SPEED)*.4, (MAX_SPEED+MIN_SPEED)*.4);
                    rotateRight(); // (motor.c)
                    setDelay_ms(600); // (timer.c)
                    updateDirection(DIRECTION_RIGHT);
                    // Transition to next state
                    searchState = SEARCH_WAIT_LEFT;
                } else if (delay_done() && getDirection() == DIRECTION_LEFT) { // (timer.c), (line_sensor.c)
                    // We are facing the correct direstion, so stop
                    stop(); // (motor.c)
                    setDelay_ms(2000); // (timer.c)
                    // Record last measured position before moving
                    lastX = x;
                    lastY = y;
                    // Transition to next state
                    searchState = SEARCH_LEFT;
                }
                break;

            // Wait a bit before continuing search
            case SEARCH_WAIT_LEFT:
                // Always wait for delay to finish
                if (delay_done()) { // (timer.c)
                    stop(); // (motor.c)
                    setDelay_ms(2000); // (timer.c)
                    // Transition to next state
                    searchState = SEARCH_TURN_LEFT;
                }
                break;

            // Move parallel to y-axis
            case SEARCH_UP:
            case SEARCH_DOWN:
                // Read the distance through servo sweep
                if (servoSweepDistanceTask() < 30) { // (servo.c)
                    // Stop if we see an obstacle
                    stop(); // (motor.c)
                // Always wait for delay to finish
                } else if (delay_done() && y == lastY && y != 0) {
                    // Move forward if we have not changed positions and see no obstacle
                    setSpeed(.1+MIN_SPEED, .1+MIN_SPEED);
                    moveForward(); // (motor.c)
                } else if (delay_done() && y != lastY) {
                    // We successfully moved to next position in grid
                    setDelay_ms(400); // (timer.c)
                    // Transition to next state
                    searchState = SEARCH_OVERHEAD;
                }
                break;

            // Move parallel to x-axis
            case SEARCH_LEFT:
            case SEARCH_RIGHT:
                // Read the distance through servo sweep
                if (servoSweepDistanceTask() < 30) { // (servo.c)
                    // Stop if we see an obstacle
                    stop(); // (motor.c)
                // Always wait for delay to finish
                } else if (delay_done() && x == lastX && x != 0) { // (timer.c)
                    // Move forward if we have not changed positions and see no obstacle
                    setSpeed(.1+MIN_SPEED, .1+MIN_SPEED); // (motor.c)
                    moveForward(); // (motor.c)
                } else if (delay_done() && x != lastX) { // (timer.c)
                    // We successfully moved to next position in grid
                    setDelay_ms(400); // (timer.c)
                    // Transition to next state
                    searchState = SEARCH_OVERHEAD;
                }
                break;

            case SEARCH_OVERHEAD:
                // Read the distance through servo sweep
                if (servoSweepDistanceTask() < 30) { // (servo.c)
                    // Stop if we see an obstacle
                    stop(); // (motor.c)
                // Always wait for delay to finish
                } else if (delay_done()) { // (timer.c)
                    // We reached the home position, stop and wait a bit before reseting to initial state
                    stop(); // (motor.c)
                    setDelay_ms(2000); // (timer.c)
                    // Transition to next state
                    searchState = SEARCH_START;
                }
                break;
        }
    } else {
        stop(); // (motor.c)
    }

    // Maintain that we are in autonomous home returning mode
    return ROBOT_MODE_AUTONOMOUS_HOME;
}
