#ifndef MAINCONFIGURATION_H
#define MAINCONFIGURATION_H

/*
    file that contains the default configuration for the main function
    Note:
        SENSOR_* is concerning the sensor
        *ITERATION* is an iteration variable (e.g- for loops)
        *SECONDS* is stating, that the value is given in seconds and will be recalculated for milliseconds (for func delay())
*/

#define MAX_NUMBEROFSERVABLE_SMARTBOX_REQUESTS 10
#define DIST_TO_BOX 0.5   // distance to SmartBox in cm


// below is copied from Configuration.h
//Setup for chassis /////////////////
#define RIGHT_MOTOR     1
#define LEFT_MOTOR      2
#define PIN_SENSOR_0    13
#define PIN_SENSOR_1    12
#define PIN_SENSOR_2    11
#define PIN_SENSOR_3    10
#define PIN_SENSOR_4    9
#define SPEED           70
#define REDUCED_SPEED   60
#define PUSH_SPEED      80
/////////////////////////////////////

//Setup for PID /////////////////////
#define K_P  11
#define K_I  0.3
#define K_D  0
/////////////////////////////////////

//Setup for sonar ///////////////////
#define SONAR_SERVO_PIN     5
#define SONAR_TRIGGER_PIN   15
#define SONAR_ECHO_PIN      14
#define SONAR_MAX_DISTANCE  9999
#define MIN_ERROR           -5
#define MAX_ERROR           5
#define MIN_TURN_ANGLE      180
#define MAX_TURN_ANGLE      0
////////////////////////////////////

//Setup for Hoist //////////////////
#define HOIST_SERVO_PIN     6
#define HOIST_SERVO_DELAY   30
#define HOIST_POSITION_MIN  168
#define HOIST_POISITION_MAX 65
////////////////////////////////////

//Setup for Vision /////////////////
#define VISION_SERVO_PIN         5
#define VISION_DELAY_FACTOR      6
#define VISION_TOLERANCE_LEFT   165
#define VISION_TOLERANCE_RIGHT  155
#define VISION_START_ANGLE      90
////////////////////////////////////

#endif