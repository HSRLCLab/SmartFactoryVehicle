/*
    Chassis.h - Library for mechatronic component chassis.
    The chassis contains 2 DC-motors which power the movement of the chassis
    and 5 IR-sensors which work as internal navigation components.
    Created by Glenn Huber, 03.05.2018
*/

#ifndef Chassis_h
#define Chassis_h

#include "Arduino.h"
#include "Modular.h"
#include "Adafruit_MotorShield.h"
#include "ChassisConfiguration.h"

/*
struct ChassisState
{
    int speed;
    int directionError = 0;
    int virtualError = 0;
    int sensor[5];
    float PidValue;
    bool actionDone = false;
    bool IR = true;
    bool fullLine = false;
    String direction = "";
};
*/

class Chassis : public Component
{
  public:
    Chassis();
    Chassis(int givenSpeed, float kp, float ki, float kd, int motorNumRight, int motorNumLeft, int sensorPin0, int sensorPin1, int sensorPin2, int sensorPin3, int sensorPin4);
    void loop(ChassisState *state);
    void readSensor();
    void calculatePID();
    void motorControl(float sonarFactor);
    void turnLeftLimited(unsigned int curveTime, unsigned long startTime);
    void turnLeft();
    void turnRightLimited(unsigned int curveTime, unsigned long startTime);
    void turnRight();
    void driveStraightLimited(unsigned int driveTime, unsigned long startTime);
    void driveStraight();
    void turnAround(int sensor);
    void driveBack();
    void driveBackLimited(unsigned int driveTime, unsigned long startTime);
    int getStartSpeedLeft();
    int getStartSpeedRight();
    void stop();
    ChassisState currentState;

  protected:
    //Motor objects
    Adafruit_MotorShield AFMS;
    Adafruit_DCMotor *motorRight;
    Adafruit_DCMotor *motorLeft;
    int numRight;
    int numLeft;
    bool actionDone = false;
    int speed;
    String direc;
    int direcErr;

    //Sensor variables
    int sensor[5];
    bool sens_IR = true;
    int sensor0;
    int sensor1;
    int sensor2;
    int sensor3;
    int sensor4;

    int previousSensorValue[5];

    int sensorPin0;
    int sensorPin1;
    int sensorPin2;
    int sensorPin3;
    int sensorPin4;

    //PID variables
    float KP;
    float KI;
    float KD;
    int error;
    float PValue;
    float IValue;
    float DValue;
    float PIDValue;
    float ppidValue;
    float previousPIDValue;
    float previousError;
    float previousI;
    float korr;
    float korr1;
    float korr2;
    float leftMotorSpeed;
    float rightMotorSpeed;
    float leftMotorSpeed1;
    float rightMotorSpeed1;
    float targetSpeedLeft;
    float targetSpeedRight;

    //Timevariables
    unsigned long millisOfDriving;
    unsigned long previousMillis;
    int timeLoopCounter;

    //Counter
    int turnCounter;
    int crossCounterLeft;
    int crossCounterRight;
};

#endif