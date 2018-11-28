/*
    Chassis.cpp - Library for mechatronic component chassis.
    The chassis contains 2 DC-lego-motors which power the movement of the chassis
    and 5 sens_IR-sensors which work as internal navigation components.
    Created by Glenn Huber, 03.05.2018
    Basecode by Robert Paly
*/

#include "Arduino.h"
#include "Chassis.h"

Chassis::Chassis(int givenSpeed, float kp, float ki, float kd, int motorNumRight, int motorNumLeft, int sensorPin_0, int sensorPin_1, int sensorPin_2, int sensorPin_3, int sensorPin_4)
{
    LOG1("Initializing chassis...");
    //Initializing sensors.
    sensorPin0 = sensorPin_0;
    sensorPin1 = sensorPin_1;
    sensorPin2 = sensorPin_2;
    sensorPin3 = sensorPin_3;
    sensorPin4 = sensorPin_4;
    pinMode(sensorPin0, INPUT);
    pinMode(sensorPin1, INPUT);
    pinMode(sensorPin2, INPUT);
    pinMode(sensorPin3, INPUT);
    pinMode(sensorPin4, INPUT);
    sensor0 = 0;
    sensor1 = 0;
    sensor2 = 0;
    sensor3 = 0;
    sensor4 = 0;

    previousSensorValue[0] = 0;
    previousSensorValue[1] = 0;
    previousSensorValue[2] = 0;
    previousSensorValue[3] = 0;
    previousSensorValue[4] = 0;

    sensor[0] = 0;
    sensor[1] = 0;
    sensor[2] = 0;
    sensor[3] = 0;
    sensor[4] = 0;

    //Initializing motors.
    numRight = motorNumRight;
    numLeft = motorNumLeft;
    AFMS = Adafruit_MotorShield();
    motorRight = AFMS.getMotor(numRight);
    motorLeft = AFMS.getMotor(numLeft);
    AFMS.begin();
    speed = givenSpeed;

    //Initializing PID values.
    KP = kp;
    KI = ki;
    KD = kd;
    error = 0;
    PValue = 0;
    IValue = 0;
    DValue = 0;
    PIDValue = 0;
    previousPIDValue = 0;
    previousError = 0;
    previousI = 0;
    korr = -0.01;
    korr1 = 0;
    korr2 = 0;
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
    leftMotorSpeed1 = 0;
    rightMotorSpeed1 = 0;
    targetSpeedLeft = 0;
    targetSpeedRight = 0;

    //Initializing timevariables
    millisOfDriving = 0;
    previousMillis = 0;
    timeLoopCounter = 0;

    //Initializing counter
    turnCounter = 0;
    crossCounterLeft = 0;
    crossCounterRight = 0;

    LOG3(" complete!");
}

void Chassis::loop(bool sens_IRR, int speedd, String directionn, int direcErrr)
{
    if (sens_IRR == false)
    {
        sens_IR = false;
        sens_IRR = true;
    }
    speed = speedd;
    direc = directionn;
    readSensor();

    direcErr = direcErrr;
    sensor[0] = sensor[0];
    sensor[1] = sensor[1];
    sensor[2] = sensor[2];
    sensor[3] = sensor[3];
    sensor[4] = sensor[4];

    calculatePID();

    state->PidValue = ppidValue;

    if (actionDone == true)
    {
        state->actionDone = actionDone;
        actionDone = false;

        LOG3("actionDone: "+String(actionDone));
    }
}

void Chassis::readSensor()
{
    //Read sensor values

    LOG3("sensorreading");
    sensor0 = digitalRead(sensorPin0);
    sensor1 = digitalRead(sensorPin1);
    sensor2 = digitalRead(sensorPin2);
    sensor3 = digitalRead(sensorPin3);
    sensor4 = digitalRead(sensorPin4);

    previousSensorValue[0] = sensor[0];
    previousSensorValue[1] = sensor[1];
    previousSensorValue[2] = sensor[2];
    previousSensorValue[3] = sensor[3];
    previousSensorValue[4] = sensor[4];

    LOG3("previousValues: ");

    LOG3(previousSensorValue[0]);

    LOG3(previousSensorValue[1]);

    LOG3(previousSensorValue[2]);

    LOG3(previousSensorValue[3]);

    LOG3(previousSensorValue[4]);

    sensor[0] = sensor0;
    sensor[1] = sensor1;
    sensor[2] = sensor2;
    sensor[3] = sensor3;
    sensor[4] = sensor4;

    LOG3("actualValues: ");

    LOG3(sensor[0]);

    LOG3(sensor[1]);

    LOG3(sensor[2]);

    LOG3(sensor[3]);

    LOG3(sensor[4]);

    //If no fullline is detected let the sensors read the lines for normal motorcontrol
    if (currentState.fullLine == false)
    {

        LOG3("Normal sensorreading");
        if ((sensor0 == 0) && (sensor1 == 0) && (sensor2 == 0) && (sensor3 == 0) && (sensor4 == 1))
            error = 4;
        else if ((sensor0 == 0) && (sensor1 == 0) && (sensor2 == 0) && (sensor3 == 1) && (sensor4 == 1))
            error = 3;
        else if ((sensor0 == 0) && (sensor1 == 0) && (sensor2 == 0) && (sensor3 == 1) && (sensor4 == 0))
            error = 2;
        else if ((sensor0 == 0) && (sensor1 == 0) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 0))
            error = 1;
        else if ((sensor0 == 0) && (sensor1 == 0) && (sensor2 == 1) && (sensor3 == 0) && (sensor4 == 0))
            error = 0;
        else if ((sensor0 == 0) && (sensor1 == 1) && (sensor2 == 1) && (sensor3 == 0) && (sensor4 == 0))
            error = -1;
        else if ((sensor0 == 0) && (sensor1 == 1) && (sensor2 == 0) && (sensor3 == 0) && (sensor4 == 0))
            error = -2;
        else if ((sensor0 == 1) && (sensor1 == 1) && (sensor2 == 0) && (sensor3 == 0) && (sensor4 == 0))
            error = -3;
        else if ((sensor0 == 1) && (sensor1 == 0) && (sensor2 == 0) && (sensor3 == 0) && (sensor4 == 0))
            error = -4;
        else if ((sensor0 == 1) && (sensor1 == 1) && (sensor2 == 1) && (sensor3 == 0) && (sensor4 == 0)) //90°-Kurve
            error = -10;
        else if ((sensor0 == 0) && (sensor1 == 0) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 1)) //90°-Kurve
            error = 10;
        else if ((sensor0 == 1) && (sensor1 == 1) && (sensor2 == 1) && (sensor3 == 1) && (sensor4 == 1))
        {
            currentState.fullLine = true;
        }
        //Conditions for lost navigation lines
        else if ((sensor0 == 0) && (sensor1 == 0) && (sensor2 == 0) && (sensor3 == 0) && (sensor4 == 0))
        {
            if (error == -4)
                error = -5; //If outer sensor loses line, turn hard.
            else if (error == 4)
                error = 5;
        }
    }
    //Conditions for fulllines
    if (currentState.fullLine == true)
    {
        if ((((sensor0 == 0) && (sensor4 == 0)) || (currentState.dsens_IRection == "stop")) && (sens_IR == true))
        {
            currentState.fullLine = false;

            LOG3("changing actiondone because of fulline");
            actionDone = true;
        }
        if (currentState.dsens_IRection == "right")
        {
            error = 5;

            LOG3("Error: Cross");

            LOG3(error);
        }
        if (currentState.dsens_IRection == "straight")
        {
            error = 0;

            LOG3("Error: Cross");

            LOG3(error);
        }
        if (currentState.dsens_IRection == "left")
        {
            error = -5;

            LOG3("Error: Cross");

            LOG3(error);
        }
        if (currentState.dsens_IRection == "stop")
        {
            error = 0;

            LOG3("error = 0");
        }
    }
    dsens_IRectionError = error;
}

void Chassis::calculatePID()
{
    //previousPIDValue = PIDValue;
    PValue = error;
    IValue = IValue + error;
    DValue = PIDValue - previousPIDValue;
    IValue = constrain(IValue, -300, 300);

    //Print out values of the controller
    LOG3(" I= " + String(IValue) + " P= " + String(PValue) + " D= " + String(DValue));
    PIDValue = (KP * PValue) + (KI * IValue) * (KD * DValue);
    previousError = error;
    if ((sensor0 == 0) && (sensor1 == 0) && (sensor2 == 1) && (sensor3 == 0) && (sensor4 == 0))
    {
        if ((IValue > 10) || (IValue < -10))
        {
            IValue = 0.95 * IValue;
        }
        else
        {
            IValue = 0;
        }
    }
    ppidValue = PIDValue;

    LOG3("PID = " + String(PIDValue));
}

void Chassis::motorControl(float sonarFactor)
{
    if (speed == 0)
    {

        LOG3("stopping inside motorcontrol");
        stop();
    }
    else
    {

        LOG3("Motorcontrol");
        korr2 = 1 - korr;
        korr1 = 1 + korr;

        leftMotorSpeed = speed + ppidValue;
        rightMotorSpeed = speed - ppidValue;
        leftMotorSpeed1 = constrain(leftMotorSpeed, 0, speed + 50);
        rightMotorSpeed1 = constrain(rightMotorSpeed, 0, speed + 50);

        //Calculate the speed for both motors individualy
        LOG3("SonarFactor inside MotorControl: " + String(sonarFactor));
        targetSpeedLeft = sonarFactor * korr2 * leftMotorSpeed1;
        targetSpeedRight = sonarFactor * korr1 * rightMotorSpeed1;

        motorLeft->run(FORWARD);
        motorRight->run(FORWARD);

        //Startspeed settings
        motorLeft->setSpeed(getStartSpeedLeft());
        motorRight->setSpeed(getStartSpeedRight());

        //Drivespeed settings
        motorLeft->setSpeed(targetSpeedLeft);
        motorRight->setSpeed(targetSpeedRight);
        LOG3("Speed of motors: " + String(targetSpeedLeft) + " || " + String(targetSpeedRight));
    }
}

int Chassis::getStartSpeedLeft()
{
    if (targetSpeedLeft < 30)
    {
        return 0;
    }
    else if ((targetSpeedLeft >= 30) && (targetSpeedLeft < 60))
    {
        return 100;
    }
    else
    {
        return 60;
    }
}

int Chassis::getStartSpeedRight()
{
    if (targetSpeedRight < 30)
    {
        return 0;
    }
    else if ((targetSpeedRight >= 30) && (targetSpeedRight < 60))
    {
        return 100;
    }
    else
    {
        return 60;
    }
}

void Chassis::stop()
{
    motorLeft->setSpeed(0);
    motorRight->setSpeed(0);
    LOG3("Vehicle stopped");
}

void Chassis::driveBack()
{
    motorLeft->run(BACKWARD);
    motorRight->run(BACKWARD);
    motorLeft->setSpeed(PUSH_SPEED);
    motorRight->setSpeed(PUSH_SPEED);
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
}

void Chassis::driveBackLimited(unsigned int driveTime, unsigned long startTime)
{
    unsigned long actualMillis = millis();
    millisOfDriving = actualMillis - startTime;
    if (millisOfDriving > driveTime)
    {
        motorLeft->setSpeed(0);
        motorRight->setSpeed(0);
        actionDone = true;
        LOG3("Changing actiondone because of driveback");
        sens_IR = true;
    }
    else
    {
        sens_IR = false;
        motorLeft->run(BACKWARD);
        motorRight->run(BACKWARD);
        motorLeft->setSpeed(PUSH_SPEED);
        motorLeft->setSpeed(speed);
        motorRight->setSpeed(PUSH_SPEED);
        motorRight->setSpeed(speed);
        LOG3("Driving back");
    }
}

void Chassis::driveStraightLimited(unsigned int driveTime, unsigned long startTime)
{
    unsigned long actualMillis = millis();
    millisOfDriving = actualMillis - startTime;
    LOG3("millifOfDriving: ");
    LOG3(millisOfDriving);
    if (millisOfDriving > driveTime)
    {
        motorRight->setSpeed(0);
        motorLeft->setSpeed(0);
        actionDone = true;
        LOG3("Changing actiondone because of drivestraight");
        LOG3("ifcase inside drive straight");
        sens_IR = true;
    }
    else
    {
        sens_IR = false;
        motorLeft->run(FORWARD);
        motorRight->run(FORWARD);
        motorLeft->setSpeed(PUSH_SPEED);
        motorRight->setSpeed(PUSH_SPEED);
        motorLeft->setSpeed(speed);
        motorRight->setSpeed(speed);
        LOG3("elsecase inside drive straight");
    }
}

void Chassis::driveStraight()
{
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
    motorLeft->setSpeed(PUSH_SPEED);
    motorRight->setSpeed(PUSH_SPEED);
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
}

void Chassis::turnLeftLimited(unsigned int curveTime, unsigned long startTime)
{
    motorLeft->setSpeed(0);
    unsigned long actualMillis = millis();
    millisOfDriving = actualMillis - startTime;
    LOG3("millifOfDriving: ");
    LOG3(millisOfDriving);
    if (millisOfDriving > curveTime)
    {
        motorRight->setSpeed(0);
        actionDone = true;
        LOG3("Changing actiondone because of turnleft");
        LOG3("ifcase inside turn left");
        sens_IR = true;
    }
    else
    {
        sens_IR = false;
        motorRight->run(FORWARD);
        motorRight->setSpeed(PUSH_SPEED);
        motorRight->setSpeed(speed);
        LOG3("elsecase inside turn left");
    }
}

void Chassis::turnLeft()
{
    motorLeft->run(RELEASE);
    motorRight->run(FORWARD);
    motorRight->setSpeed(PUSH_SPEED);
    motorRight->setSpeed(speed);
    LOG3("Turnleft");
}

void Chassis::turnRightLimited(unsigned int curveTime, unsigned long startTime)
{
    motorRight->setSpeed(0);
    unsigned long actualMillis = millis();
    millisOfDriving = actualMillis - startTime;
    LOG3("millifOfDriving: ");
    LGO3(millisOfDriving);
    if (millisOfDriving > curveTime)
    {
        motorLeft->setSpeed(0);
        actionDone = true;
        LOG3("Changing actiondone because of turnright");
        LOG3("ifcase inside turn right");
        sens_IR = true;
    }
    else
    {
        sens_IR = false;
        motorLeft->run(FORWARD);
        motorLeft->setSpeed(PUSH_SPEED);
        motorLeft->setSpeed(speed);
        LOG3("elsecase inside turn right");
    }
}

void Chassis::turnRight()
{
    motorRight->run(RELEASE);
    motorLeft->run(FORWARD);
    motorLeft->setSpeed(PUSH_SPEED);
    motorLeft->setSpeed(speed);
    LOG3("Turnright");
}

void Chassis::turnAround(int sensor)
{
    int i = sensor;
    LOG3("Inside turnaround");
    if (actionDone == false)
    {
        motorLeft->run(BACKWARD);
        motorRight->run(FORWARD);
        motorLeft->setSpeed(PUSH_SPEED);
        motorLeft->setSpeed(speed);
        motorRight->setSpeed(PUSH_SPEED);
        motorRight->setSpeed(speed);
        LOG3("Turnaround");
        LOG3("turnCounter: " + String(turnCounter));
        LOG3("currentsensor: " + String(sensor[i]);
        LOG3("Sensor: " + String(i));
        sens_IR =false;
    }
    if ((sensor[i] == 0) && (turnCounter == 0))
    {
        turnCounter++;
        LOG3("turncounter incrementing");
    }
    if ((previousSensorValue[i] == 0) && (sensor[i] == 1) && (turnCounter == 1))
    {
        actionDone = true;
        LOG3("Changing actiondone because of turnaround");
        sens_IR = true;
        turnCounter = 0;
        motorLeft->setSpeed(0);
        motorRight->setSpeed(0);
        LOG3("Turnaround complete");
    }
}