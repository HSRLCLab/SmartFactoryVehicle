/*
    Sonar.h - Library for mechatronic component sonar.
    The sonar module consists of a sonar sensor and a servo motor.
    Based on the deviation of the sonar to the focus point an error is
    given to the loop to turn the sonar to the right direction.
    Created by Glenn Huber, 03.05.2018
    Basecode by Felix Nyffenegger
*/

#ifndef sonar_h
#define sonar_h

#include "Arduino.h"
#include "Modular.h"
#include "NewPing.h"
#include "Servo.h"
#include "Configuration.h"

struct SonarState {
    int obstacleDistance = 9999;
    float sonarFactor = 1;
    bool detachServo = false;
    bool isAttached = false;
};

class Sonar : public Component
{
public:
    Sonar(int servoPin, int triggerPin, int echoPin, int maxDistance, int min_Error, int max_Error, int min_TurnAngle, int max_TurnAngle);
    void loop(SonarState *state, int directionError, bool servoActive);
    void turnSonar(int directionError);
    void calculateSonarFactor(SonarState *state);
    int readSonar();

protected:
    NewPing *sonar;
    Servo sonarServo;
    int minError;
    int maxError;
    int minTurnAngle;
    int maxTurnAngle;
    int _servoPin;
};

#endif