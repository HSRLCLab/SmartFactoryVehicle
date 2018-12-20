/*
    Sonar.cpp - Library for mechatronic component sonar.
    The sonar module consists of a sonar sensor and a servo motor.
    Based on the deviation of the sonar to the focus point an error is
    given to the loop to turn the sonar to the right direction.
    Created by Glenn Huber, 03.05.2018
    Basecode by Felix Nyffenegger
*/

#include "Arduino.h"
#include "Sonar.h"

Sonar::Sonar(int servoPin, int triggerPin, int echoPin, int maxDistance, int min_Error, int max_Error, int min_TurnAngle, int max_TurnAngle)
{
    if (DEBUGGER == true)
        Serial.print("Initializing sonar...");
    sonar = new NewPing(triggerPin, echoPin, maxDistance);
    minError = min_Error;
    maxError = max_Error;
    minTurnAngle = min_TurnAngle;
    maxTurnAngle = max_TurnAngle;
    _servoPin = servoPin;
    if (DEBUGGER == true)
        Serial.println(" complete!");
}

void Sonar::loop(SonarState *state, int directionError, bool servoActive)
{
    if ((servoActive == true) && (state->isAttached == false))
    {
        sonarServo.attach(_servoPin);
        state->isAttached = true;
    }
    int uSonar = sonar->ping_cm();
    if (DEBUGGER == true)
        Serial.print("distance: ");
    if (DEBUGGER == true)
        Serial.println(uSonar);
    state->obstacleDistance = uSonar;
    if (servoActive == true)
    {
        if (DEBUGGER == true)
            Serial.print("Turn sonar with directionError: ");
        if (DEBUGGER == true)
            Serial.println(directionError);
        turnSonar(directionError);
    }
    calculateSonarFactor(state);
    if (state->detachServo == true)
    {
        sonarServo.detach();
        state->detachServo = false;
        state->isAttached = false;
    }
}

int readSonar()
{
    return sonar->ping_cm();
}

void Sonar::turnSonar(int directionError)
{
    int turnSonar = map(directionError, minError, maxError, minTurnAngle, maxTurnAngle);
    if (DEBUGGER == true)
        Serial.print("turnanglesonar: ");
    if (DEBUGGER == true)
        Serial.println(turnSonar);
    sonarServo.write(turnSonar);
}

void Sonar::calculateSonarFactor(SonarState *state)
{
    if ((state->obstacleDistance > 10) || (state->obstacleDistance == 0))
    {
        state->sonarFactor = 1;
        if (DEBUGGER == true)
            Serial.println("Factor1");
    }
    else if ((state->obstacleDistance <= 10) && (state->obstacleDistance >= 5))
    {
        state->sonarFactor = 0.5;
        if (DEBUGGER == true)
            Serial.println("Factor0.5");
    }
    else
    {
        state->sonarFactor = 0;
        if (DEBUGGER == true)
            Serial.println("Factor0");
    }
}
