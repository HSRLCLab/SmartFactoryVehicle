/*
    Vision.h - Library for mechatronic component vision.
    The vision module consists of a pixy-cam and a servo motor.
    The cam searches for target-objects and adjusts its position to that direction.
    Afterwarts it sends information for course adaption.
    Created by Glenn Huber, 05.06.2018
    Basecode by Robert Paly
*/
#ifndef vision_h
#define vision_h

#include "Arduino.h"
#include "Modular.h"
#include "Servo.h"
#include "Wire.h"
#include "PixyCam_Lib/PixyI2C.h"
#include "Configuration.h"

struct VisionState {
int target;
int servoAngle;
bool targetDetected = false;
bool reset = false;
};

class Vision : public Component
{
public:
    Vision(int startAngle, int servoPin, int dddelay, int toleranceL, int toleranceR);
    void loop(VisionState *state);
    void turnVision(int angle);
    VisionState visionState;

protected:
    
    Servo visionServo;
    PixyI2C pixy;

    int toleranceLeft;
    int toleranceRight;

    int target1;
    int target2;
    int target3;
    int target4;
    int target5;
    int target6;
    int target7;
    int target8;
    int targetFound;
    int targetFound1;
    int targetFound2;
    int targetFound3;
    int targetFound4;
    int ddelay;
    int failCounter;
    int _servoPin;
};

#endif