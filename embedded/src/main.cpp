// Smart Box main file

#include <Arduino.h>
#include <ArduinoJson.h>
//#include "SPI.h"
// own files:
#include <NetworkManager.h>
#include <MainConfiguration.h>
#include <NetworkManagerStructs.h>
#include <LogConfiguration.h>
#include <Sonar.h>
#include <Chassis.h>
#include <Hoist.h>
#include <Vision.h>
#include <SmartBoxList.h>

// ===================================== Global Variables =====================================
myJSONStr *JSarra;          // used in NetworkManager.h, used for saving incoming Messages, FIFO order, see also MAX_JSON_MESSAGES_SAVED
NetworkManager *mNetwP = 0; // used for using NetworkManager access outside setup()
Sonar *vehicleSonar;        // used for Sonar access
Vision *vehicleVision;      // used for Vision access
Hoist *vehicleHoist;        // used for Hoist access
Chassis *vehicleChassis;    // used for Chassis access
VehicleWebAPI *vehicleAPI;  // used for Sonar access
bool hasAnswered = false;   // variable used to see if Vehicle have answered
bool publishParams = false; // if SmartBox requests Vehicle to answer
bool publishAck = false;
myJSONStr *tmp_mess; // pointer to array of messages, used for iteration of messages
MQTTTasks *TaskMain; // filled in NetworkManager.cpp, used for saving incoming Messages, FIFO order
SmartBoxList SB_hostnames;

// -.-.-.-.-.-.-.-.-.-.-.-.-.- used for Statuses -.-.-.-.-.-.-.-.-.-.-.-.-.-
enum status_main // stores main status for Program run (main.cpp)
{
    status_noTask = 0,
    status_hasRequest = 1,
    status_handleRequest = 2,
    status_transporting = 3,
    status_transported = 4,
    stauts_requestHandeled = 5
};
int mcount = 0; // needed for number of messages received, lower num
int mcount2 = 0;
status_main stat = status_main::status_noTask;
bool toNextStatus = true; // true if changing state, false if staying in state, it's enshuring that certain code will only run once
int messageCounter = 0;
int numOfTasksToDo = 0;
myJSONStr currentTask;
void (*myFuncToCall)() = nullptr; // func to call in main-loop, for finite state machine

// ===================================== my helper Functions =====================================

void getVehicleInfo(){} // print Smart Box Information TODO
{
    LOG2("Vehicle state: " + String(stat));
    LOG2("Variable has answered: " + String(hasAnswered));
    LOG2("Variable mcount: " + String(mcount));
    LOG2("Variable mcount2: " + String(mcount2));
    LOG2("Variable toNextStatus: " + String(toNextStatus));
    LOG2("Variable messageCounter: " + String(messageCounter));
    LOG2("Variable numOfTasksToDo: " + String(numOfTasksToDo));
    LOG2("-------- current Task --------");
    LOG2("current Task (hostname): " + String(currentTask.hostname));
    LOG2("current Task (topic): " + String(currentTask.topic));
    LOG2("current Task (request): " + String(currentTask.request));
    LOG2("current Task (level): " + String(currentTask.level));
};

// ===================================== my Functions =====================================

void loopNoTask()
{
    if (toNextStatus)
    {
        LOG1("entering new state: loopNoTask");
        mcount = TaskMain->returnCurrentIterator();
        mcount2 = TaskMain->returnCurrentIterator();
        toNextStatus = false;
    }
    mNetwP->loop();
    mcount2 = TaskMain->returnCurrentIterator();
    if (mcount2 > mcount)
    {
        toNextStatus = true;
        LOG2("Messages arrived");
        stat = status_main::status_hasRequest;
        myFuncToCall = loopTask;
    }
    else if (mcount2 == mcount)
    {
        mNetwP->loop();
        LOG3("nothing to do");
    }
    else
        LOG1("something is wrong... ");
};

void loopTask() // loops through all unhandeled tasks
{
    LOG1("entering new state: loopTask");
    mNetwP->loop();
    mcount2 = TaskMain->returnCurrentIterator();
    tmp_mess = TaskMain->getBetween(mcount, mcount2);
    numOfTasksToDo = (sizeof(tmp_mess) / sizeof(tmp_mess[0]));
    if (tmp_mess == nullptr)
    {
        LOG2("no messages");
        stat = status_main::status_noTask; // jump back to noTask status
        myFuncToCall = loopNoTask;
    }
    else
    {
        LOG3("num of tasks: " + String(numOfTasksToDo));
        currentTask = tmp_mess[0]; // do first tasks
        mcount++;
        stat = status_main::status_handleRequest;
        LOG3("going to do next task");
        String toparr[] = TaskMain->returnMQTTtopics(currentTask); // array indexes: 0: SmartBox, 1: SmartBox_ID, 2: level/decision
        if ((toparr[0] == "SmartBox") && (toparr[2] == "level"))   // if SmartBox/+/level
        {
            publishParams = true;
            SB_hostnames.push(toparr[1]); // save hostname
        }
        else if ((toparr[0] == "SmartBox") && SB_hostnames.isInList(toparr[1]) && (toparr[2] == "decision")) // if SmartBox/+/decision
        {
            SB_hostnames.pop(toparr[1]);
            if (SB_hostnames.getSize() == 0)
                publishParams = false;
            mNetwP->publishMessage("Vehicle/" + mNetwP->getHostName() + "/ack", "{hostname:" + toparr[1] + "}"); // acknoledge message received and coming for transport
            stat = status_main::status_transporting;
            myFuncToCall = transportBox1;
        }
        else
        {
            LOG3("topic: [" + currentTask.topic + "] unknown, and therefore not treated here\t\t hostname: " + currentTask.hostname + "\t Request: " + currentTask.request);
        }
    }
};

void transportBox1() // used to transport the Smart Box, 1 is for Setup from Luciano Bettinaglio, can be editted for other Setups
{
    //transport box TODO -> see other file to get right commands
    // do move until condition set, call checkForUrgendTasks on regular basis
    // TODO: möglichst schneller Durchlauf!
    mNetwP->loop();
    if (checkForUrgendTasks)
        ; // TODO if urgent task arrived

    // if transported:
    mNetwP->publishMessage("Vehicle/" + mNetwP->getHostName() + "/ack", "{request:" + toparr[1] + "}"); // publish acknoledgement transported to Vehicle/...ID.../ack
    stat = status_main::status_hasRequest;
    myFuncToCall = loopTask;
};

bool checkForUrgendTasks() // during transport checks if something is very urgent, true if urgent task arrived
{
    return false;
    // TODO
};

// ===================================== Arduino Functions =====================================
void setup() // for initialisation
{
    if (log_level > 0)
    {
        Serial.begin(12000); //Initialize serial
        while (!Serial)
            ; // wait for serial port to connect
    }
    mNetwP = new NetworkManager();
    vehicleSonar = new Sonar(SONAR_SERVO_PIN, SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE, MIN_ERROR, MAX_ERROR, MIN_TURN_ANGLE, MAX_TURN_ANGLE);
    vehicleVision = new Vision(VISION_START_ANGLE, VISION_SERVO_PIN, VISION_DELAY_FACTOR, VISION_TOLERANCE_LEFT, VISION_TOLERANCE_RIGHT);
    vehicleHoist = new Hoist(HOIST_SERVO_PIN, HOIST_SERVO_DELAY, HOIST_POISITION_MAX, HOIST_POSITION_MIN);

    // TODO: #define isVehicle true/false -> verschiedene main.cpp laden
    JSarra = mNetwP->JSarrP;
    mNetwP->subscribe("SmartBox/+/level");
    mNetwP->subscribe("SmartBox/+/decision");
    TaskMain = mNetwP->NetManTask_classPointer;
    myFuncToCall = loopNoTask;

    if (true) // for debugging purpose, DELETE ON FINAL TODO
    {
        pinMode(13, OUTPUT); // debug LED
    }
}

void loop() // one loop per one cycle (SB full -> transported -> returned empty)
{
    if (true) // degug cycle -- DELETE ON FINAL TODO
    {
        digitalWrite(13, LOW);
        delay(1000);
        digitalWrite(13, HIGH);
        delay(1000);
    }

    myFuncToCall();

    mNetwP->loop();
    if (publishParams)
        mNetwP->publishMessage("Vehicle/" + mNetwP->getHostName() + "/params", "{params:[100,100" + String(numOfTasksToDo) + "] }"); // publish params [distance, velocity, tasks]
    // TODO: define above signal/array list
    // TODO: params in callback2 of SmartBox???

    /*
    if (publishAck)
    {
        for(int i=0; i<=SB_hostnames_it; i++)
        {
            mNetwP->publishMessage("Vehicle/" + mNetwP->getHostName() + "/ack", "{hostname:" + SB_hostnames[i] + "}");
        }
    }
    */
}
