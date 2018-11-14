// Smart Box main file

#include <Arduino.h>
#include <ArduinoJson.h>
//#include "SPI.h"
// own files:
#include <NetworkManager.h>
#include <MainConfiguration.h>
#include <NetworkManagerStructs.h>
#include <Sonar.h>
#include <Chassis.h>
#include <Hoist.h>
#include <Vision.h>

// ===================================== Global Variables =====================================
myJSONStr *JSarra;                   // used in NetworkManager.h, used for saving incoming Messages, FIFO order, see also MAX_JSON_MESSAGES_SAVED
int my_json_counter = 0;             // is last element in array, used for referencing to the last Element, attention: pay attention to out of bound see MAX_JSON_MESSAGES_SAVED, DON'T TOUCH THIS: https://www.youtube.com/watch?v=otCpCn0l4Wo
bool my_json_counter_isEmpty = true; // used in NetworkManager.h
NetworkManager *mNetwP = 0;          // used for using NetworkManager access outside setup()
Sonar *vehicleSonar;                 // used for Sonar access
Vision *vehicleVision;               // used for Vision access
Hoist *vehicleHoist;                 // used for Hoist access
Chassis *vehicleChassis;             // used for Chassis access
VehicleWebAPI *vehicleAPI;           // used for Sonar access
const int log_level = 1;             // can have values from 0-3
bool hasAnswered = false;            // variable used to see if Vehicle have answered
bool publishParams = false;          // if SmartBox requests Vehicle to answer
byte isLastRoundonError = 1;         // currently two max values are included, if both are not responding, this Variable will be set to true, must be min 1
enum SBLevel                         // describes Smart Box level states, -5 is default if not set!
{
    full = 0,
    empty = 1
};
void (*myFuncPtr)(int) = NULL; // Pointer for the following Functions: ... TODO

// ===================================== Function Headers of my helper Functions =====================================
void transportBox1();
void handleRequest(int ii);
int returnNumOfTaksksToDo();
// ===================================== my helper Functions =====================================
String *returnMQTTtopics(String top) // returns String-Array of topics from MQTT topic structure, strings divided by /
{
    String tmp[MAX_MQTT_TOPIC_DEPTH];
    int k1 = 0; // lower cut-bound
    int k2 = 0; // upper cut-bound
    int k3 = 0; // num of strings (must be below above!)
    for (int i = 0; i < top.length(); i++)
    {
        if (top.charAt(i) == '/')
        {
            k1 = i + 1;
            if (k3 == MAX_MQTT_TOPIC_DEPTH)
                break;
            else
            {
                tmp[k3] = top.substring(k1, k2);
                k3++;
            }
        }
        else
        {
            k2++;
        }
    }
    String tmp2[k3];
    for (int i = 0; i < k3; i++)
    {
        tmp2[i] = tmp[i];
    }
    return tmp2;
};

void transportBox1() // used to transport the Smart Box, 1 is for Setup from Luciano Bettinaglio, can be editted for other Setups
{
    // TODO
}

int returnNumOfTasksToDo()
{
    // TODO
}

void handleRequest(int ii) // handles Messages based on topic
{
    String toparr[] = returnMQTTtopics(JSarra[ii].topic);                     // 0: SmartBox, 1: SmartBox_ID, 2: level/decision
    if ((toparr[0] == "SmartBox") && (toparr[2] == "level") && publishParams) // if SmartBox/+/level
    {
        mNetwP->publishMessage("Vehicle/" + mNetwP->getHostName() + "/params", "{params:[100,100" + String(returnNumOfTasksToDo()) + "] }"); // publish params [distance, velocity, tasks]
        // TODO: define above signal/array list
        // TODO: params in callback2 of SmartBox???
    }
    else if ((toparr[0] == "SmartBox") && (toparr[2] == "decision")) // if SmartBox/+/decision
    {
        // TODO check if decision for right request was made!
        mNetwP->publishMessage("Vehicle/" + mNetwP->getHostName() + "/ack", "{hostname:" + toparr[1] + "}");
        publishParams = false;
        transportBox1();                                                                                    // transport Smart Box
        mNetwP->publishMessage("Vehicle/" + mNetwP->getHostName() + "/ack", "{request:" + toparr[1] + "}"); // publish acknoledgement transported to Vehicle/...ID.../ack
    }
    else
    {
        Serial.println("topic: [" + JSarra[ii].topic + "] unknown, and therefore not treated here\t\t hostname: " + JSarra[ii].hostname + "\t Request: " + JSarra[ii].request);
    }
}

// void getSmartBoxInfo(){}; // print Smart Box Information TODO

// ===================================== my Functions =====================================
// -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.- wait for Answer (which vehicles are there?)

void loopNoTask()
{
    int mcount = my_json_counter; // needed for number of messages received during checks
    while (mcount == my_json_counter)
    {
        mNetwP->loop();
        delay(1000);
    }
    mNetwP->loop();
};

void loopTask() // loops through all unhandeled tasks
{
    publishParams = true;
    int mcount = my_json_counter; // needed for number of messages received during run

    if (my_json_counter != mcount) // if new messages arriced during run
    {
        if (mcount > my_json_counter)
        {
            for (int i = my_json_counter; i < MAX_JSON_MESSAGES_SAVED; i++)
            {
                handleRequest(i);
            }
            for (int i = 0; i < my_json_counter; i++)
            {
                handleRequest(i);
            }
        }
        else // if mcount < my_json_counter
        {
            for (int i = mcount; i < my_json_counter; i++)
            {
                handleRequest(i);
            }
        }
    }
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
    // TODO: #define isVehicle true/false -> verschiedene main.cpp laden
    JSarra = mNetwP->JSarrP;
    mNetwP->subscribe("SmartBox/+/level");
    mNetwP->subscribe("SmartBox/+/decision");

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

    loopNoTask();

    loopTask();
}
