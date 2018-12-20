/*
    Main programm to run the transport vehicle
    Created by Glenn Huber 05.05.2018
*/

/*
#include "Arduino.h"
#include "VehicleWebAPI.h"
#include "SPI.h"
#include "Sonar.h"
#include "Chassis.h"
#include "Hoist.h"
#include "Vision.h"

struct VehicleState {
    ChassisState chassis;
    SonarState sonar;
    VisionState vision;
    HoistState hoist;
    ApiState api;
    float curveFactor = 0;
    int turnCount = 1;
    int waitCount = 0;
    int sonarCount = 0;
    unsigned long timeStart = 0;
    bool turnSonar = true;
    bool waitForTurn = false;
    bool visual = false;
    String side = "";
} state;

Sonar *vehicleSonar;
Vision *vehicleVision;
Hoist *vehicleHoist;
Chassis *vehicleChassis;
VehicleWebAPI *vehicleAPI;

void strategy() {
    if (DEBUGGER == true) Serial.print("Sector begin strategy"); if (DEBUGGER == true) Serial.println(state.api.sector);
    if (DEBUGGER == true) Serial.print("VehicleCount: "); if (DEBUGGER == true) Serial.println(state.api.vehicleCount);
    if (DEBUGGER == true) Serial.print("JobCount: "); if (DEBUGGER == true) Serial.println(state.api.jobCount);
    if (DEBUGGER == true) Serial.print("otherTask: "); if (DEBUGGER == true) Serial.println(state.api.otherTask);
    if (DEBUGGER == true) Serial.print("othersector: "); if (DEBUGGER == true) Serial.println(state.api.otherSector);
    if (DEBUGGER == true) Serial.print("Container: "); if (DEBUGGER == true) Serial.println(state.api.container);
    if (DEBUGGER == true) Serial.print("port: "); if (DEBUGGER == true) Serial.println(state.api.unloadPort);

    //After Task is finished, change the task
    if ((state.api.state == "resting") && (state.api.task != "finished")) {
        state.api.task = "finished";
    }
    //Sending the state if change happend
    if (state.api.taskChange == true) {
        if (DEBUGGER == true) Serial.println("Sending task");
        vehicleAPI->sendTask();
        if ((state.api.vehicleID == "Idefix") && (state.api.vehicleCount == 2)) {
            if (DEBUGGER == true) Serial.println("send obelix task");
        vehicleAPI->sendTaskToObelix();
        }
        if ((state.api.vehicleID == "Obelix") && (state.api.vehicleCount == 2)) {
            if (DEBUGGER == true) Serial.println("send idefix task");
            vehicleAPI->sendTaskToIdefix();
        }
        state.api.taskChange = false;
    }
    if (state.api.sectorChange == true) {
        if (DEBUGGER == true) Serial.println("Sending sector");
        vehicleAPI->sendSector();
        if ((state.api.vehicleID == "Idefix") && (state.api.vehicleCount == 2) && ((state.api.sector == "20") || (state.api.sector == "24") || (state.api.sector == "31") || (state.api.sector == "40"))) {
            if (DEBUGGER == true) Serial.println("send obelix sector");
        vehicleAPI->sendSectorToObelix();
        }
        if ((state.api.vehicleID == "Obelix") && (state.api.vehicleCount == 2) && ((state.api.sector == "20") || (state.api.sector == "24") || (state.api.sector == "31") || (state.api.sector == "40"))) {
            if (DEBUGGER == true) Serial.println("send idefix sector");
            vehicleAPI->sendSectorToIdefix();
        }
        state.api.sectorChange = false;
    }
    if (state.api.stateChange == true) {
        if (DEBUGGER == true) Serial.println("Sending state");
        vehicleAPI->sendState();
        state.api.stateChange = false;
    }

    //Sectorstrategy the workState is changed, dependend on the position of the other vehicle
    if (((state.api.sector == "10") || (state.api.sector == "11")) && (state.api.otherSector == "20")) {
        state.api.workState = false;
        state.api.workStateChange = true;
    }
    else if ((state.api.sector == "20") && (state.api.otherSector == "24")) {
        state.api.workState = false;
        state.api.workStateChange = true;
    }
    else if ((state.api.sector == "31") && (state.api.otherSector == "31")) {
        state.api.workState = false;
        state.api.workStateChange = true;
    }
    //else if ((state.api.sector == "20") && (state.api.otherSector == "22")) {
    //    state.api.workState = false;
    //    state.api.workStateChange = true;
    //}
    //else if ((state.api.sector == "30") && (state.api.otherSector == "32")) {
    //    state.api.workState = false;
    //    state.api.workStateChange = true;
    //}
    //else if ((state.api.sector == "40") && (state.api.otherSector == "20") && (state.api.vehicleID == "Idefix")) {
    //    state.api.workState = false;
    //    state.api.workStateChange = true;
    //}
    else if (((state.api.sector == "10") || (state.api.sector == "11")) && (state.api.otherSector == "24")) {
        state.api.workState = true;
        state.api.workStateChange = true;
    }
    else if ((state.api.sector == "20") && (state.api.otherSector == "31")) {
        state.api.workState = true;
        state.api.workStateChange = true;
    }
    else if ((state.api.sector == "31") && (state.api.otherSector == "40")) {
        state.api.workState = true;
        state.api.workStateChange = true;
    }
    //else if ((state.api.sector == "20") && ((state.api.otherSector == "10") || (state.api.otherSector == "22") || (state.api.otherSector == "32") || (state.api.otherSector == "40"))) {
    //    state.api.workState = true;
    //    state.api.workStateChange = true;
    //}
    //else if ((state.api.sector == "30") && ((state.api.otherSector == "10") || (state.api.otherSector == "20") || (state.api.otherSector == "22") || (state.api.otherSector == "40"))) {
    //    state.api.workState = true;
    //    state.api.workStateChange = true;
    //}
    //else if ((state.api.sector == "40") && ((state.api.otherSector == "10") || (state.api.otherSector == "20") || (state.api.otherSector == "32") || (state.api.otherSector == "40")) && (state.api.vehicleID == "Idefix")) {
    //    state.api.workState = true;
    //    state.api.workStateChange = true;
    //}
    //Jobstrategy
    if ((state.api.container == "yellow") && (state.api.unloadPort == "1") && (state.api.task != "RED_TO_PORT1") && (state.api.task != "RED_TO_PORT2") && (state.api.task != "YELLOW_TO_PORT2") && (state.api.task != "YELLOW_TO_PORT1") && (state.api.otherTask != "YELLOW_TO_PORT1")) {
        state.api.task = "YELLOW_TO_PORT1";
        state.api.state = "Starting_Task";
        state.api.workState = true;
        state.api.workStateChange = true;
        state.vision.target = 2;
    }
    else if ((state.api.container == "red") && (state.api.unloadPort == "1") && (state.api.task != "RED_TO_PORT1") && (state.api.task != "RED_TO_PORT2") && (state.api.task != "YELLOW_TO_PORT2") && (state.api.task != "YELLOW_TO_PORT1") && (state.api.otherTask != "RED_TO_PORT1")) {
        state.api.task = "RED_TO_PORT1";
        state.api.state = "Starting_Task";
        state.api.workState = true;
        state.api.workStateChange = true;
        state.vision.target = 1;
    }
    else if ((state.api.container == "yellow") && (state.api.unloadPort == "2") && (state.api.task != "RED_TO_PORT1") && (state.api.task != "RED_TO_PORT2") && (state.api.task != "YELLOW_TO_PORT2") && (state.api.task != "YELLOW_TO_PORT1") && (state.api.otherTask != "YELLOW_TO_PORT2")) {
        state.api.task = "YELLOW_TO_PORT2";
        state.api.state = "Starting_Task";
        state.api.workState = true;
        state.api.workStateChange = true;
        state.vision.target = 2;
    }
    else if ((state.api.container == "red") && (state.api.unloadPort == "2") && (state.api.task != "RED_TO_PORT1") && (state.api.task != "RED_TO_PORT2") && (state.api.task != "YELLOW_TO_PORT2") && (state.api.task != "YELLOW_TO_PORT1") && (state.api.otherTask != "RED_TO_PORT2")) {
        state.api.task = "RED_TO_PORT2";
        state.api.state = "Starting_Task";
        state.api.workState = true;
        state.api.workStateChange = true;
        state.vision.target = 1;
    }
    else {
        if (DEBUGGER == true) Serial.println("Container change in else");
        state.api.container = "";
        state.api.containerChange =  true;
    }

   //Drive strategy without an active workstate, there is no sectorhandling
    if (state.api.workState == true) {
       
        //Navigation strategy
        if (DEBUGGER == true) Serial.print("IR-status: "); if (DEBUGGER == true) Serial.println(state.chassis.IR);
        if (DEBUGGER == true) Serial.print("curveFactor before pid: ");
        if (DEBUGGER == true) Serial.println(state.curveFactor);
        state.curveFactor = state.chassis.PidValue;
        
        //Sector 10 movement
        if ((state.api.sector == "10") && (state.api.task != "nothing") && (state.api.task != "finished")) {
            if (DEBUGGER == true) Serial.println("Sector 10");
            if (state.chassis.direction != "right") {
                delay(1000);
            }
            state.chassis.direction = "right";
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (state.chassis.actionDone == true) {
                if (DEBUGGER == true) Serial.print("Sector 10 vehicleID"); if (DEBUGGER == true) Serial.println(state.api.vehicleID);
                if (state.api.vehicleID == "Obelix") {
                    state.api.sector = "11";
                    if (DEBUGGER == true) Serial.println("Sector 10 order to Obelix");
                    state.chassis.direction = "straight";
                }
                else if (state.api.vehicleID == "Idefix") {
                    state.api.sector = "12";
                    if (DEBUGGER == true) Serial.println("Sector 10 order to Idefix");
                    state.chassis.direction = "stop";
                }
                if (DEBUGGER == true) Serial.println("end of scector10"); 
                state.chassis.actionDone = false;
                if (DEBUGGER == true) Serial.print("Sector 10 next sector: "); if (DEBUGGER == true) Serial.println(state.api.sector);
            }
        }
        if (DEBUGGER == true) Serial.print("Between 10 and 11 sector: "); if (DEBUGGER == true) Serial.println(state.api.sector);
        //Sector 11 movement
        if (state.api.sector == "11") {
            if (DEBUGGER == true) Serial.println("Sector 11");
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (state.chassis.actionDone == true) {
               state.api.sector = "12";
                state.chassis.direction = "stop";
                if (DEBUGGER == true) Serial.println("end of sector11"); 
                state.chassis.actionDone = false;
            }
        }

        //Sector 12 movement
        if (state.api.sector == "12") {
            if (DEBUGGER == true) Serial.println("Sector 12");
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (state.chassis.actionDone == true) {
                if (DEBUGGER == true) Serial.println("End of 12");
                state.api.sector = "20";
                state.chassis.actionDone = false;
                vehicleChassis->stop();
                state.chassis.direction = "straight";
                state.api.state = "Searching_box";
                state.visual = true;
                state.turnSonar = false;
                state.sonar.detachServo = true;
            }
        }
        
        //Sector 20 movement
        if (state.api.sector == "20") {
            if (DEBUGGER == true) Serial.println("start of sector20");

            if (((state.chassis.sensor[0] + state.chassis.sensor[1] + state.chassis.sensor[2] + state.chassis.sensor[3] + state.chassis.sensor[4]) == 0) || ((state.chassis.sensor[0] + state.chassis.sensor[1] + state.chassis.sensor[2] + state.chassis.sensor[3] + state.chassis.sensor[4]) == 5)) {
                if ((state.waitForTurn == false) && (state.waitCount == 0)) {
                    if (DEBUGGER == true) Serial.println("changing wait for turn");
                    state.waitForTurn = true;
                }
            }
            if (DEBUGGER == true) Serial.print("Targetdetected: "); if (DEBUGGER == true) Serial.println(state.vision.targetDetected);
            if ((state.vision.targetDetected == true) && (state.turnCount < 4)) {
                //First move dependend on which side the box is on
                if (DEBUGGER == true) Serial.println("before first turn");

                //Is the Box on the right side of the vehicle turn right
                if (state.vision.servoAngle < 90) {
                    if (DEBUGGER == true) Serial.print("waitForturn: "); if (DEBUGGER == true) Serial.println(state.waitForTurn);
                    if (DEBUGGER == true) Serial.print("TurnCount: "); if (DEBUGGER == true) Serial.println(state.turnCount);
                    if ((state.waitForTurn == true) && (state.turnCount ==1)) {

                        if (state.chassis.actionDone == true) {
                            state.turnCount++;
                            if (DEBUGGER == true) Serial.println("right turn accomplished");
                            state.chassis.actionDone = false;
                        }   

                        if (state.waitCount == 0) {
                            if (DEBUGGER == true) Serial.println("starting timer");
                            state.timeStart = millis();
                            state.waitCount++;
                        }

                        if (state.turnCount == 1) {
                        vehicleChassis->turnRightLimited(2800, state.timeStart);
                        state.api.state = "Picking_up_right_box";
                        state.side = "right";
                        if (DEBUGGER == true) Serial.println("turnRightLimited");
                        state.visual = false;
                        state.turnSonar = true;
                        //vehicleVision->turnVision(90);
                        //if (DEBUGGER == true) Serial.println("after turnVision");
                        }                        
                    }

                    //Second move is per default a drive straight for adjustment
                    if (DEBUGGER == true) Serial.println("before second turn");
                    if (state.turnCount == 2) {
                        if (state.chassis.actionDone == true) {
                            state.turnCount++;
                            if (DEBUGGER == true) Serial.println("drive straight accomplished");
                            state.chassis.actionDone = false;
                        }

                        if (DEBUGGER == true) Serial.println("turncount2");
                        if (state.waitCount == 1) {
                            state.timeStart = millis();
                            if (DEBUGGER == true) Serial.println("starting second timer");
                            state.waitCount++;
                        }

                        if (state.turnCount == 2) {
                        vehicleChassis->driveStraightLimited(2800, state.timeStart);
                        if (DEBUGGER == true) Serial.println("drivestraight");
                        }
                    }
                    //Third move is a counterturn dependend on the first move
                    if (DEBUGGER == true) Serial.println("before third turn");
                    if (state.turnCount == 3) { 
                        if (DEBUGGER == true) Serial.println("turncount3");
                        if (state.chassis.actionDone == true) {
                            state.turnCount++;
                            if (DEBUGGER == true) Serial.println("right turn accomplished");
                            state.chassis.actionDone = false;
                            state.visual = true;
                            state.turnSonar = false;
                            state.sonar.detachServo = true;
                        }

                        if (state.waitCount == 2) {
                            state.timeStart = millis();
                            if (DEBUGGER == true) Serial.println("starting third timer");
                            state.waitCount++;
                        }

                        if (state.turnCount == 3) {
                        vehicleChassis->turnLeftLimited(2600, state.timeStart);
                        if (DEBUGGER == true) Serial.println("turnLeftLimited");
                        state.waitForTurn = false;
                        }
                    }
                }
                //Is the Box on the left side of the vehicle turn left
                if (state.vision.servoAngle > 90) {
                    if (DEBUGGER == true) Serial.print("waitForturn: "); if (DEBUGGER == true) Serial.println(state.waitForTurn);
                    if (DEBUGGER == true) Serial.print("TurnCount: "); if (DEBUGGER == true) Serial.println(state.turnCount);
                    if ((state.waitForTurn == true) && (state.turnCount == 1)) {

                        if (state.chassis.actionDone == true) {
                            state.turnCount++;
                            if (DEBUGGER == true) Serial.println("left turn accomplished");
                            state.chassis.actionDone = false;
                        }   

                        if (state.waitCount == 0) {
                            if (DEBUGGER == true) Serial.println("starting timer");
                            state.timeStart = millis();
                            state.waitCount++;
                        }

                        if (state.turnCount == 1) {
                        vehicleChassis->turnLeftLimited(2600, state.timeStart);
                        state.api.state = "Picking_up_left_box";
                        state.side = "left";
                        if (DEBUGGER == true) Serial.println("turnleftlimited");
                        state.visual = false;
                        state.turnSonar = true;
                        if (DEBUGGER == true) Serial.println("after turnVision");
                        }                        
                    }

                    //Second move is per default a drive straight for adjustment
                    if (DEBUGGER == true) Serial.println("before second turn");
                    if (state.turnCount == 2) {
                        if (state.chassis.actionDone == true) {
                            state.turnCount++;
                            if (DEBUGGER == true) Serial.println("drive straight accomplished");
                            state.chassis.actionDone = false;
                        }

                        if (DEBUGGER == true) Serial.println("turncount2");
                        if (state.waitCount == 1) {
                            state.timeStart = millis();
                            if (DEBUGGER == true) Serial.println("starting second timer");
                            state.waitCount++;
                        }

                        if (state.turnCount == 2) {
                                vehicleChassis->driveStraightLimited(2400, state.timeStart);
                        if (DEBUGGER == true) Serial.println("drivestraight");
                        }
                    }
                    //Third move is a counterturn dependend on the first move
                    if (DEBUGGER == true) Serial.println("before third turn");
                    if (state.turnCount == 3) {
                        if (DEBUGGER == true) Serial.println("turncount3");
                        if (state.chassis.actionDone == true) {
                            state.turnCount++;
                            if (DEBUGGER == true) Serial.println("right turn accomplished");
                            state.chassis.actionDone = false;
                            state.visual = true;
                            state.turnSonar = false;
                            state.sonar.detachServo = true;
                            state.vision.targetDetected = false;
                        }

                        if (state.waitCount == 2) {
                            state.timeStart = millis();
                            if (DEBUGGER == true) Serial.println("starting third timer");
                            state.waitCount++;
                        }

                        if (state.turnCount == 3) {
                        vehicleChassis->turnRightLimited(3200, state.timeStart);
                        if (DEBUGGER == true) Serial.println("turnright");
                        state.waitForTurn = false;
                        }
                    }
                }
            }
            //If the target is lost or never detected, the system has to stop and search for a box again
            else if ((state.vision.targetDetected == false) && (state.turnCount < 4)) {
                state.visual = true;
                state.turnSonar = false;
                state.sonar.detachServo = true;
                vehicleChassis->stop();
                if (DEBUGGER == true) Serial.println("Stopping when No target");
            }

            //Before the box the vehicle first has to refind the line
            if (DEBUGGER == true) Serial.println("before cameranavigation");
            if (state.turnCount == 4) {
                if (DEBUGGER == true) Serial.print("Turn4 with servoAngle: ");
                if (DEBUGGER == true) Serial.println(state.vision.servoAngle);
                if (DEBUGGER == true) Serial.print("Visual: "); if (DEBUGGER == true) Serial.println(state.visual);
                if (((state.vision.targetDetected == true) || (state.visual == false)) && (state.hoist.loaded == false)) {
                    if ((state.vision.servoAngle > 92) && (state.visual == true)) {
                        vehicleChassis->turnLeft();
                        if (DEBUGGER == true) Serial.println("after turnleft");
                    }
                    if ((state.vision.servoAngle < 88) && (state.visual == true)) {
                        vehicleChassis->turnRight();
                        if (DEBUGGER == true) Serial.println("after turnright");
                    }
                    if (((state.vision.servoAngle <= 92) && (state.vision.servoAngle >= 88)) && (state.visual == true)) {
                        vehicleChassis->driveStraight();
                        if (DEBUGGER == true) Serial.println("after drivestraight");
                    }
                    if (DEBUGGER == true) Serial.print(state.chassis.sensor[0]); if (DEBUGGER == true) Serial.print(state.chassis.sensor[1]); if (DEBUGGER == true) Serial.print(state.chassis.sensor[2]); if (DEBUGGER == true) Serial.print(state.chassis.sensor[3]); if (DEBUGGER == true) Serial.print(state.chassis.sensor[4]);
                    if (((state.chassis.sensor[0] + state.chassis.sensor[1] + state.chassis.sensor[2] + state.chassis.sensor[3] + state.chassis.sensor[4]) != 0) && (state.visual == true)) {
                        if (DEBUGGER == true) Serial.println("detected line");
                        //state.chassis.IR = true;
                        state.visual = false;
                        state.turnSonar = true;
                        state.api.sector = "21";
                    }
                }
                else if (state.visual == true) {
                    vehicleChassis->stop();
                    if (DEBUGGER == true) Serial.println("stopping when no target available");
                }
            }            

            
        }

        //Sector 21 movement
        if (state.api.sector == "21") {
                if (DEBUGGER == true) Serial.println("following line again");
                vehicleChassis->motorControl(state.sonar.sonarFactor);
                if (state.chassis.actionDone == true) {
                state.api.sector = "22";
                state.chassis.direction = "stop";
                if (DEBUGGER == true) Serial.println("end of sector21"); 
                state.chassis.actionDone = false;
                }
        }
    
        //Sector 22 movement
        if (state.api.sector == "22") {
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (state.chassis.actionDone == true) {
            state.api.sector = "23";
            if (DEBUGGER == true) Serial.println("end of sector22"); 
            state.chassis.actionDone = false;
            }
        }

        //Sector 23 movement
        if (state.api.sector == "23") {
            state.vision.targetDetected = false;
            if (DEBUGGER == true) Serial.println("Arrived at loadingpoint");
            if (DEBUGGER == true) Serial.print("ActionDone 23: "); if (DEBUGGER == true) Serial.println(state.chassis.actionDone);
            if (state.hoist.loaded == false) {
                vehicleHoist->load();
                vehicleChassis->stop();
                state.chassis.direction = "straight";
                state.api.state = "Loading_box";
                state.chassis.IR = false;
                if (DEBUGGER == true) Serial.print("ActionDone 23 loading: "); if (DEBUGGER == true) Serial.println(state.chassis.actionDone);
            }
            //After loading process the system has to turn around
            if (state.hoist.loaded == true) {
                state.hoist.detachServo = true;
                state.api.state = "Bringing_box_to_port";
                if (DEBUGGER == true) Serial.println("Loading complete");
                    if (state.chassis.actionDone == false) {
                        vehicleChassis->turnAround(2);
                    }
                    if (DEBUGGER == true) Serial.print("ActionDone: "); if (DEBUGGER == true) Serial.println(state.chassis.actionDone);
                if (state.chassis.actionDone == true) {
                    state.api.sector = "24";
                    state.chassis.direction = "right";
                    state.chassis.actionDone = false;
                    if (DEBUGGER == true) Serial.println("sector23 finished");
                }
            } 
        }

        //Start of sector 24 where the vehicle drives the container to the targeted port
        if (state.api.sector == "24") {
            if (DEBUGGER == true) Serial.println("Sector24");
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (state.chassis.actionDone == true) {
                state.api.sector = "30";
                if (state.side == "right") {
                    state.chassis.direction = "left";
                    if (DEBUGGER == true) Serial.println("direction from box at beginning was right");
                }
                if (state.side == "left") {
                    state.chassis.direction = "straight";
                    if (DEBUGGER == true) Serial.println("direction from box at beginning was left");
                }
                if (DEBUGGER == true) Serial.println("Sector24 actiondone");
                state.chassis.actionDone = false;                
            }  
        }

        //Start of sector 30 where the vehicle still drives the container to the targeted port
        if (state.api.sector == "30") {
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (DEBUGGER == true) Serial.println("inside sector 30");
            if (state.chassis.actionDone == true) {
                    
                state.api.sector = "31";
                if (state.api.unloadPort == "1") {
                state.chassis.direction = "straight";
                }
                else if (state.api.unloadPort == "2") {
                    state.chassis.direction = "left";
                }
                state.chassis.actionDone = false;
            }
        }
        
        //Sector 31 movement
        if (state.api.sector == "31") {
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (DEBUGGER == true) Serial.println("Inside sector 31");
            if (state.chassis.actionDone == true) {
                
                if (state.api.unloadPort == "1") {
                  state.chassis.direction = "stop";
                  state.api.sector = "33";  
                }
                if (state.api.unloadPort == "2") {
                    state.chassis.direction = "right";
                    state.api.sector = "32";
                }
                state.chassis.actionDone = false;
            }
        }

        //Sector 32 movement
        if (state.api.sector == "32") {
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (DEBUGGER == true) Serial.println("inside sector 32");
            if (state.chassis.actionDone == true) {
                state.api.sector = "33";
                if (DEBUGGER == true) Serial.println("Finnishing sector 32");
                state.chassis.direction = "stop";
                state.chassis.actionDone = false;
            }
        }

        //Sector 33 movement
        if (state.api.sector == "33") {
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (state.chassis.actionDone == true) {
                state.api.sector = "34";
                if (DEBUGGER == true) Serial.println("Finnishing sector 33");
                state.chassis.direction = "straight";
                state.chassis.actionDone = false;
            }
        }
        
        //Sector 34 movement
        if (state.api.sector == "34") {
            vehicleChassis->stop();
            vehicleHoist->unload();
            state.chassis.IR = false;
            if (DEBUGGER == true) Serial.println("unload");
            if (state.hoist.loaded == false) {
                state.hoist.detachServo = true;
                state.api.sector = "35";
                state.api.state = "Box_delivered";
                state.timeStart = millis();
                
                if (DEBUGGER == true) Serial.println("starting fourth timer");
            }
        }

        //Sector 35 movement
        if (state.api.sector == "35") {
            if (state.chassis.actionDone == false) {
            vehicleChassis->driveBackLimited(2000, state.timeStart);
            state.chassis.IR = false;
            }
            if (state.chassis.actionDone == true) {
                state.api.sector = "36";
                if (DEBUGGER == true) Serial.println("Driveback home");
                state.api.state = "driving_back_home";
                state.chassis.actionDone = false;
            }
            
        }
        
        //Sector 36 movement
        if (state.api.sector == "36") { 
            if (DEBUGGER == true) Serial.print("Sector 36 Actiondone: "); if (DEBUGGER == true) Serial.println(state.chassis.actionDone);
            if (state.chassis.actionDone == false) {
            vehicleChassis->turnAround(2);
            if (DEBUGGER == true) Serial.println("Sector 36 after turnaround");
            }
            if (state.chassis.actionDone == true) {
                state.api.sector = "37";
                state.chassis.direction = "right";
                state.chassis.actionDone = false;
            }
        }

        //Sector 37 movement
        if (state.api.sector == "37") {
            if (DEBUGGER == true) Serial.println("Sector 37");
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (state.chassis.actionDone == true) {
                if (state.api.unloadPort == "1") {
                    state.api.sector = "38";
                    state.chassis.direction = "straight";
                }
                else if (state.api.unloadPort == "2") {
                    state.api.sector = "40";
                    if (state.api.vehicleID == "Idefix") {
                    state.chassis.direction = "left";
                    }
                    else if (state.api.vehicleID == "Obelix") {
                        state.chassis.direction = "straight";
                    }
                    state.chassis.actionDone = false;
                }
            state.chassis.actionDone = false;
            }
        }

        //Sector 38 movement
        if (state.api.sector == "38") {
            if (DEBUGGER == true) Serial.println("Sector 38");
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (state.chassis.actionDone == true) {
                state.api.sector = "40";
                if (state.api.vehicleID == "Idefix") {
                    state.chassis.direction = "left";
                }
                else if (state.api.vehicleID == "Obelix") {
                    state.chassis.direction = "straight";
                }
                state.chassis.actionDone = false;
            }
        }

        //Sector 40 movement
        if (state.api.sector == "40") {
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (DEBUGGER == true) Serial.println("Sector 40");
            if (state.chassis.actionDone == true) {
                if (state.api.vehicleID == "Obelix") {
                    if (DEBUGGER == true) Serial.println("Sector 40 Obelix finished");
                    state.chassis.direction = "stop";
                    state.api.sector = "42";
                }
                else if (state.api.vehicleID == "Idefix") {
                    if (DEBUGGER == true) Serial.println("Sector 40 Idefix finished");
                    state.chassis.direction = "right";
                    state.api.sector = "41";
                }
                state.chassis.actionDone = false;
            }
        }

        //Sector 41 movement
        if (state.api.sector == "41") {
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (DEBUGGER == true) Serial.print("Sector 41 Actiondone: "); if (DEBUGGER == true) Serial.println(state.chassis.actionDone);
            if (state.chassis.actionDone == true) {
                state.chassis.direction = "stop";
                if (DEBUGGER == true) Serial.println("Sector 41 finished");
                state.api.sector = "42";
                state.chassis.actionDone = false;
            }
        }

        //Sector 42 movement
        if (state.api.sector == "42") {
            vehicleChassis->motorControl(state.sonar.sonarFactor);
            if (DEBUGGER == true) Serial.print("Sector 42 Actiondone: "); if (DEBUGGER == true) Serial.println(state.chassis.actionDone);
            if (state.chassis.actionDone == true) {
                state.chassis.direction = "stop";
                if (DEBUGGER == true) Serial.println("Sector 42 finished");
                vehicleChassis->stop();
                state.api.sector = "43";
                state.chassis.actionDone = false;
            }
        }  

        //Sector 43 movement reset the variables once the task is finished
        if (state.api.sector == "43") {
            if (DEBUGGER == true) Serial.print("Sector 43 Actiondone: "); if (DEBUGGER == true) Serial.println(state.chassis.actionDone);
            if (state.chassis.actionDone == false) {
            vehicleChassis->turnAround(2);
            if (DEBUGGER == true) Serial.println("Sector 43 after turnaround");
            }
            if (state.chassis.actionDone == true) {
                if (DEBUGGER == true) Serial.println("Sector 43 finished");
                state.api.sector = "10";
                state.api.state = "resting";
                state.api.task = "finished";
                state.api.workState = false;
                state.api.workStateChange = true;
                state.chassis.actionDone = false;
                state.waitCount = 0;
                state.turnCount = 1;
                state.waitForTurn = false;
                state.vision.targetDetected = false;
                state.api.jobCount++;
                state.vision.reset = true;
                state.api.container = "";
                state.api.unloadPort= "";
                state.api.containerChange = true;
                state.api.unloadPortChange = true;
            }    
             
        }
    }
    else {
        vehicleChassis->stop();
    }
}


void setup() {
    //Initialize serial and wait for port to open:
    if (DEBUGGER == true) Serial.begin(9600);
    //while (!if (DEBUGGER == true) Serial) {
    //; // wait foserial port to connect. Needed for native USB port only
    //}
    if (DEBUGGER == true) Serial.println("Vehicle: booting...");
    vehicleChassis = new Chassis(SPEED, K_P, K_I, K_D, RIGHT_MOTOR, LEFT_MOTOR, PIN_SENSOR_0, PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4);
    state.chassis.speed = SPEED;
    delay(1000);
    vehicleSonar = new Sonar(SONAR_SERVO_PIN, SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE, MIN_ERROR, MAX_ERROR, MIN_TURN_ANGLE, MAX_TURN_ANGLE);
    delay(1000);
    vehicleVision = new Vision(VISION_START_ANGLE, VISION_SERVO_PIN, VISION_DELAY_FACTOR, VISION_TOLERANCE_LEFT, VISION_TOLERANCE_RIGHT);
    delay(1000);
    vehicleHoist = new Hoist(HOIST_SERVO_PIN, HOIST_SERVO_DELAY, HOIST_POISITION_MAX, HOIST_POSITION_MIN);
    delay(1000);
    vehicleAPI = new VehicleWebAPI(&state.api);
    delay(1000);
    if (DEBUGGER == true) Serial.println("Booting complete!");
}




void loop() { 
    vehicleAPI->loop(&state.api);
    if (state.api.workState == true) {
        state.sonarCount++;
        if ((state.api.sector == "20") && (state.visual == true)) {
            vehicleVision->loop(&state.vision);
        }
        vehicleChassis->loop(&state.chassis);
        if (state.sonarCount > 30) {
            state.sonarCount = 0;
            vehicleSonar->loop(&state.sonar, state.chassis.directionError, state.turnSonar);
        }
        vehicleHoist->loop(&state.hoist);   
    }
    strategy();
    if (DEBUGGER == true) Serial.println("-----------------------------------------");
}

*/