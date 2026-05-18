#include "FsmManager.h"
#include "BleManager.h"
#include "SensorManager.h"
#include "NavigationManager.h"

FsmManager::FsmManager(BleManager* ble, SensorManager* sensor, NavigationManager* nav)
    : bleManager(ble), sensorManager(sensor), navManager(nav), 
      currentState(STATE_IDLE), previousState(STATE_IDLE), 
      statusMessage("Ready"), numRoomsInQueue(0), currentQueueIndex(0), currentRoom(0), stateStartTime(0) {}

void FsmManager::init() {
    changeState(STATE_IDLE, "System Initialized");
}

void FsmManager::changeState(RobotState newState, String msg) {
    previousState = currentState;
    currentState = newState;
    stateStartTime = millis();
    if (msg.length() > 0) {
        statusMessage = msg;
    }
    Serial.print("State Changed to: ");
    Serial.println(getCurrentStateString());
}

unsigned long FsmManager::timeInState() const {
    return millis() - stateStartTime;
}

void FsmManager::update() {
    checkPhysicalButton();

    switch (currentState) {
        case STATE_IDLE: handleIdle(); break;
        case STATE_PLAN_ROUTE: handlePlanRoute(); break;
        case STATE_MOVE_FORWARD: handleMoveForward(); break;
        case STATE_AVOID_OBSTACLE: handleAvoidObstacle(); break;
        case STATE_SEARCH_ROOM: handleSearchRoom(); break;
        case STATE_ENTER_ROOM: handleEnterRoom(); break;
        case STATE_DELIVER_MEDICINE: handleDeliverMedicine(); break;
        case STATE_EXIT_ROOM: handleExitRoom(); break;
        case STATE_WAITING_FOR_ACK: handleWaitingForAck(); break;
        case STATE_NEXT_ROOM: handleNextRoom(); break;
        case STATE_RETURN_HOME: handleReturnHome(); break;
        case STATE_STOP: handleStop(); break;
        case STATE_MANUAL_DRIVE: handleManualDrive(); break;
        case STATE_ERROR: handleError(); break;
        default: break;
    }
}

// Commands from App
void FsmManager::startDelivery(DeliveryMode mode, int priorityRoom, int rooms[], int compartments[], int numRooms) {
    if (currentState != STATE_IDLE) return;
    
    // Copy queue
    currentMode = mode;
    priorityRoom = priorityRoom;
    numRoomsInQueue = (numRooms > MAX_QUEUE_SIZE) ? MAX_QUEUE_SIZE : numRooms;
    
    for (int i = 0; i < numRoomsInQueue; i++) {
        deliveryQueue[i] = rooms[i];
        deliveryCompartments[i] = compartments[i];
    }
    
    currentQueueIndex = 0;
    currentRoom = 0;
    currentCompartment = 0;
    if (mode == MODE_PRIORITY) {
        // Move priority room to front
        for(int i=0; i<numRooms; i++) {
            if (deliveryQueue[i] == priorityRoom) {
                int temp = deliveryQueue[0];
                deliveryQueue[0] = deliveryQueue[i];
                deliveryQueue[i] = temp;
                break;
            }
        }
    } else if (mode == MODE_RANDOM) {
        // Simple swap shuffle
        for (int i=0; i<numRooms; i++) {
            int r = random(i, numRooms);
            int temp = deliveryQueue[i];
            deliveryQueue[i] = deliveryQueue[r];
            deliveryQueue[r] = temp;
        }
    }
    // Strict mode requires no changes
    
    changeState(STATE_PLAN_ROUTE, "Planning Route");
}

void FsmManager::triggerEmergencyStop() {
    changeState(STATE_STOP, "EMERGENCY STOP");
}

void FsmManager::triggerReturnHome() {
    changeState(STATE_RETURN_HOME, "Returning Home");
}

void FsmManager::triggerContinueDelivery() {
    if (currentState == STATE_WAITING_FOR_ACK) {
        changeState(STATE_EXIT_ROOM, "Continuing to next");
    }
}

void FsmManager::handleManualMove(String direction) {
    if (direction == "STOP") {
        navManager->stop();
        if (currentState == STATE_MANUAL_DRIVE) {
            changeState(STATE_IDLE, "Manual Stopped");
        }
    } else {
        if (currentState != STATE_MANUAL_DRIVE) {
            changeState(STATE_MANUAL_DRIVE, "Manual Drive");
        }
        
        if (direction == "FORWARD") navManager->moveForward(255);
        else if (direction == "BACKWARD") navManager->moveBackward(255);
        else if (direction == "LEFT") navManager->turnLeft(255);
        else if (direction == "RIGHT") navManager->turnRight(255);
    }
}

void FsmManager::checkPhysicalButton() {
    if (digitalRead(PIN_BUTTON) == LOW) {
        if (millis() - lastButtonPressTime > 500) {
            lastButtonPressTime = millis();
            triggerContinueDelivery();
        }
    }
}

// State Handlers Implementation (Mocked logic for brevity)
void FsmManager::handleIdle() {
    navManager->stop();
}

void FsmManager::handlePlanRoute() {
    navManager->stop();
    if (currentQueueIndex < numRoomsInQueue) {
        currentRoom = deliveryQueue[currentQueueIndex];
        currentCompartment = deliveryCompartments[currentQueueIndex];
        changeState(STATE_MOVE_FORWARD, "Moving to Room " + String(currentRoom));
    } else {
        changeState(STATE_RETURN_HOME, "Queue Empty");
    }
}

void FsmManager::handleMoveForward() {
    if (sensorManager->isObstacleAhead()) {
        changeState(STATE_AVOID_OBSTACLE, "Obstacle Detected!");
        return;
    }
    
    // Wall follow logic
    navManager->followWall(sensorManager->getLeftDistance(), sensorManager->getRightDistance(), 150);
    
    // Mock room detection (e.g. after moving 5 seconds)
    if (timeInState() > 5000) {
        changeState(STATE_SEARCH_ROOM, "Searching Doorway");
    }
}

void FsmManager::handleAvoidObstacle() {
    navManager->stop();
    if (!sensorManager->isObstacleAhead()) {
        changeState(STATE_MOVE_FORWARD, "Path Clear");
    }
}

void FsmManager::handleSearchRoom() {
    navManager->moveForward(100); // Move slower
    if (timeInState() > 2000) {
        changeState(STATE_ENTER_ROOM, "Entering Room");
    }
}

void FsmManager::handleEnterRoom() {
    navManager->turnRight(150); // Hardcoded turn for demo
    if (timeInState() > 1000) {
        navManager->stop();
        changeState(STATE_DELIVER_MEDICINE, "Arrived at destination");
    }
}

void FsmManager::handleDeliverMedicine() {
    navManager->stop();
    // Reached target room. Wait briefly then go to waiting for ACK.
    if (timeInState() > 3000) {
        // Beep buzzer to notify arrival
        digitalWrite(PIN_BUZZER, HIGH);
        delay(500);
        digitalWrite(PIN_BUZZER, LOW);
        changeState(STATE_WAITING_FOR_ACK, "Waiting for Continue");
    }
}

void FsmManager::handleWaitingForAck() {
    navManager->stop();
    // Stays in this state until triggerContinueDelivery() is called by BLE or physical button
}

void FsmManager::handleExitRoom() {
    navManager->moveBackward(150);
    if (timeInState() > 1000) {
        changeState(STATE_NEXT_ROOM, "Exited Room");
    }
}

void FsmManager::handleNextRoom() {
    currentQueueIndex++;
    changeState(STATE_PLAN_ROUTE, "Fetching Next Destination");
}

void FsmManager::handleReturnHome() {
    navManager->moveBackward(150); // simplistic return home mechanism
    if (timeInState() > 5000) {
        changeState(STATE_IDLE, "Arrived Home");
    }
}

void FsmManager::handleStop() {
    navManager->stop();
    // Requires command to recover
}

void FsmManager::handleError() {
    navManager->stop();
}

void FsmManager::handleManualDrive() {
    // Movement handled dynamically by BLE commands
    
    // TEMPORARILY DISABLED: Ultrasonic sensors often glitch when motors draw heavy current,
    // causing false "Obstacle" detections which instantly stop the robot.
    /*
    if (sensorManager->isObstacleAhead()) {
        navManager->stop();
        changeState(STATE_IDLE, "Obstacle in Manual!");
    }
    */
}

// Getters
RobotState FsmManager::getCurrentState() const { return currentState; }
int FsmManager::getCurrentRoom() const { return currentRoom; }
int FsmManager::getCurrentCompartment() const { return currentCompartment; }

String FsmManager::getCurrentStateString() const {
    switch(currentState) {
        case STATE_IDLE: return "IDLE";
        case STATE_PLAN_ROUTE: return "PLAN_ROUTE";
        case STATE_MOVE_FORWARD: return "MOVE_FORWARD";
        case STATE_AVOID_OBSTACLE: return "AVOID_OBSTACLE";
        case STATE_SEARCH_ROOM: return "SEARCH_ROOM";
        case STATE_ENTER_ROOM: return "ENTER_ROOM";
        case STATE_DELIVER_MEDICINE: return "DELIVER_MEDICINE";
        case STATE_EXIT_ROOM: return "EXIT_ROOM";
        case STATE_WAITING_FOR_ACK: return "WAITING_FOR_ACK";
        case STATE_NEXT_ROOM: return "NEXT_ROOM";
        case STATE_RETURN_HOME: return "RETURN_HOME";
        case STATE_STOP: return "STOP";
        case STATE_MANUAL_DRIVE: return "MANUAL_DRIVE";
        case STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

String FsmManager::getStatusMessage() const {
    return statusMessage;
}
