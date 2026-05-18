#include "FsmManager.h"
#include "BleManager.h"
#include "SensorManager.h"
#include "NavigationManager.h"

FsmManager::FsmManager(BleManager* ble, SensorManager* sensor, NavigationManager* nav)
    : bleManager(ble), sensorManager(sensor), navManager(nav), 
      currentState(STATE_IDLE), previousState(STATE_IDLE), 
      statusMessage("Ready"), numRoomsInQueue(0), currentQueueIndex(0), 
      currentRoom(0), currentCompartment(0), stateStartTime(0),
      currentX(2), currentY(0), currentHeading(2), // Home coordinates, facing South
      cmdQueueSize(0), currentCmdIndex(0), isCommandPaused(false) {}

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
        case STATE_EXECUTE_CMD: handleExecuteCmd(); break;
        case STATE_AVOID_OBSTACLE: handleAvoidObstacle(); break;
        case STATE_DELIVER_MEDICINE: handleDeliverMedicine(); break;
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
    
    currentMode = mode;
    this->priorityRoom = priorityRoom;
    numRoomsInQueue = (numRooms > MAX_QUEUE_SIZE) ? MAX_QUEUE_SIZE : numRooms;
    
    for (int i = 0; i < numRoomsInQueue; i++) {
        deliveryQueue[i] = rooms[i];
        deliveryCompartments[i] = compartments[i];
    }
    
    currentQueueIndex = 0;
    currentRoom = 0;
    currentCompartment = 0;

    if (mode == MODE_PRIORITY) {
        for(int i=0; i<numRooms; i++) {
            if (deliveryQueue[i] == priorityRoom) {
                int tempR = deliveryQueue[0];
                int tempC = deliveryCompartments[0];
                deliveryQueue[0] = deliveryQueue[i];
                deliveryCompartments[0] = deliveryCompartments[i];
                deliveryQueue[i] = tempR;
                deliveryCompartments[i] = tempC;
                break;
            }
        }
    } else if (mode == MODE_RANDOM) {
        for (int i=0; i<numRooms; i++) {
            int r = random(i, numRooms);
            int tempR = deliveryQueue[i];
            int tempC = deliveryCompartments[i];
            deliveryQueue[i] = deliveryQueue[r];
            deliveryCompartments[i] = deliveryCompartments[r];
            deliveryQueue[r] = tempR;
            deliveryCompartments[r] = tempC;
        }
    }
    
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
        changeState(STATE_NEXT_ROOM, "Continuing to next");
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

// ---------------------------------------------------------
// PATHFINDING & COMMAND QUEUE
// ---------------------------------------------------------

struct Point { int x; int y; };
Point getRoomLocation(int roomNumber) {
    if (roomNumber == 1) return {0, 1};
    if (roomNumber == 2) return {4, 1};
    if (roomNumber == 3) return {0, 3};
    if (roomNumber == 4) return {4, 3};
    return {2, 0}; // Home
}

void FsmManager::addCommand(NavCommandType type, unsigned long duration) {
    if (cmdQueueSize < MAX_NAV_COMMANDS) {
        commandQueue[cmdQueueSize].type = type;
        commandQueue[cmdQueueSize].duration = duration;
        cmdQueueSize++;
    }
}

void FsmManager::generatePath(int targetRoom) {
    cmdQueueSize = 0;
    currentCmdIndex = 0;
    
    Point target = getRoomLocation(targetRoom);
    int dx = target.x - currentX;
    int dy = target.y - currentY;
    
    unsigned long TURN_DURATION = 500; // ms to turn 90 degrees
    unsigned long TILE_DURATION = 1000; // ms to move 1 unit
    
    // Move along Y axis
    if (dy != 0) {
        if (dy > 0) { // Target is South
            if (currentHeading == 1) { addCommand(CMD_TURN_RIGHT, TURN_DURATION); currentHeading = 2; }
            else if (currentHeading == 3) { addCommand(CMD_TURN_LEFT, TURN_DURATION); currentHeading = 2; }
            
            if (currentHeading == 2) addCommand(CMD_FORWARD, dy * TILE_DURATION);
            else if (currentHeading == 0) addCommand(CMD_BACKWARD, dy * TILE_DURATION);
        } else { // Target is North
            int absDy = -dy;
            if (currentHeading == 1) { addCommand(CMD_TURN_LEFT, TURN_DURATION); currentHeading = 0; }
            else if (currentHeading == 3) { addCommand(CMD_TURN_RIGHT, TURN_DURATION); currentHeading = 0; }
            
            if (currentHeading == 0) addCommand(CMD_FORWARD, absDy * TILE_DURATION);
            else if (currentHeading == 2) addCommand(CMD_BACKWARD, absDy * TILE_DURATION);
        }
    }
    
    // Move along X axis
    if (dx != 0) {
        if (dx > 0) { // Target is East
            if (currentHeading == 0) { addCommand(CMD_TURN_RIGHT, TURN_DURATION); currentHeading = 1; }
            else if (currentHeading == 2) { addCommand(CMD_TURN_LEFT, TURN_DURATION); currentHeading = 1; }
            
            if (currentHeading == 1) addCommand(CMD_FORWARD, dx * TILE_DURATION);
            else if (currentHeading == 3) addCommand(CMD_BACKWARD, dx * TILE_DURATION);
        } else { // Target is West
            int absDx = -dx;
            if (currentHeading == 0) { addCommand(CMD_TURN_LEFT, TURN_DURATION); currentHeading = 3; }
            else if (currentHeading == 2) { addCommand(CMD_TURN_RIGHT, TURN_DURATION); currentHeading = 3; }
            
            if (currentHeading == 3) addCommand(CMD_FORWARD, absDx * TILE_DURATION);
            else if (currentHeading == 1) addCommand(CMD_BACKWARD, absDx * TILE_DURATION);
        }
    }
    
    // Arrived at destination
    if (targetRoom == 0) {
        addCommand(CMD_HOME, 0);
    } else {
        addCommand(CMD_DELIVER, 0);
    }
    
    // Update our internal map position to the new target
    currentX = target.x;
    currentY = target.y;
}

void FsmManager::executeNextCommand() {
    if (currentCmdIndex >= cmdQueueSize) {
        navManager->stop();
        return; // No more commands
    }
    
    NavCommand cmd = commandQueue[currentCmdIndex];
    currentCmdStartTime = millis();
    currentCmdElapsedTime = 0;
    isCommandPaused = false;
    
    switch (cmd.type) {
        case CMD_FORWARD: navManager->moveForward(255); break;
        case CMD_BACKWARD: navManager->moveBackward(255); break;
        case CMD_TURN_LEFT: navManager->turnLeft(255); break;
        case CMD_TURN_RIGHT: navManager->turnRight(255); break;
        case CMD_STOP: navManager->stop(); break;
        case CMD_DELIVER: 
            changeState(STATE_DELIVER_MEDICINE, "Arrived at Room " + String(currentRoom));
            return;
        case CMD_HOME:
            changeState(STATE_IDLE, "Arrived Home");
            return;
    }
    
    changeState(STATE_EXECUTE_CMD, "Executing Route");
}


// ---------------------------------------------------------
// STATE HANDLERS
// ---------------------------------------------------------

void FsmManager::handleIdle() {
    navManager->stop();
}

void FsmManager::handlePlanRoute() {
    navManager->stop();
    if (currentQueueIndex < numRoomsInQueue) {
        currentRoom = deliveryQueue[currentQueueIndex];
        currentCompartment = deliveryCompartments[currentQueueIndex];
        
        generatePath(currentRoom);
        executeNextCommand();
    } else {
        changeState(STATE_RETURN_HOME, "Queue Empty");
    }
}

void FsmManager::handleExecuteCmd() {
    // If we are currently moving forward and an obstacle appears, PAUSE.
    NavCommand cmd = commandQueue[currentCmdIndex];
    
    if (cmd.type == CMD_FORWARD || cmd.type == CMD_BACKWARD) {
        if (sensorManager->isObstacleAhead()) {
            navManager->stop();
            isCommandPaused = true;
            // Record how much time we already spent moving
            currentCmdElapsedTime += (millis() - currentCmdStartTime);
            changeState(STATE_AVOID_OBSTACLE, "Obstacle Detected!");
            return;
        }
    }
    
    unsigned long timeExecuting = currentCmdElapsedTime + (millis() - currentCmdStartTime);
    
    if (timeExecuting >= cmd.duration) {
        // Command Finished
        navManager->stop();
        currentCmdIndex++;
        executeNextCommand();
    }
}

void FsmManager::handleAvoidObstacle() {
    navManager->stop();
    if (!sensorManager->isObstacleAhead()) {
        // Obstacle is gone. Resume command.
        currentCmdStartTime = millis();
        isCommandPaused = false;
        
        NavCommand cmd = commandQueue[currentCmdIndex];
        if (cmd.type == CMD_FORWARD) navManager->moveForward(255);
        else if (cmd.type == CMD_BACKWARD) navManager->moveBackward(255);
        
        changeState(STATE_EXECUTE_CMD, "Path Clear");
    }
}

void FsmManager::handleDeliverMedicine() {
    navManager->stop();
    if (timeInState() > 1000) { // Brief pause
        digitalWrite(PIN_BUZZER, HIGH);
        delay(500);
        digitalWrite(PIN_BUZZER, LOW);
        changeState(STATE_WAITING_FOR_ACK, "Please take medicine");
    }
}

void FsmManager::handleWaitingForAck() {
    navManager->stop();
    // Stays in this state until triggerContinueDelivery() is called by BLE or physical button
}

void FsmManager::handleNextRoom() {
    currentQueueIndex++;
    changeState(STATE_PLAN_ROUTE, "Fetching Next Destination");
}

void FsmManager::handleReturnHome() {
    navManager->stop();
    generatePath(0); // 0 is Home
    executeNextCommand();
}

void FsmManager::handleStop() {
    navManager->stop();
}

void FsmManager::handleError() {
    navManager->stop();
}

void FsmManager::handleManualDrive() {
    // Obstacle detection explicitly disabled for manual drive to prevent glitches
}

// Getters
RobotState FsmManager::getCurrentState() const { return currentState; }
int FsmManager::getCurrentRoom() const { return currentRoom; }
int FsmManager::getCurrentCompartment() const { return currentCompartment; }

String FsmManager::getCurrentStateString() const {
    switch(currentState) {
        case STATE_IDLE: return "IDLE";
        case STATE_PLAN_ROUTE: return "PLAN_ROUTE";
        case STATE_EXECUTE_CMD: return "EXECUTE_CMD";
        case STATE_AVOID_OBSTACLE: return "AVOID_OBSTACLE";
        case STATE_DELIVER_MEDICINE: return "DELIVER_MEDICINE";
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
