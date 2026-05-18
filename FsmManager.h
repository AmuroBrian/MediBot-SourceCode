#ifndef FSM_MANAGER_H
#define FSM_MANAGER_H

#include <Arduino.h>
#include "Config.h"

// Forward declarations
class BleManager;
class SensorManager;
class NavigationManager;

enum NavCommandType {
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_STOP,
    CMD_DELIVER,
    CMD_HOME
};

struct NavCommand {
    NavCommandType type;
    unsigned long duration;
};

#define MAX_NAV_COMMANDS 50

class FsmManager {
public:
    FsmManager(BleManager* ble, SensorManager* sensor, NavigationManager* nav);
    void init();
    void update();
    
    // Command interface
    void startDelivery(DeliveryMode mode, int priorityRoom, int rooms[], int compartments[], int numRooms);
    void triggerEmergencyStop();
    void triggerReturnHome();
    void triggerContinueDelivery();
    void handleManualMove(String direction);
    void checkPhysicalButton();
    
    // State getters
    RobotState getCurrentState() const;
    String getCurrentStateString() const;
    int getCurrentRoom() const;
    int getCurrentCompartment() const;
    String getStatusMessage() const;

private:
    BleManager* bleManager;
    SensorManager* sensorManager;
    NavigationManager* navManager;
    
    RobotState currentState;
    RobotState previousState;
    String statusMessage;
    
    // Delivery Data
    DeliveryMode currentMode;
    int deliveryQueue[MAX_QUEUE_SIZE];
    int deliveryCompartments[MAX_QUEUE_SIZE];
    int numRoomsInQueue;
    int currentQueueIndex;
    int priorityRoom;
    int currentRoom;
    int currentCompartment;
    
    // Timers for states
    unsigned long stateStartTime;
    unsigned long lastButtonPressTime = 0;
    unsigned long timeInState() const;
    
    // Grid Coordinates
    int currentX;
    int currentY;
    int currentHeading; // 0=North, 1=East, 2=South, 3=West

    // Command Queue
    NavCommand commandQueue[MAX_NAV_COMMANDS];
    int cmdQueueSize;
    int currentCmdIndex;
    
    unsigned long currentCmdStartTime;
    unsigned long currentCmdElapsedTime;
    bool isCommandPaused;
    
    void generatePath(int targetRoom);
    void addCommand(NavCommandType type, unsigned long duration);
    void executeNextCommand();

    void changeState(RobotState newState, String msg = "");
    
    // State Handlers
    void handleIdle();
    void handlePlanRoute();
    void handleExecuteCmd();
    void handleAvoidObstacle();
    void handleEnterRoom();
    void handleDeliverMedicine();
    void handleExitRoom();
    void handleWaitingForAck();
    void handleNextRoom();
    void handleReturnHome();
    void handleStop();
    void handleManualDrive();
    void handleError();
};

#endif // FSM_MANAGER_H
