#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// UUIDs for the service and characteristics
#define SERVICE_UUID           "12345678-1234-1234-1234-123456789012"
#define STATUS_CHAR_UUID       "87654321-4321-4321-4321-210987654321"
#define COMMAND_CHAR_UUID      "11223344-5566-7788-9900-aabbccddeeff"

// Forward declaration of FsmManager to prevent circular dependency
class FsmManager;

class BleManager : public BLEServerCallbacks, public BLECharacteristicCallbacks {
public:
    BleManager();
    void init(FsmManager* fsm);
    void sendStatusUpdate(String state, int currentRoom, int battery, String statusMsg);
    
    // Callbacks overrides
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
    void onWrite(BLECharacteristic* pCharacteristic) override;

private:
    BLEServer* pServer;
    BLECharacteristic* pStatusCharacteristic;
    BLECharacteristic* pCommandCharacteristic;
    bool deviceConnected;
    FsmManager* fsmManager;
};

#endif // BLE_MANAGER_H
