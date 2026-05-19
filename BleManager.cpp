#include "BleManager.h"
#include "FsmManager.h"

BleManager::BleManager() : pServer(nullptr), pStatusCharacteristic(nullptr), pCommandCharacteristic(nullptr), deviceConnected(false), fsmManager(nullptr) {}

void BleManager::init(FsmManager* fsm) {
    this->fsmManager = fsm;
    
    BLEDevice::init("MediBot_ESP32");
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(this);
    
    BLEService* pService = pServer->createService(SERVICE_UUID);
    
    // Status characteristic: Notify only
    pStatusCharacteristic = pService->createCharacteristic(
        STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pStatusCharacteristic->addDescriptor(new BLE2902());
    
    // Command characteristic: Write only
    pCommandCharacteristic = pService->createCharacteristic(
        COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCommandCharacteristic->setCallbacks(this);
    
    pService->start();
    
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

void BleManager::onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE Device Connected");
}

void BleManager::onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE Device Disconnected");
    BLEDevice::startAdvertising(); // restart advertising
}

void BleManager::onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    
    if (value.length() > 0) {
        Serial.print("Received BLE Payload: ");
        Serial.println(value);
        
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, value);
        
        if (error) {
            Serial.print("JSON Parse Failed: ");
            Serial.println(error.c_str());
            return;
        }
        
        // Check for specific commands
        if (doc.containsKey("command")) {
            String cmd = doc["command"];
            if (cmd == "EMERGENCY_STOP") {
                fsmManager->triggerEmergencyStop();
                return;
            } else if (cmd == "RETURN_HOME") {
                fsmManager->triggerReturnHome();
                return;
            } else if (cmd == "CONTINUE_DELIVERY") {
                fsmManager->triggerContinueDelivery();
                return;
            } else if (cmd == "MANUAL_MOVE") {
                if (doc.containsKey("direction")) {
                    fsmManager->handleManualMove(doc["direction"]);
                }
                return;
            }
        }
        
        // Parse delivery configuration
        if (doc.containsKey("mode") && doc.containsKey("rooms")) {
            String modeStr = doc["mode"];
            int priorityRoom = doc.containsKey("priorityRoom") ? doc["priorityRoom"].as<int>() : 0;
            
            DeliveryMode mode = MODE_STRICT;
            if (modeStr == "random") mode = MODE_RANDOM;
            else if (modeStr == "priority") mode = MODE_PRIORITY;
            
            int rooms[MAX_QUEUE_SIZE];
            int compartments[MAX_QUEUE_SIZE];
            int numRooms = 0;
            JsonArray arr = doc["rooms"].as<JsonArray>();
            JsonArray compArr = doc["compartments"].as<JsonArray>();
            for(int i = 0; i < arr.size(); i++) {
                if (numRooms < MAX_QUEUE_SIZE) {
                    rooms[numRooms] = arr[i].as<int>();
                    compartments[numRooms] = compArr[i].as<int>();
                    numRooms++;
                }
            }
            
            fsmManager->startDelivery(mode, priorityRoom, rooms, compartments, numRooms);
        }
    }
}

void BleManager::sendStatusUpdate(String state, int room, int battery, String msg, int leftDist, int rightDist) {
    if (!deviceConnected) return;

    StaticJsonDocument<256> doc;
    doc["state"] = state;
    doc["currentRoom"] = room;
    doc["currentCompartment"] = fsmManager->getCurrentCompartment();
    doc["battery"] = battery;
    doc["status"] = msg;
    doc["leftDist"] = leftDist;
    doc["rightDist"] = rightDist;
    
    char buffer[256];
    serializeJson(doc, buffer);
    
    pStatusCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    pStatusCharacteristic->notify();
}
