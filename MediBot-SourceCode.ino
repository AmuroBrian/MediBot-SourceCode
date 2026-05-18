#include <Arduino.h>
#include "Config.h"
#include "BleManager.h"
#include "SensorManager.h"
#include "NavigationManager.h"
#include "FsmManager.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lastScrollTime = 0;
int scrollPos = 0;

// Instantiate Managers
BleManager bleManager;
SensorManager sensorManager;
NavigationManager navManager;
FsmManager fsmManager(&bleManager, &sensorManager, &navManager);

unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 1000; // 1 second

void setup() {
    Serial.begin(115200);
    Serial.println("Starting MediBot...");

    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("MediBot Init...");

    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    digitalWrite(PIN_BUZZER, LOW);

    // Initialize Subsystems
    sensorManager.init();
    navManager.init();
    bleManager.init(&fsmManager);
    fsmManager.init();

    Serial.println("MediBot Initialization Complete.");
}

void loop() {
    // Non-blocking update loop
    sensorManager.update();
    fsmManager.update();

    unsigned long currentMillis = millis();

    // Scroll LCD text
    if (currentMillis - lastScrollTime >= 400) {
        lastScrollTime = currentMillis;
        lcd.clear();
        if (fsmManager.getCurrentState() == STATE_WAITING_FOR_ACK) {
            lcd.print("Click Button...");
            lcd.setCursor(0, 1);
            lcd.print("Open Comp: ");
            lcd.print(fsmManager.getCurrentCompartment());
        } else {
            lcd.print("MediBot");
            lcd.setCursor(0, 1);
            lcd.print(fsmManager.getCurrentStateString());
        }
        
        scrollPos++;
        if (scrollPos > 16) scrollPos = -7;
    }

    // Send periodic status update to App
    if (currentMillis - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
        lastStatusUpdate = currentMillis;
        bleManager.sendStatusUpdate(
            fsmManager.getCurrentStateString(),
            fsmManager.getCurrentRoom(),
            95, // Mock battery
            fsmManager.getStatusMessage()
        );
    }
}
