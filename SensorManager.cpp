#include "SensorManager.h"
#include "Config.h"

SensorManager::SensorManager() : leftDistance(0), rightDistance(0), lastReadTime(0) {}

void SensorManager::init() {
    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);
}

void SensorManager::update() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentMillis;
        
        // In a real non-blocking scenario, pulseIn can block. 
        // For simplicity and typical Arduino IDE usage, we use short timeouts.
        leftDistance = measureDistance(TRIG_LEFT, ECHO_LEFT);
        
        // Small delay to prevent ultrasonic cross-talk
        delay(5);
        
        rightDistance = measureDistance(TRIG_RIGHT, ECHO_RIGHT);
    }
}

int SensorManager::measureDistance(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Timeout after ~30ms (approx 5 meters) to prevent long blocking
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 999; // No echo, assume clear
    
    return duration * 0.034 / 2;
}

int SensorManager::getLeftDistance() const {
    return leftDistance;
}

int SensorManager::getRightDistance() const {
    return rightDistance;
}

bool SensorManager::isObstacleAhead() const {
    // Basic heuristic: if both sensors report a close object, it's a wall or obstacle ahead.
    return (leftDistance < AVOID_DISTANCE_CM && rightDistance < AVOID_DISTANCE_CM);
}
