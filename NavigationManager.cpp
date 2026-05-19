#include "NavigationManager.h"
#include "Config.h"

NavigationManager::NavigationManager() : adxlFound(false) {}

void NavigationManager::init() {
    pinMode(PIN_MOTOR_EN_L, OUTPUT);
    pinMode(PIN_MOTOR_EN_R, OUTPUT);
    pinMode(PIN_MOTOR_IN1_L, OUTPUT);
    pinMode(PIN_MOTOR_IN2_L, OUTPUT);
    pinMode(PIN_MOTOR_IN3_R, OUTPUT);
    pinMode(PIN_MOTOR_IN4_R, OUTPUT);
    stop();

    adxlFound = accel.begin();
    if (adxlFound) {
        Serial.println("ADXL345 Initialized.");
        accel.setRange(ADXL345_RANGE_16_G);
    } else {
        Serial.println("ADXL345 NOT FOUND!");
    }
}

void NavigationManager::setMotors(int leftSpeed, int rightSpeed) {
    // Left Motor
    if (leftSpeed > 0) {
        digitalWrite(PIN_MOTOR_IN1_L, HIGH);
        digitalWrite(PIN_MOTOR_IN2_L, LOW);
        if (leftSpeed >= 255) digitalWrite(PIN_MOTOR_EN_L, HIGH);
        else analogWrite(PIN_MOTOR_EN_L, leftSpeed);
    } else if (leftSpeed < 0) {
        digitalWrite(PIN_MOTOR_IN1_L, LOW);
        digitalWrite(PIN_MOTOR_IN2_L, HIGH);
        if (leftSpeed <= -255) digitalWrite(PIN_MOTOR_EN_L, HIGH);
        else analogWrite(PIN_MOTOR_EN_L, -leftSpeed);
    } else {
        digitalWrite(PIN_MOTOR_IN1_L, LOW);
        digitalWrite(PIN_MOTOR_IN2_L, LOW);
        digitalWrite(PIN_MOTOR_EN_L, LOW);
    }

    // Right Motor
    if (rightSpeed > 0) {
        digitalWrite(PIN_MOTOR_IN3_R, HIGH);
        digitalWrite(PIN_MOTOR_IN4_R, LOW);
        if (rightSpeed >= 255) digitalWrite(PIN_MOTOR_EN_R, HIGH);
        else analogWrite(PIN_MOTOR_EN_R, rightSpeed);
    } else if (rightSpeed < 0) {
        digitalWrite(PIN_MOTOR_IN3_R, LOW);
        digitalWrite(PIN_MOTOR_IN4_R, HIGH);
        if (rightSpeed <= -255) digitalWrite(PIN_MOTOR_EN_R, HIGH);
        else analogWrite(PIN_MOTOR_EN_R, -rightSpeed);
    } else {
        digitalWrite(PIN_MOTOR_IN3_R, LOW);
        digitalWrite(PIN_MOTOR_IN4_R, LOW);
        digitalWrite(PIN_MOTOR_EN_R, LOW);
    }
}

void NavigationManager::stop() {
    setMotors(0, 0);
}

void NavigationManager::moveForward(int speed) {
    if (adxlFound) {
        driveStraightCorrection(speed);
    } else {
        setMotors(speed, speed);
    }
}

void NavigationManager::moveBackward(int speed) {
    setMotors(-speed, -speed);
}

void NavigationManager::turnLeft(int speed) {
    setMotors(-speed, speed);
}

void NavigationManager::turnRight(int speed) {
    setMotors(speed, -speed);
}

void NavigationManager::driveStraightCorrection(int baseSpeed) {
    if (!adxlFound) {
        setMotors(baseSpeed, baseSpeed);
        return;
    }
    sensors_event_t event;
    accel.getEvent(&event);
    
    // Basic proportional correction based on lateral (Y) acceleration
    // event.acceleration.y is in m/s^2. 
    // If robot slips/turns, lateral acceleration changes slightly.
    float lateralAccel = event.acceleration.y;
    int correction = (int)(lateralAccel * 10); // Kp = 10
    
    int leftS = baseSpeed + correction;
    int rightS = baseSpeed - correction;
    
    // Constrain to positive
    if (leftS > 255) leftS = 255;
    if (leftS < 0) leftS = 0;
    if (rightS > 255) rightS = 255;
    if (rightS < 0) rightS = 0;
    
    setMotors(leftS, rightS);
}

void NavigationManager::followWall(int leftDist, int rightDist, int baseSpeed) {
    // Proportional control to keep centered in the hallway
    int error = leftDist - rightDist;
    int kP = 2; // Proportional gain
    int adjustment = error * kP;
    
    // Limit adjustment to prevent jerky movements
    if (adjustment > 50) adjustment = 50;
    if (adjustment < -50) adjustment = -50;
    
    int leftSpeed = baseSpeed + adjustment;
    int rightSpeed = baseSpeed - adjustment;
    
    // Ensure speeds are within bounds 0-255
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    setMotors(leftSpeed, rightSpeed);
}
