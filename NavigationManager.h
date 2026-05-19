#ifndef NAVIGATION_MANAGER_H
#define NAVIGATION_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

class NavigationManager {
public:
    NavigationManager();
    void init();
    void setMotors(int leftSpeed, int rightSpeed);
    
    // Core Movement
    void moveForward(int speed);
    void moveBackward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stop();
    
    void driveStraightCorrection(int baseSpeed);
    void followWall(int leftDistance, int rightDistance, int baseSpeed);

private:
    Adafruit_ADXL345_Unified accel;
    bool adxlFound;
};

#endif // NAVIGATION_MANAGER_H
