#ifndef NAVIGATION_MANAGER_H
#define NAVIGATION_MANAGER_H

#include <Arduino.h>

class NavigationManager {
public:
    NavigationManager();
    void init();
    
    void stop();
    void moveForward(int speed);
    void moveBackward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    
    // Advanced logic
    void followWall(int leftDist, int rightDist, int baseSpeed);

private:
    void setMotors(int leftSpeed, int rightSpeed);
};

#endif // NAVIGATION_MANAGER_H
