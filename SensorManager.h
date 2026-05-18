#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

class SensorManager {
public:
    SensorManager();
    void init();
    void update();
    
    int getLeftDistance() const;
    int getRightDistance() const;
    bool isObstacleAhead() const;

private:
    int leftDistance;
    int rightDistance;
    
    unsigned long lastReadTime;
    const unsigned long READ_INTERVAL = 50; // ms between reads
    
    int measureDistance(uint8_t trigPin, uint8_t echoPin);
};

#endif // SENSOR_MANAGER_H
