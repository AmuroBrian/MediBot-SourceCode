#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Motor Pins ---
#define PIN_MOTOR_EN_L 14
#define PIN_MOTOR_IN1_L 26
#define PIN_MOTOR_IN2_L 27
#define PIN_MOTOR_IN3_R 25
#define PIN_MOTOR_IN4_R 33
#define PIN_MOTOR_EN_R 32

// --- Peripherals ---
#define PIN_BUZZER 2
#define PIN_BUTTON 4

// --- Ultrasonic Sensor Pins ---
#define TRIG_LEFT 12
#define ECHO_LEFT 13
#define TRIG_RIGHT 5
#define ECHO_RIGHT 18

// --- Constants ---
#define MAX_ROOMS 4
#define MAX_QUEUE_SIZE 10
#define AVOID_DISTANCE_CM 5 // Distance to stop and avoid obstacle (3-5 cm)
#define HALLWAY_WIDTH_CM 80  // Typical hallway width
#define MAX_SPEED 220 // Reduced motor speed for stability

// --- Robot States ---
enum RobotState {
    STATE_IDLE,
    STATE_PLAN_ROUTE,
    STATE_EXECUTE_CMD,
    STATE_AVOID_OBSTACLE,
    STATE_DELIVER_MEDICINE,
    STATE_WAITING_FOR_ACK,
    STATE_NEXT_ROOM,
    STATE_RETURN_HOME,
    STATE_STOP,
    STATE_MANUAL_DRIVE,
    STATE_ERROR
};

// --- Delivery Modes ---
enum DeliveryMode {
    MODE_RANDOM,
    MODE_PRIORITY,
    MODE_STRICT
};

#endif // CONFIG_H
