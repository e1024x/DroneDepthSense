// MotorController.h
#pragma once

#include <Arduino.h>

// motor assignments
enum MotorIndex {
    MOTOR_SENSOR1_LEFT = 0,
    MOTOR_SENSOR1_RIGHT = 1,
    MOTOR_SENSOR2_LEFT = 2,
    MOTOR_SENSOR2_RIGHT = 3
};

// motor speed constants
struct MotorConfig {
    static const uint16_t MIN_DETECTION_DISTANCE = 150;
    static const uint16_t MAX_DETECTION_DISTANCE = 1000;
    static const uint16_t MIN_MOTOR_SPEED = 1050;
    static const uint16_t MAX_MOTOR_SPEED = 1100;
    static const uint16_t MOTOR_OFF_SPEED = 1000;
};

class MotorController {
private:
    bool inCliMode;
    
public:
    MotorController(uint8_t rxPin, uint8_t txPin, uint32_t baudRate);
    
    void setMotorSpeed(MotorIndex motorIndex, uint16_t speed);
    void stopMotor(MotorIndex motorIndex);
    uint16_t calculateSpeedFromDistance(float distance);
    
private:
    void enterCLIMode();
    void sendCLICommand(MotorIndex motorIndex, uint16_t value);
    void clearSerialBuffer();
};