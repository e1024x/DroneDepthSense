// MotorController.cpp
#include "MotorController.h"

MotorController::MotorController(uint8_t rxPin, uint8_t txPin, uint32_t baudRate) 
    : inCliMode(false) {
    Serial1.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    delay(500);
    clearSerialBuffer();
}

void MotorController::setMotorSpeed(MotorIndex motorIndex, uint16_t speed) {
    sendCLICommand(motorIndex, speed);
}

void MotorController::stopMotor(MotorIndex motorIndex) {
    sendCLICommand(motorIndex, MotorConfig::MOTOR_OFF_SPEED);
}

uint16_t MotorController::calculateSpeedFromDistance(float distance) {
    float clampedDistance = constrain(distance, 
                                    MotorConfig::MIN_DETECTION_DISTANCE, 
                                    MotorConfig::MAX_DETECTION_DISTANCE);
    
    uint16_t speed = map(clampedDistance, 
                        MotorConfig::MIN_DETECTION_DISTANCE, 
                        MotorConfig::MAX_DETECTION_DISTANCE,
                        MotorConfig::MAX_MOTOR_SPEED, 
                        MotorConfig::MIN_MOTOR_SPEED);
    
    return speed;
}

void MotorController::enterCLIMode() {
    if (!inCliMode) {
        Serial.println("Entering CLI mode");
        Serial1.write('#');
        delay(500);
        clearSerialBuffer();
        inCliMode = true;
    }
}

void MotorController::sendCLICommand(MotorIndex motorIndex, uint16_t value) {
    if (!inCliMode) {
        enterCLIMode();
    }
    
    String command = String("motor ") + String(motorIndex) + String(" ") + String(value);
    Serial.print("Sending CLI command: ");
    Serial.println(command);
    
    Serial1.print(command);
    Serial1.write(13);  // carriage return
    Serial1.write(10);  // line feed
    
    delay(200);
    clearSerialBuffer();
}

void MotorController::clearSerialBuffer() {
    while (Serial1.available()) {
        Serial1.read();
    }
}