// SensorProcessor.cpp
#include "SensorProcessor.h"
#include <Arduino.h>

SensorProcessor::SensorProcessor(MotorController& controller) 
    : motorController(controller) {
}

void SensorProcessor::processSensor(VL53L8CX_Sensor* sensor) {
    int positionId = sensor->getPositionId();
    
    Serial.print("Processing sensor ");
    Serial.print(positionId == POSITION_TOP ? "1 (TOP)" : "2 (BOTTOM)");
    Serial.println();
    
    DetectionState newState = analyzeSensorData(sensor);
    
    if (positionId == POSITION_TOP) {
        updateMotorStates(positionId, newState, sensor1State);
        sensor1State = newState;
    } else {
        updateMotorStates(positionId, newState, sensor2State);
        sensor2State = newState;
    }
}

Region SensorProcessor::getRegionFromXYZ(float x, float y, float z) {
    return (x < 0) ? REGION_LEFT : REGION_RIGHT;
}

DetectionState SensorProcessor::analyzeSensorData(VL53L8CX_Sensor* sensor) {
    DetectionState state;
    float leftSum = 0, rightSum = 0;
    
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            String dataPoint = sensor->getXYZDataPoint(row, col);
            
            // parse XYZ values
            int commaIndex1 = dataPoint.indexOf(',');
            int commaIndex2 = dataPoint.indexOf(',', commaIndex1 + 1);
            int commaIndex3 = dataPoint.indexOf(',', commaIndex2 + 1);
            
            if (commaIndex1 > 0 && commaIndex2 > 0 && commaIndex3 > 0) {
                float x = dataPoint.substring(0, commaIndex1).toFloat();
                float y = dataPoint.substring(commaIndex1 + 1, commaIndex2).toFloat();
                float z = dataPoint.substring(commaIndex2 + 1, commaIndex3).toFloat();
                float distance = dataPoint.substring(commaIndex3 + 1).toFloat();
                
                // only process valid distances below threshold
                if (distance > 0 && distance < MAX_DETECTION_DISTANCE) {
                    Region region = getRegionFromXYZ(x, y, z);
                    
                    if (region == REGION_LEFT) {
                        leftSum += distance;
                        state.leftPoints++;
                    } else {
                        rightSum += distance;
                        state.rightPoints++;
                    }
                }
            }
        }
    }
    
    state.leftAvgDistance = (state.leftPoints > 0) ? leftSum / state.leftPoints : 0;
    state.rightAvgDistance = (state.rightPoints > 0) ? rightSum / state.rightPoints : 0;
    state.leftDetected = (state.leftPoints >= MIN_DETECTION_POINTS);
    state.rightDetected = (state.rightPoints >= MIN_DETECTION_POINTS);
    
    return state;
}

void SensorProcessor::updateMotorStates(int positionId, const DetectionState& newState, const DetectionState& oldState) {
    if (positionId == POSITION_TOP) {
        // sensor 1 (top) controls motors 0 and 1
        handleRegionDetection(MOTOR_SENSOR1_LEFT, newState.leftDetected, oldState.leftDetected,
                            newState.leftAvgDistance, newState.leftPoints, "Sensor 1 Left");
        
        handleRegionDetection(MOTOR_SENSOR1_RIGHT, newState.rightDetected, oldState.rightDetected,
                            newState.rightAvgDistance, newState.rightPoints, "Sensor 1 Right");
    } else {
        // sensor 2 (bottom) controls motors 2 and 3
        handleRegionDetection(MOTOR_SENSOR2_LEFT, newState.leftDetected, oldState.leftDetected,
                            newState.leftAvgDistance, newState.leftPoints, "Sensor 2 Left");
        
        handleRegionDetection(MOTOR_SENSOR2_RIGHT, newState.rightDetected, oldState.rightDetected,
                            newState.rightAvgDistance, newState.rightPoints, "Sensor 2 Right");
    }
}

void SensorProcessor::handleRegionDetection(MotorIndex motorIndex, bool newDetected, bool oldDetected,
                                          float avgDistance, int points, const char* regionName) {
    // update motor if state changed or if continuously detected (for speed updates)
    if (newDetected != oldDetected || (oldDetected && newDetected)) {
        if (newDetected) {
            uint16_t motorSpeed = motorController.calculateSpeedFromDistance(avgDistance);
            
            Serial.print(regionName);
            Serial.print(": Object detected (");
            Serial.print(points);
            Serial.print(" points, avg distance: ");
            Serial.print(avgDistance);
            Serial.print(" mm) - Setting motor ");
            Serial.print(motorIndex);
            Serial.print(" to speed ");
            Serial.println(motorSpeed);
            
            motorController.setMotorSpeed(motorIndex, motorSpeed);
        } else {
            Serial.print(regionName);
            Serial.print(": Object no longer detected - Stopping motor ");
            Serial.println(motorIndex);
            
            motorController.stopMotor(motorIndex);
        }
    }
}