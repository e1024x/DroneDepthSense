// SensorProcessor.h
#pragma once

#include "VL53L8CX_Sensor.h"
#include "MotorController.h"

enum Region {
    REGION_LEFT = 0,
    REGION_RIGHT = 1
};

enum SensorPosition {
    POSITION_TOP = 0,
    POSITION_BOTTOM = 1
};

struct DetectionState {
    bool leftDetected = false;
    bool rightDetected = false;
    float leftAvgDistance = 0;
    float rightAvgDistance = 0;
    int leftPoints = 0;
    int rightPoints = 0;
};

class SensorProcessor {
private:
    MotorController& motorController;
    DetectionState sensor1State;  // top sensor
    DetectionState sensor2State;  // bttm sensor
    
    static const int MIN_DETECTION_POINTS = 8;
    static const int MAX_DETECTION_DISTANCE = 1000;
    
public:
    SensorProcessor(MotorController& controller);
    
    void processSensor(VL53L8CX_Sensor* sensor);
    
private:
    Region getRegionFromXYZ(float x, float y, float z);
    DetectionState analyzeSensorData(VL53L8CX_Sensor* sensor);
    void updateMotorStates(int positionId, const DetectionState& newState, const DetectionState& oldState);
    void handleRegionDetection(MotorIndex motorIndex, bool newDetected, bool oldDetected, 
                              float avgDistance, int points, const char* regionName);
};

