// DroneController.h
#pragma once

#include "WiFiManager.h"
#include "VL53L8CX_Manager.h"
#include "SensorProcessor.h"

class DroneController {
private:
    WiFiManager& wifiManager;
    VL53L8CX_Manager& sensorManager;
    SensorProcessor& sensorProcessor;
    
public:
    DroneController(WiFiManager& wifi, VL53L8CX_Manager& sensors, SensorProcessor& processor);
    
    bool initialize();
    bool setupSensors();
    void update();
    
private:
    bool waitForSensors();
    bool configureSensors();
    void processSensorData();
};