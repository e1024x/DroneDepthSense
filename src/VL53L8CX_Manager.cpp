#include "VL53L8CX_Manager.h"

VL53L8CX_Manager::VL53L8CX_Manager() : _initialized(false) {
    // constructor
}

bool VL53L8CX_Manager::addSensor(VL53L8CX_Sensor* sensor) {
    if (sensor == nullptr) {
        return false;
    }
    
    _sensors.push_back(sensor);
    return true;
}

bool VL53L8CX_Manager::initializeSensors() {
    bool allInitialized = true;
    
    for (auto sensor : _sensors) {
        if (sensor->init() != VL53L8CX_STATUS_OK) {
            allInitialized = false;
        }
    }
    
    _initialized = allInitialized;
    return allInitialized;
}

bool VL53L8CX_Manager::startRanging() {
    if (!_initialized) {
        return false;
    }
    
    bool allStarted = true;
    
    for (auto sensor : _sensors) {

        uint8_t status = sensor->startRanging();
        Serial.println(status);
        if (status != VL53L8CX_STATUS_OK) {
            allStarted = false;
        }
        //delay(1);
    }
    
    return allStarted;
}

bool VL53L8CX_Manager::stopRanging() {
    bool allStopped = true;
    
    for (auto sensor : _sensors) {
        if (sensor->stopRanging() != VL53L8CX_STATUS_OK) {
            allStopped = false;
        }
    }
    
    return allStopped;
}

size_t VL53L8CX_Manager::getSensorCount() const {
    return _sensors.size();
}

VL53L8CX_Sensor* VL53L8CX_Manager::getSensor(size_t index) {
    if (index >= _sensors.size()) {
        return nullptr;
    }
    
    return _sensors[index];
}

bool VL53L8CX_Manager::configureAllSensors(uint8_t resolution, uint8_t frequencyHz) {
    bool allConfigured = true;
    
    for (auto sensor : _sensors) {
        // set resolution
        if (sensor->setResolution(resolution) != VL53L8CX_STATUS_OK) {
            allConfigured = false;
        }
        
        // set ranging frequency
        if (sensor->setRangingFrequency(frequencyHz) != VL53L8CX_STATUS_OK) {
            allConfigured = false;
        }
    }
    
    return allConfigured;
}