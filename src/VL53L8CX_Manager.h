#ifndef VL53L8CX_MANAGER_H
#define VL53L8CX_MANAGER_H

#include <Arduino.h>
#include <vector>
#include "VL53L8CX_Sensor.h"

class VL53L8CX_Manager {
private:
    std::vector<VL53L8CX_Sensor*> _sensors;
    bool _initialized;

public:
    VL53L8CX_Manager();
    
    bool addSensor(VL53L8CX_Sensor* sensor);
    bool initializeSensors();
    bool startRanging();
    bool stopRanging();
    
    size_t getSensorCount() const;
    VL53L8CX_Sensor* getSensor(size_t index);
    bool configureAllSensors(uint8_t resolution, uint8_t frequencyHz);
};

#endif // VL53L8CX_MANAGER_H