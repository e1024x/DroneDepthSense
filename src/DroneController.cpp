// DroneController.cpp
#include "DroneController.h"
#include <Arduino.h>
#include <SPI.h>

DroneController::DroneController(WiFiManager& wifi, VL53L8CX_Manager& sensors, SensorProcessor& processor)
    : wifiManager(wifi), sensorManager(sensors), sensorProcessor(processor) {
}

bool DroneController::initialize() {
    // init WiFi
    if (!wifiManager.initialize()) {
        Serial.println("Failed to initialize WiFi");
        return false;
    }
    
    // init SPI
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    
    return true;
}

bool DroneController::setupSensors() {
    if (!waitForSensors()) {
        return false;
    }
    
    if (!configureSensors()) {
        return false;
    }
    
    return true;
}

bool DroneController::waitForSensors() {
    bool sensor1Alive = false;
    bool sensor2Alive = false;
    int retryCount = 0;
    
    Serial.println("Waiting for sensors to respond...");
    
    while (!sensor1Alive || !sensor2Alive) {
        for (size_t i = 0; i < sensorManager.getSensorCount(); i++) {
            VL53L8CX_Sensor* sensor = sensorManager.getSensor(i);
            if (sensor->getCsPin() == 4) {
                sensor1Alive = sensor->isAlive();
            } else if (sensor->getCsPin() == 5) {
                sensor2Alive = sensor->isAlive();
            }
        }
        
        Serial.print("Retry #");
        Serial.print(++retryCount);
        Serial.print(": Sensor 1 is ");
        Serial.print(sensor1Alive ? "alive" : "not responding");
        Serial.print(", Sensor 2 is ");
        Serial.println(sensor2Alive ? "alive" : "not responding");
        
        if (!sensor1Alive || !sensor2Alive) {
            delay(66);
        }
    }
    
    Serial.println("Both sensors are responding!");
    return true;
}

bool DroneController::configureSensors() {
    if (!sensorManager.initializeSensors()) {
        Serial.println("Failed to initialize sensors");
        return false;
    }
    
    if (!sensorManager.configureAllSensors(VL53L8CX_RESOLUTION_8X8, 15)) {
        Serial.println("Failed to configure sensors");
        return false;
    }
    
    if (!sensorManager.startRanging()) {
        Serial.println("Failed to start ranging");
        return false;
    }
    
    Serial.println("Sensors configured and ranging started");
    return true;
}

void DroneController::update() {
    wifiManager.handleClient();
    processSensorData();
}

void DroneController::processSensorData() {
    for (size_t sensorIndex = 0; sensorIndex < sensorManager.getSensorCount(); sensorIndex++) {
        VL53L8CX_Sensor* sensor = sensorManager.getSensor(sensorIndex);
        
        if (!sensor->isAlive()) {
            Serial.print("Sensor ");
            Serial.print(sensor->getCsPin());
            Serial.println(" not responding");
            continue;
        }
        
        if (sensor->checkDataReady()) {
            sensor->getRangingData();
            sensorProcessor.processSensor(sensor);
            
            // send data to WiFi client if connected
            WiFiClient client = wifiManager.getClient();
            if (client && client.connected()) {
                String message = sensor->getXYZDataMessage();
                client.print(message);
            }
        }
    }
}