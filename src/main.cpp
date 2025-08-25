// main.cpp
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include "VL53L8CX_Sensor.h"
#include "VL53L8CX_Manager.h"
#include "DroneController.h"
#include "WiFiManager.h"
#include "SensorProcessor.h"
#include "MotorController.h"

// conf constants
const char* WIFI_SSID = "esp32";
const char* WIFI_PASSWORD = "12345678xd";
const uint16_t SERVER_PORT = 8080;

// hw pin definitions
const uint8_t SENSOR1_CS_PIN = 4;
const uint8_t SENSOR2_CS_PIN = 5;
const uint8_t RX1_PIN = 16;
const uint8_t TX1_PIN = 17;
const uint32_t BAUD_RATE = 38400;

// system components
WiFiManager wifiManager(WIFI_SSID, WIFI_PASSWORD, SERVER_PORT);
VL53L8CX_Manager sensorManager;
VL53L8CX_Sensor sensor1(SENSOR1_CS_PIN);
VL53L8CX_Sensor sensor2(SENSOR2_CS_PIN);
MotorController motorController(RX1_PIN, TX1_PIN, BAUD_RATE);
SensorProcessor sensorProcessor(motorController);
DroneController droneController(wifiManager, sensorManager, sensorProcessor);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // init all systems
  if (!droneController.initialize()) {
    Serial.println("Failed to initialize drone controller");
    ESP.restart();
  }

  // config sensors
  sensor1.setPosition(0, 70, 0, 0, 0, 1, POSITION_TOP);
  sensor2.setPosition(0, -70, 0, 0, 0, 1, POSITION_BOTTOM);
  
  sensorManager.addSensor(&sensor1);
  sensorManager.addSensor(&sensor2);

  // wait for sensors and initialize them
  if (!droneController.setupSensors()) {
    Serial.println("Failed to setup sensors");
    ESP.restart();
  }

  Serial.println("Setup complete - system ready");
}

void loop() {
  droneController.update();
  delay(66); // ~15Hz update rate
}