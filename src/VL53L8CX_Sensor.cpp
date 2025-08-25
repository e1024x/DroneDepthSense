#include "VL53L8CX_Sensor.h"
#include <Arduino.h>
#include <SPI.h>

const float VL53L8CX_Sensor::FOV_X = 45.0;
const float VL53L8CX_Sensor::FOV_Y = 45.0;

VL53L8CX_Sensor::VL53L8CX_Sensor(uint8_t csPin) : 
    _csPin(csPin),
    _posX(0),
    _posY(0),
    _posZ(0),
    _dirX(0),
    _dirY(0),
    _dirZ(1),
    _positionId(0)
{
    // config CS pin
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _dev.platform.cs_pin = _csPin;
    
    // init transformation matrix to identity
    _rotationMatrix.setIdentity();
    
    // init angles for FOV
    for (int i = 0; i < 8; i++) {
        _angles_x[i] = radians(-FOV_X/2.0 + (FOV_X/(8-1)) * i);
        _angles_y[i] = radians(-FOV_Y/2.0 + (FOV_Y/(8-1)) * i);
    }
}

void VL53L8CX_Sensor::setPosition(float x, float y, float z, float dirX, float dirY, float dirZ, uint8_t positionId) {
    _posX = x;
    _posY = y;
    _posZ = z;
    _dirX = dirX;
    _dirY = dirY;
    _dirZ = dirZ;
    _positionId = positionId;
    
    // calculate rotation matrix
    setupRotationMatrix();
}

void VL53L8CX_Sensor::normalizeVector(float &x, float &y, float &z) {
    float length = sqrt(x*x + y*y + z*z);
    if (length > 0) {
        x /= length;
        y /= length;
        z /= length;
    }
}

void VL53L8CX_Sensor::crossProduct(float ax, float ay, float az, float bx, float by, float bz, float &rx, float &ry, float &rz) {
    rx = ay * bz - az * by;
    ry = az * bx - ax * bz;
    rz = ax * by - ay * bx;
}

float VL53L8CX_Sensor::dotProduct(float ax, float ay, float az, float bx, float by, float bz) {
    return ax * bx + ay * by + az * bz;
}

void VL53L8CX_Sensor::setupRotationMatrix() {
    // get the direction vector (z-axis in sensor space)
    float zAxisX = _dirX;
    float zAxisY = _dirY;
    float zAxisZ = _dirZ;
    normalizeVector(zAxisX, zAxisY, zAxisZ);
    
    float upX = 0, upY = 0, upZ = 1;
    
    if ((abs(zAxisZ) > 0.99) && (abs(zAxisX) < 0.01) && (abs(zAxisY) < 0.01)) {
        float xAxisX = 1, xAxisY = 0, xAxisZ = 0;
        
        float yAxisX, yAxisY, yAxisZ;
        crossProduct(zAxisX, zAxisY, zAxisZ, xAxisX, xAxisY, xAxisZ, yAxisX, yAxisY, yAxisZ);
        normalizeVector(yAxisX, yAxisY, yAxisZ);
        
        // fill in rotation matrix columns
        _rotationMatrix.m[0][0] = xAxisX; _rotationMatrix.m[1][0] = xAxisY; _rotationMatrix.m[2][0] = xAxisZ;
        _rotationMatrix.m[0][1] = yAxisX; _rotationMatrix.m[1][1] = yAxisY; _rotationMatrix.m[2][1] = yAxisZ;
        _rotationMatrix.m[0][2] = zAxisX; _rotationMatrix.m[1][2] = zAxisY; _rotationMatrix.m[2][2] = zAxisZ;
    } else {
        // alculate X-axis (right) as cross product of direction and global up
        float xAxisX, xAxisY, xAxisZ;
        crossProduct(zAxisX, zAxisY, zAxisZ, upX, upY, upZ, xAxisX, xAxisY, xAxisZ);
        normalizeVector(xAxisX, xAxisY, xAxisZ);
        
        // calculate Y-axis (up) as cross product of Z and X
        float yAxisX, yAxisY, yAxisZ;
        crossProduct(zAxisX, zAxisY, zAxisZ, xAxisX, xAxisY, xAxisZ, yAxisX, yAxisY, yAxisZ);
        normalizeVector(yAxisX, yAxisY, yAxisZ);
        
        // fill in rotation matrix columns
        _rotationMatrix.m[0][0] = xAxisX; _rotationMatrix.m[1][0] = xAxisY; _rotationMatrix.m[2][0] = xAxisZ;
        _rotationMatrix.m[0][1] = yAxisX; _rotationMatrix.m[1][1] = yAxisY; _rotationMatrix.m[2][1] = yAxisZ;
        _rotationMatrix.m[0][2] = zAxisX; _rotationMatrix.m[1][2] = zAxisY; _rotationMatrix.m[2][2] = zAxisZ;
    }
    
    Serial.print("Rotation matrix for sensor ");
    Serial.println(_positionId);
    for (int i = 0; i < 3; i++) {
        Serial.print("  ");
        for (int j = 0; j < 3; j++) {
            Serial.print(_rotationMatrix.m[i][j]);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void VL53L8CX_Sensor::convertToXYZ(float distance, int row, int col, float& x, float& y, float& z) {
    x = distance * sin(_angles_y[row]) * cos(_angles_x[col]);
    y = distance * sin(_angles_x[col]);
    z = distance * cos(_angles_y[row]) * cos(_angles_x[col]);
}

void VL53L8CX_Sensor::transformPoint(float& x, float& y, float& z) {
    // apply rotation
    _rotationMatrix.transform(x, y, z);
    
    // apply translation
    x += _posX;
    y += _posY;
    z += _posZ;
}

String VL53L8CX_Sensor::getXYZDataPoint(int row, int col) {
    float distance = _results.distance_mm[row * 8 + col];
    String dataPoint = "";
    
    // skip invalid distances
    if (distance <= 0) {
        dataPoint = ",,,,";  // x,y,z,distance,
        return dataPoint;
    }
    
    float x, y, z;
    convertToXYZ(distance, row, col, x, y, z);
    
    transformPoint(x, y, z);
    
    dataPoint += String(x);
    dataPoint += ",";
    dataPoint += String(y);
    dataPoint += ",";
    dataPoint += String(z);
    dataPoint += ",";
    dataPoint += String(distance);
    dataPoint += ",";
    
    return dataPoint;
}

String VL53L8CX_Sensor::getXYZDataMessage() {
    String message = "XYZ_DATA_";
    message += _positionId;
    message += "\n";
    
    // convert distance measurements to XYZ coordinates
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            message += getXYZDataPoint(row, col);
        }
    }
    
    message += "\n";  // final newline delimiter
    return message;
}

uint8_t VL53L8CX_Sensor::init() {
    uint8_t status = vl53l8cx_init(&_dev);
    return status;
}

bool VL53L8CX_Sensor::isAlive() {
    uint8_t isAlive = 0;
    uint8_t status;
    status = vl53l8cx_is_alive(&_dev, &isAlive);
    return (isAlive == 1) && (status == VL53L8CX_STATUS_OK);
}

uint8_t VL53L8CX_Sensor::startRanging() {
    uint8_t status = vl53l8cx_start_ranging(&_dev);
    return status;
}

uint8_t VL53L8CX_Sensor::stopRanging() {
    uint8_t status = vl53l8cx_stop_ranging(&_dev);
    return status;
}

bool VL53L8CX_Sensor::checkDataReady() {
    uint8_t dataReady = 0;
    uint8_t status = vl53l8cx_check_data_ready(&_dev, &dataReady);
    return (dataReady == 1) && (status == VL53L8CX_STATUS_OK);
}

uint8_t VL53L8CX_Sensor::getRangingData() {
    uint8_t status = vl53l8cx_get_ranging_data(&_dev, &_results);
    return status;
}

uint8_t VL53L8CX_Sensor::setResolution(uint8_t resolution) {
    uint8_t status = vl53l8cx_set_resolution(&_dev, resolution);
    return status;
}

uint8_t VL53L8CX_Sensor::setRangingFrequency(uint8_t frequencyHz) {
    uint8_t status = vl53l8cx_set_ranging_frequency_hz(&_dev, frequencyHz);
    return status;
}

const VL53L8CX_ResultsData* VL53L8CX_Sensor::getResults() const {
    return &_results;
}

VL53L8CX_Configuration* VL53L8CX_Sensor::getDeviceConfig() {
    return &_dev;
}

uint8_t VL53L8CX_Sensor::getCsPin() const {
    return _csPin;
}

uint8_t VL53L8CX_Sensor::getPositionId() const {
    return _positionId;
}

const Matrix3x3& VL53L8CX_Sensor::getRotationMatrix() const {
    return _rotationMatrix;
}