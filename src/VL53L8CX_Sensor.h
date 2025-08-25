#ifndef VL53L8CX_SENSOR_H
#define VL53L8CX_SENSOR_H

#include "vl53l8cx_api.h"
#include "platform.h"
#include <math.h>
#include <Arduino.h> 

// Matrix3x3 for coordinate transformations
struct Matrix3x3 {
  float m[3][3];
  
  void setIdentity() {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        m[i][j] = (i == j) ? 1.0f : 0.0f;
      }
    }
  }
  
  void transform(float &x, float &y, float &z) const {
    float nx = m[0][0] * x + m[0][1] * y + m[0][2] * z;
    float ny = m[1][0] * x + m[1][1] * y + m[1][2] * z;
    float nz = m[2][0] * x + m[2][1] * y + m[2][2] * z;
    x = nx;
    y = ny;
    z = nz;
  }
};

class VL53L8CX_Sensor {
private:
    uint8_t _csPin;
    VL53L8CX_Configuration _dev;
    VL53L8CX_ResultsData _results;
    
    // pos and orientation
    float _posX;
    float _posY;
    float _posZ;
    float _dirX;
    float _dirY;
    float _dirZ;
    uint8_t _positionId;
    
    // transf matrix
    Matrix3x3 _rotationMatrix;
    
    // FOV angles
    static const float FOV_X;
    static const float FOV_Y;
    float _angles_x[8];
    float _angles_y[8];
    
    // setup the rotation matrix based on direction vector
    void setupRotationMatrix();
    
    // util functions for vector operations
    void normalizeVector(float &x, float &y, float &z);
    void crossProduct(float ax, float ay, float az, float bx, float by, float bz, float &rx, float &ry, float &rz);
    float dotProduct(float ax, float ay, float az, float bx, float by, float bz);

public:
    VL53L8CX_Sensor(uint8_t csPin);
    
    // set the position and orientation of this sensor
    void setPosition(float x, float y, float z, float dirX, float dirY, float dirZ, uint8_t positionId);
    
    // basic sensor operations
    uint8_t init();
    bool isAlive();
    uint8_t startRanging();
    uint8_t stopRanging();
    bool checkDataReady();
    uint8_t getRangingData();
    uint8_t setResolution(uint8_t resolution);
    uint8_t setRangingFrequency(uint8_t frequencyHz);
    
    // conv distance to xyz coordinates
    void convertToXYZ(float distance, int row, int col, float& x, float& y, float& z);
    
    // transf point from sensor space to global space
    void transformPoint(float& x, float& y, float& z);
    
    // get transformed data point as string for TCP transmission
    String getXYZDataPoint(int row, int col);
    
    // get full data message for TCP transmission
    String getXYZDataMessage();
    
    // getters
    const VL53L8CX_ResultsData* getResults() const;
    VL53L8CX_Configuration* getDeviceConfig();
    uint8_t getCsPin() const;
    uint8_t getPositionId() const;
    const Matrix3x3& getRotationMatrix() const;
    const uint16_t* getInterpolatedMatrix() const;
};

#endif // VL53L8CX_SENSOR_H