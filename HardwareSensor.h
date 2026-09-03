#pragma once

#include "RTIMULib.h"
#include <cstdint>                                                          // uint64_t timestamp

struct IMUDataPacket {
    float accX, accY, accZ;
    float gyrX, gyrY, gyrZ;
    float temperature;
    bool isTempValid;
    uint64_t timestamp; 
    bool isDataValid;  
};

class HardwareSensor {
private:
    RTIMUSettings *settings;
    RTIMU *imu;
    bool isInitialized;

public:
    HardwareSensor();
    ~HardwareSensor();
    bool initialize();                                                      // Inisialisasi I2C/SPI 

    IMUDataPacket readData();
};
