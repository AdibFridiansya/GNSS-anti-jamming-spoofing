#include "HardwareSensor.h"
#include <iostream>

HardwareSensor::HardwareSensor() : settings(nullptr), imu(nullptr), isInitialized(false) {}

HardwareSensor::~HardwareSensor() {
    if (imu) {
        delete imu;
    }
    if (settings) {
        delete settings;
    }
}

bool HardwareSensor::initialize() {
    settings = new RTIMUSettings("RTIMULib");
    
    imu = RTIMU::createIMU(settings);
    
    if (imu == nullptr || imu->IMUType() == RTIMU_TYPE_NULL) {
        std::cerr << "Sensor IMU tidak ditemukan" << std::endl;
        return false;
    }

    imu->IMUInit();
    
    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(false);                                   // Magnetometer mati

    isInitialized = true;
    std::cout << "Sensor " << imu->IMUName() << " berhasil" << std::endl;
    return true;
}

IMUDataPacket HardwareSensor::readData() {
    IMUDataPacket packet = {}; 
    packet.isDataValid = false;

    if (!isInitialized) return packet;

    RTIMU_DATA imuData;

    if (imu->IMURead()) {
        imuData = imu->getIMUData();

        packet.accX = imuData.accel.x();
        packet.accY = imuData.accel.y();
        packet.accZ = imuData.accel.z();
        
        packet.gyrX = imuData.gyro.x();
        packet.gyrY = imuData.gyro.y();
        packet.gyrZ = imuData.gyro.z();

        packet.isTempValid = imuData.temperatureValid;               // Temperature
        if (packet.isTempValid) {
            packet.temperature = imuData.temperature;
        }

        packet.timestamp = imuData.timestamp;                        // Timestamp (ms)

        packet.isDataValid = true;
    }

    return packet;
}