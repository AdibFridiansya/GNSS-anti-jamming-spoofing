#pragma once

#include "SensorSim.h"
#include "HardwareSensor.h"
#include <Eigen/Dense>

struct ProcessedIMUData {
    double time;
    Eigen::Vector3f acc;                                     // f (Final specific force)
    Eigen::Vector3f gyr;                                     // w (Final angular rate)
    bool isValid;
};

class IMUPreProcessing {
private:
    SimDataPacket currentSimData;                            // ~(f^b_ib) & ~(w^b_ib)
    IMUDataPacket currentHardwareData;                       // ~(f^b_ib) & ~(w^b_ib)

    // 4.4.1 Biases
    Eigen::Vector3f bias_AccStatic;                          // b_as
    Eigen::Vector3f bias_GyrStatic;                          // b_gs

    // 4.4.2 Scale Factor & Cross Coupling Errors 
    Eigen::Matrix3f matrix_AccScaleCross;                    // M_a   
    Eigen::Matrix3f matrix_GyrScaleCross;                    // M_g  
    Eigen::Matrix3f matrix_GyrGDependent;                    // G_g

    bool isUsingSimData;

public:
    IMUPreProcessing();
    ~IMUPreProcessing();

    void inputRawData(const SimDataPacket& rawData);
    void inputRawData(const IMUDataPacket& rawData);
    
    ProcessedIMUData getProcessedData();

    void setBiasesStatic(const Eigen::Vector3f& b_as, const Eigen::Vector3f& b_gs);                 // Setter Biases
    void setScaleCrossMatrices(const Eigen::Matrix3f& M_a, const Eigen::Matrix3f& M_g,              // Setter Scale Factor & Cross Coupling
        const Eigen::Matrix3f& G_g);                                                         
};
