/* 
 The matrix addition (I_3 + ^M) and the matrix inversion operation must be computed offline (Eq. 4.21)
 in a separate calibration software (e.g., MATLAB, Python) to avoid heavy real-time CPU load. 
 This C++ firmware only stores and applies the final pre-calculated matrices.
*/
#include "IMUPreProcessing.h"

IMUPreProcessing::IMUPreProcessing(){
    bias_AccStatic = Eigen::Vector3f::Zero();
    bias_GyrStatic = Eigen::Vector3f::Zero();
    /* Calibration (Vector Changed) Template Exmp.
    bias_AccStatic = Eigen::Vector3d(0.012, -0.005, 0.001);
    bias_GyrStatic = Eigen::Vector3d(0.002, 0.001, -0.003);
    */

    matrix_AccScaleCross = Eigen::Matrix3f::Identity();
    matrix_GyrScaleCross = Eigen::Matrix3f::Identity();
    matrix_GyrGDependent = Eigen::Matrix3f::Zero();                                     
    /* Calibration (Identity Matrix Changed) Template Exmp.
    matrix_AccScaleCross << 1.002,  0.001,  0.000,
                            0.001,  0.998,  0.002,
                            0.000,  0.002,  1.001;
    matrix_GyrScaleCross << 1.001,  0.000,  0.000,
                            0.000,  1.000,  0.000,
                            0.000,  0.000,  0.999;
    matrix_GyrGDependent << 0.0001, 0.0000, 0.0000,
                            0.0000, 0.0001, 0.0000,
                            0.0000, 0.0000, 0.0001;
    */

    isUsingSimData = false;                                                                             
}
IMUPreProcessing::~IMUPreProcessing(){
}

void IMUPreProcessing::inputRawData(const SimDataPacket& rawData){
    currentSimData = rawData;
    isUsingSimData = true;
}
void IMUPreProcessing::inputRawData(const IMUDataPacket& rawData){
    currentHardwareData = rawData;
    isUsingSimData = false;
}
void IMUPreProcessing::setBiasesStatic(const Eigen::Vector3f& b_as, const Eigen::Vector3f& b_gs){
    bias_AccStatic = b_as;
    bias_GyrStatic = b_gs;
}
void IMUPreProcessing::setScaleCrossMatrices(const Eigen::Matrix3f& M_a, const Eigen::Matrix3f& M_g,
    const Eigen::Matrix3f& G_g){
    matrix_AccScaleCross = M_a;                                                                         // or ^(M_a)
    matrix_GyrScaleCross = M_g;                                                                         // or ^(M_g)
    matrix_GyrGDependent = G_g;                                                                         // or ^(G_g)
}

ProcessedIMUData IMUPreProcessing::getProcessedData() {
    ProcessedIMUData outData;
    outData.isValid = false;

    Eigen::Vector3f f_raw;                                                                              // ~(f^b_ib)  
    Eigen::Vector3f w_raw;                                                                              // ~(w^b_ib)

    if (isUsingSimData) {
        f_raw << currentSimData.accX, currentSimData.accY, currentSimData.accZ;                         // struct SimDataPacket
        w_raw << currentSimData.gyrX, currentSimData.gyrY, currentSimData.gyrZ;
    } else {
        f_raw << currentHardwareData.accX, currentHardwareData.accY, currentHardwareData.accZ;          // struct IMUDataPacket
        w_raw << currentHardwareData.gyrX, currentHardwareData.gyrY, currentHardwareData.gyrZ;
    }

    // Eq.(4.13), b_ad (bias_AccDynamic) & b_gd(bias_GyrDynamic) will be Continued in EKF Equations
    Eigen::Vector3f f_unbiased = f_raw - bias_AccStatic;                                                                                                                    
    Eigen::Vector3f w_unbiased = w_raw - bias_GyrStatic;                                                

    // from Eq.(4.13) and Eq.(4.15), hence Eq.(4.19)
    Eigen::Vector3f f_carat = matrix_AccScaleCross * f_unbiased;                                        // ^(f^b_ib)

    // Eq.(4.20)
    Eigen::Vector3f w_g_corrected = w_unbiased - (matrix_GyrGDependent * f_carat);                                      
    Eigen::Vector3f w_carat = matrix_GyrScaleCross * w_g_corrected;                                     // ^(w^b_ib)   

    /* 
     'f_carat' and 'w_carat' hold the estimated values (^f) and (^w). 
     However, to maintain architectural consistency and simplicity in the software, 
     (^f) and (^w) are changed to be (f) and (w). 
    */
     
    outData.acc = f_carat;          // f 
    outData.gyr = w_carat;          // w
    
    outData.isValid = true;
    return outData;
}