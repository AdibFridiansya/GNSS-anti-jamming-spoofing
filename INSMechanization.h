#pragma once

#include <Eigen/Dense>

class INSMechanization {
private:
    Eigen::Matrix3f Cn_b;      // (C^n_b) Orientation Matrix
    Eigen::Vector3f vn_eb;     // (v^n_eb) NED Velocity (m/s)
    double L_b;                // Geodetic Latitude (rad)
    double lambda_b;           // Geodetic Longitude (rad)
    double h_b;                // Geodetic Height (m)


    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& vec) const;
    // Eq. (5.41)
    Eigen::Matrix3f computeOhmegan_ie(double L_b) const;
    void computeEarth(double L_b, double& R_N_Lb, double& R_E_Lb) const;
    // Eq. (5.44)
    Eigen::Matrix3f computeOhmegan_en(const Eigen::Vector3f& vn_eb, 
                                      double L_b, 
                                      double h_b, 
                                      double R_N_Lb, 
                                      double R_E_Lb) const;

public:
    INSMechanization();

    void initialize(double init_L_b, double init_lambda_b, double init_h_b, 
                    float roll_rad, float pitch_rad, float yaw_rad);

    // Getters
    Eigen::Matrix3f getOrientation() const { return Cn_b; }
    Eigen::Vector3f getVelocity() const { return vn_eb; }
    void getPosition(double& out_L_b, double& out_lambda_b, double& out_h_b) const {
        out_L_b = L_b;
        out_lambda_b = lambda_b;
        out_h_b = h_b;
    }
};