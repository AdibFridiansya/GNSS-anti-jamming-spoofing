#include "INSMechanization.h"

INSMechanization::INSMechanization() {
    Cn_b = Eigen::Matrix3f::Identity();
    vn_eb = Eigen::Vector3f::Zero();
    L_b = 0.0;
    lambda_b = 0.0;
    h_b = 0.0;
}

// Initialization C^n_b Orientation Matrix
void INSMechanization::initialize(double init_L_b, double init_lambda_b, double init_h_b, 
                                double roll_rad, double pitch_rad, double yaw_rad){
    L_b = init_L_b;
    lambda_b = init_lambda_b;
    h_b = init_h_b;

    Eigen::AngleAxisf rollAngle(roll_rad, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch_rad, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw_rad, Eigen::Vector3f::UnitZ());

    Cn_b = yawAngle * pitchAngle * rollAngle;               // (C^n_b) Orientation Matrix
}

// Find Omega^n_ie
void computeOhmegan_ie(double L_b, Eigen::Matrix3f& Omegan_ie) {
    /* For navigation purposes, a constant rotation rate 
    is assumed based on the mean sidereal day 
    
    (Anon., Department of Defense World Geodetic System 1984, 
    National Imagery and Mapping Agency (now NGA), TR8350.2, 
    Third Edition, 1997.)
    */
    const double w_ie = 7.292115e-5;                        // Earth’s angular rate (rad/s)                  

    double sin_L = std::sin(L_b);
    double cos_L = std::cos(L_b);

    Omegan_ie <<  0.0,    sin_L,   0.0,
                 -sin_L,   0.0,   -cos_L,
                  0.0,    cos_L,   0.0;

    Omegan_ie = w_ie * Omegan_ie;                           // (Omega^n_ie) Skew-symmetric Matrix
}

// Eq. (5.40) 
void INSMechanization::updateAttitude(const Eigen::Vector3f& gyr, double T_i) {
    // Convert gyroscope raw angular rate vector into skew-symmetric matrix [Omegab_ib]
    Eigen::Matrix3f Omegab_ib = skewSymmetric(gyr);

    // Compute Earth rotation matrix in navigation frame [Omegan_ie]
    Eigen::Matrix3f Omegan_ie = computeOhmegan_ie(L_b);

    // Compute transport rate matrix in navigation frame [Omegan_en]
    double R_N, R_E;
    computeEarthRadii(L_b, R_N, R_E);
    Eigen::Matrix3f Omegan_en = computeOhmegan_en(vn_eb, L_b, h_b, R_N, R_E);

    // Eq (5.40): Compute orientation matrix time derivative (diffCn_b)
    Eigen::Matrix3f diffCn_b = Cn_b * Omegab_ib - (Omegan_ie + Omegan_en) * Cn_b;

    // (Next step: Discrete integration update for Cn_b using T_i...)
}

