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
                                float roll_rad, float pitch_rad, float yaw_rad){
    L_b = init_L_b;
    lambda_b = init_lambda_b;
    h_b = init_h_b;
    vn_eb = Eigen::Vector3f::Zero();

    Eigen::AngleAxisf rollAngle(roll_rad, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch_rad, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw_rad, Eigen::Vector3f::UnitZ());

    Cn_b = yawAngle * pitchAngle * rollAngle;                   // (C^n_b) Orientation Matrix
}

// Skew-Symmetric Matrix [vector X]
Eigen::Matrix3f INSMechanization::skewSymmetric(const Eigen::Vector3f& vec) const {
    Eigen::Matrix3f vector_skew;
    vector_skew <<    0.0f, -vec(2),  vec(1),
            vec(2),    0.0f, -vec(0),
           -vec(1),  vec(0),    0.0f;
    return vector_skew;
}

// Eq. (5.41) - Ohmega^n_ie
Eigen::Matrix3f INSMechanization::computeOhmegan_ie(double L_b) const {
    /* For navigation purposes, a constant rotation rate 
    is assumed based on the mean sidereal day 
    
    (Anon., Department of Defense World Geodetic System 1984, 
    National Imagery and Mapping Agency (now NGA), TR8350.2, 
    Third Edition, 1997.)
    */
    constexpr float w_ie = 7.292115e-5f;                        // Earth’s angular rate (rad/s)                  
    const float sin_L = static_cast<float>(std::sin(L_b));
    const float cos_L = static_cast<float>(std::cos(L_b));

    const float w_sin = w_ie * sin_L;
    const float w_cos = w_ie * cos_L;

    Eigen::Matrix3f Ohmegan_ie;
    Ohmegan_ie <<    0.0f,   w_sin,    0.0f,
                  -w_sin,    0.0f,  -w_cos,
                    0.0f,   w_cos,    0.0f;

    return Ohmegan_ie;                           
}

// Eq. (5.44) - Ohmegan_en
namespace{
    constexpr double R_0   = 6378137.0;                                          // Equatorial radius / semi-major axis (m)
    constexpr double E_SQR = 0.00669437999014;                                   // Squared eccentricity (e^2) 
}

void INSMechanization::computeEarth(double L_b, double& R_N_Lb, double& R_E_Lb) const {
    //Find sin^2 L (sin_L_sqr)
    const double sin_L = std::sin(L_b);
    const double sin_L_sqr = sin_L * sin_L;                                     

    // Eq. (2.105)
    const double denom_sqrt = std::sqrt(1.0 - E_SQR * sin_L_sqr);                // sqrt(1 - e^2 * sin^2(L))
    R_N_Lb = R_0 * (1.0 - E_SQR) / (denom_sqrt * denom_sqrt * denom_sqrt);
    // Eq. (2.106)
    R_E_Lb = R_0 / denom_sqrt;
}

Eigen::Matrix3f INSMechanization::computeOhmegan_en(const Eigen::Vector3f& vn_eb, 
                                                   double L_b, 
                                                   double h_b, 
                                                   double R_N_Lb, 
                                                   double R_E_Lb) 
const {
    const double v_N = static_cast<double>(vn_eb(0));
    const double v_E = static_cast<double>(vn_eb(1));

    const double tan_Lb = std::tan(L_b);

    // Eq. (5.44) - w^n_en
    const float w_en_x = static_cast<float>(v_E / (R_E_Lb + h_b));                  //North
    const float w_en_y = static_cast<float>(-v_N / (R_N_Lb + h_b));                 //East
    const float w_en_z = static_cast<float>(-v_E * tan_Lb / (R_E_Lb + h_b));        //Down

    // Omega^n_en
    Eigen::Matrix3f Omegan_en;
    Omegan_en <<     0.0f, -w_en_z,   w_en_y,
                  w_en_z,     0.0f,  -w_en_x,
                 -w_en_y,   w_en_x,     0.0f;

    return Omegan_en;
}