#pragma once

#include <Eigen/Dense>

class INSMechanization {
private:

    Eigen::Matrix3f Cn_b;                                              // (C^n_b) Orientation Matrix
    Eigen::Vector3f vn_eb;                                             // (v^n_eb) NED Velocity (m/s)
    float L_b;                                                         // Geodetic Latitude (Rad)
    float lambda_b;                                                    // Geodetic Longitude (Rad)
    float h_b;                                                         // Geodetic Height (M)

    // WGS84 Reference Ellipsoid
    static constexpr float R_0 = 6378137.0;                            // Equatorial radius or Semi major axis (m)
    static constexpr float E_SQR = 0.00669437999014;                   // Squared eccentricity (e^2)
    static constexpr float w_ie = 7.292115e-5;                         // Earth rotation rate (rad/s)

    // =====================================================================
    // 3. HELPER FUNCTIONS FOR STAGE 1
    // Mathematical calculations from Equations 5.41, 5.43, and 5.44
    // =====================================================================
    
    // Utility to create a skew-symmetric matrix [x_wedge]
    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& vec) const;
    
    // Computes Earth's Meridian (R_N) and Transverse (R_E) radii of curvature
    void computeEarthRadii(float current_L_b, float& out_R_N, float& out_R_E) const;
    
    // Eq (5.41): Computes Earth rotation matrix in the navigation frame
    Eigen::Matrix3f computeOhmegan_ie(float current_L_b) const;
    
    // Eq (5.44): Computes Transport Rate matrix in the navigation frame
    Eigen::Matrix3f computeOhmegan_en(const Eigen::Vector3f& current_vn_eb, 
                                      float current_L_b, 
                                      float current_h_b, 
                                      float R_N, 
                                      float R_E) const;

    // =====================================================================
    // 4. EXECUTION FUNCTIONS FOR STAGE 1
    // =====================================================================
    
    // Eq (5.46): First-order approximation attitude update
    void updateAttitude(const Eigen::Vector3f& omegab_ib, float T_i);

public:
    INSMechanization();

    void diffCn_b(float Cn_b, float Omegab_ib, float Omegan_ie, float Omegan_en);
    // Initializes starting position and orientation
    void initialize(float init_L_b, float init_lambda_b, float init_h_b, 
                    float roll_rad, float pitch_rad, float yaw_rad);

    // Main update function called in the loop (calls updateAttitude internally)
    // fb_ib : Specific force from the Accelerometer (m/s^2)
    // omegab_ib : Angular rate from the Gyroscope (rad/s)
    // T_i : Time step (\tau_i) in seconds
    void update(const Eigen::Vector3f& fb_ib, const Eigen::Vector3f& omegab_ib, float T_i);

    // Getters
    Eigen::Matrix3f getOrientation() const { return Cn_b; }
    Eigen::Vector3f getVelocity() const { return vn_eb; }
    void getPosition(float& out_L_b, float& out_lambda_b, float& out_h_b) const;
};