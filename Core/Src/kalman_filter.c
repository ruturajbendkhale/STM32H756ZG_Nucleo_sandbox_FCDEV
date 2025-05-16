#include "kalman_filter.h"
#include <string.h> // For memset, if used for matrix initialization, though direct assignment is clearer for 2x2.

// Note: No arm_math.h is used, all matrix operations are explicit.

void kalman_init(KalmanFilter *kf,
                 float initial_altitude_m,
                 float initial_altitude_variance,
                 float initial_velocity_variance,
                 float process_noise_accel_variance,
                 float measurement_noise_baro_variance) {
    kf->altitude_m = initial_altitude_m;
    kf->vertical_velocity_mps = 0.0f; // Assume initial vertical velocity is zero unless specified

    // Initialize covariance matrix P
    // P = [[var_alt, 0],
    //      [0, var_vel]]
    kf->P[0][0] = initial_altitude_variance;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = initial_velocity_variance;

    kf->Q_accel_variance = process_noise_accel_variance;
    kf->R_baro_altitude_variance = measurement_noise_baro_variance;
}

void kalman_predict(KalmanFilter *kf, float vertical_accel_mps2, float dt) {
    // State vector x = [altitude, velocity_z]'
    // State transition matrix A = [[1, dt],
    //                            [0,  1]]
    // Control input matrix B = [[0.5*dt^2],
    //                           [dt]]
    // Control input u = [vertical_accel_mps2]

    // Predict state: x_k_minus = A * x_k_minus_1_plus + B * u_k_minus_1
    float prev_alt = kf->altitude_m;
    float prev_vel = kf->vertical_velocity_mps;

    kf->altitude_m = prev_alt + prev_vel * dt + 0.5f * vertical_accel_mps2 * dt * dt;
    kf->vertical_velocity_mps = prev_vel + vertical_accel_mps2 * dt;

    // Predict covariance: P_k_minus = A * P_k_minus_1_plus * A' + Q
    // Q = [[0.25*dt^4, 0.5*dt^3],
    //      [0.5*dt^3,  dt^2]] * Q_accel_variance

    float p00 = kf->P[0][0];
    float p01 = kf->P[0][1];
    float p10 = kf->P[1][0]; // Should be equal to p01
    float p11 = kf->P[1][1];

    // A*P
    float ap00 = p00 + dt * p10;
    float ap01 = p01 + dt * p11;
    float ap10 = p10; // 0*p00 + 1*p10
    float ap11 = p11; // 0*p01 + 1*p11

    // (A*P)*A'
    // A' = [[1, 0],
    //       [dt,1]]
    float p_pred_00 = ap00 * 1.0f + ap01 * dt;
    float p_pred_01 = ap00 * 0.0f + ap01 * 1.0f;
    float p_pred_10 = ap10 * 1.0f + ap11 * dt;
    float p_pred_11 = ap10 * 0.0f + ap11 * 1.0f;
    
    // Add Q (Process Noise Covariance)
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    float q00 = 0.25f * dt4 * kf->Q_accel_variance;
    float q01 = 0.5f  * dt3 * kf->Q_accel_variance;
    // float q10 = q01; // Q is symmetric
    float q11 = dt2 * kf->Q_accel_variance;

    kf->P[0][0] = p_pred_00 + q00;
    kf->P[0][1] = p_pred_01 + q01;
    kf->P[1][0] = p_pred_10 + q01; // Using q01 for symmetry
    kf->P[1][1] = p_pred_11 + q11;
}

void kalman_update_barometer(KalmanFilter *kf, float baro_altitude_m) {
    // Measurement matrix H = [1, 0] (measures altitude)
    // Measurement z = baro_altitude_m

    // Innovation (measurement residual): y = z - H * x_k_minus
    float y_residual = baro_altitude_m - kf->altitude_m; // H*x_k_minus is just kf->altitude_m

    // Innovation covariance: S = H * P_k_minus * H' + R
    // H * P_k_minus * H' = P_k_minus[0][0]
    float S_innovation_cov = kf->P[0][0] + kf->R_baro_altitude_variance;

    if (S_innovation_cov == 0.0f) {
        // Avoid division by zero, though unlikely if R_baro is non-zero
        return;
    }

    // Kalman Gain K = P_k_minus * H' * S^-1
    // P_k_minus * H' = [[P00], [P10]] (since H' = [[1],[0]])
    float K0 = kf->P[0][0] / S_innovation_cov;
    float K1 = kf->P[1][0] / S_innovation_cov;

    // Update state estimate: x_k_plus = x_k_minus + K * y
    kf->altitude_m = kf->altitude_m + K0 * y_residual;
    kf->vertical_velocity_mps = kf->vertical_velocity_mps + K1 * y_residual;

    // Update covariance estimate: P_k_plus = (I - K * H) * P_k_minus
    // I - K*H = [[1-K0,  0],
    //            [ -K1,  1]]
    float p00 = kf->P[0][0];
    float p01 = kf->P[0][1];
    // p10 = kf->P[1][0]; // same as p01
    float p11 = kf->P[1][1];

    kf->P[0][0] = (1.0f - K0) * p00;
    kf->P[0][1] = (1.0f - K0) * p01;
    kf->P[1][0] = -K1 * p00 + kf->P[1][0]; // kf->P[1][0] is p10 from P_k_minus
    kf->P[1][1] = -K1 * p01 + p11;
    
    // Ensure P remains symmetric after update (numerical stability)
    // Average P[0][1] and P[1][0] or copy one to the other
    // If P_k_minus was symmetric, and computations are exact, it should remain symmetric.
    // For robustness:
    // kf->P[1][0] = kf->P[0][1]; 
    // However, the (I-KH)P formula used above should maintain symmetry if P was symmetric.
    // More robust P update: P = (I-KH)P(I-KH)' + KRK' (Joseph form, but more complex)
    // For now, the P_updated = (I - K * H) * P_k_minus is standard and often sufficient.
}
