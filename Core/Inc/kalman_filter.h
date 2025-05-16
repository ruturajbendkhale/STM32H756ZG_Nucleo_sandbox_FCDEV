#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// Kalman Filter State Structure
typedef struct {
    // State vector: x = [altitude; vertical_velocity]
    float altitude_m;         // Estimated altitude (m)
    float vertical_velocity_mps; // Estimated vertical velocity (m/s)

    // State covariance matrix P (2x2)
    // P = [[P00, P01],
    //      [P10, P11]]
    // Since P is symmetric, P01 = P10
    float P[2][2];

    // Process noise variance for acceleration (sigma_a^2)
    // Used to compute Q matrix
    float Q_accel_variance; // (m/s^2)^2

    // Measurement noise variance for barometer altitude (R_baro)
    float R_baro_altitude_variance; // m^2

} KalmanFilter;

/**
 * @brief Initializes the Kalman filter state.
 * @param kf Pointer to the KalmanFilter structure.
 * @param initial_altitude_m Initial guess for altitude (e.g., from first barometer reading).
 * @param initial_altitude_variance Variance of the initial altitude guess.
 * @param initial_velocity_variance Variance of the initial velocity guess (usually larger if unknown).
 * @param process_noise_accel_variance Variance of the unmodeled acceleration (sigma_a^2 for Q matrix).
 * @param measurement_noise_baro_variance Variance of the barometer altitude measurement (R_baro).
 */
void kalman_init(KalmanFilter *kf,
                 float initial_altitude_m,
                 float initial_altitude_variance,
                 float initial_velocity_variance,
                 float process_noise_accel_variance,
                 float measurement_noise_baro_variance);

/**
 * @brief Performs the prediction step of the Kalman filter.
 * @param kf Pointer to the KalmanFilter structure.
 * @param vertical_accel_mps2 Measured/derived vertical acceleration in world frame (m/s^2).
 * @param dt Time step (s).
 */
void kalman_predict(KalmanFilter *kf, float vertical_accel_mps2, float dt);

/**
 * @brief Performs the update step of the Kalman filter using barometer altitude measurement.
 * @param kf Pointer to the KalmanFilter structure.
 * @param baro_altitude_m Measured altitude from barometer (m).
 */
void kalman_update_barometer(KalmanFilter *kf, float baro_altitude_m);

#endif // KALMAN_FILTER_H
