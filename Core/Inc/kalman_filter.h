#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "arm_math.h"

/* USER CODE BEGIN Includes */
// If your main.c defines TARGET_LOOP_PERIOD_MS, you might need a way to access it here,
// or define KF_SAMPLING_TIME_S directly. For simplicity, we'll define it here.
// Consider passing sampling time as a parameter to init if it can vary.
/* USER CODE END Includes */

// --- KALMAN FILTER TYPEDEF ---
#define KF_STATE_DIM 3 // Dimension of the state vector [altitude, velocity, accel_bias]
#define KF_MEASUREMENT_DIM 1 // Dimension of the measurement vector [altitude_baro]
#define KF_PROCESS_NOISE_DIM 2 // Dimension of the process noise source vector (matches example)

typedef struct {
    float32_t t_sampl_s; // Sampling time in seconds

    // State vector: [altitude (m), velocity (m/s), accel_bias (m/s^2)]
    float32_t x_bar_data[KF_STATE_DIM]; // Estimated state x_k|k
    float32_t x_hat_data[KF_STATE_DIM]; // Predicted state x_k|k-1
    arm_matrix_instance_f32 x_bar;
    arm_matrix_instance_f32 x_hat;

    // State transition model: x_hat = Ad * x_bar + Bd * u
    float32_t Ad_data[KF_STATE_DIM * KF_STATE_DIM];
    float32_t Bd_data[KF_STATE_DIM * 1]; // Control input matrix (for accel input)
    arm_matrix_instance_f32 Ad;
    arm_matrix_instance_f32 Bd;

    // Covariance matrices
    float32_t P_bar_data[KF_STATE_DIM * KF_STATE_DIM]; // Estimated covariance P_k|k
    float32_t P_hat_data[KF_STATE_DIM * KF_STATE_DIM]; // Predicted covariance P_k|k-1
    arm_matrix_instance_f32 P_bar;
    arm_matrix_instance_f32 P_hat;

    // Process noise covariance Q_discrete = Gd * Q_source * Gd_T
    float32_t Gd_data[KF_STATE_DIM * KF_PROCESS_NOISE_DIM]; // Maps process noise sources to states
    float32_t Q_source_data[KF_PROCESS_NOISE_DIM * KF_PROCESS_NOISE_DIM]; // Variances of noise sources
    float32_t GdQGd_T_data[KF_STATE_DIM * KF_STATE_DIM]; // Resulting discrete process noise
    arm_matrix_instance_f32 Gd;
    arm_matrix_instance_f32 Q_source;
    arm_matrix_instance_f32 GdQGd_T;

    // Measurement model: y = H * x
    float32_t H_data[KF_MEASUREMENT_DIM * KF_STATE_DIM];
    arm_matrix_instance_f32 H;

    // Measurement noise covariance R
    float32_t R_val; // Scalar measurement noise variance

    // Kalman Gain K
    float32_t K_data[KF_STATE_DIM * KF_MEASUREMENT_DIM];
    arm_matrix_instance_f32 K;

    // Temporary matrices for calculations
    float32_t Ad_T_data[KF_STATE_DIM * KF_STATE_DIM];
    arm_matrix_instance_f32 Ad_T;
    float32_t H_T_data[KF_STATE_DIM * KF_MEASUREMENT_DIM]; // Transpose of H
    arm_matrix_instance_f32 H_T;

    // Buffers for matrix multiplications and inversions
    float32_t temp_P_H_T_data[KF_STATE_DIM * KF_MEASUREMENT_DIM];
    arm_matrix_instance_f32 temp_P_H_T;
    float32_t temp_H_P_H_T_data[KF_MEASUREMENT_DIM * KF_MEASUREMENT_DIM]; // Scalar for 1D measurement
    arm_matrix_instance_f32 temp_H_P_H_T;
    float32_t temp_K_H_data[KF_STATE_DIM * KF_STATE_DIM];
    arm_matrix_instance_f32 temp_K_H;
    float32_t I_temp_data[KF_STATE_DIM * KF_STATE_DIM]; // Identity matrix
    arm_matrix_instance_f32 I_temp;

} kalman_filter_state_t;


// --- KALMAN FILTER TUNING CONSTANTS ---
#define KF_TARGET_LOOP_PERIOD_MS (10) // Assuming 10ms loop, same as in main.c
#define KF_SAMPLING_TIME_S (KF_TARGET_LOOP_PERIOD_MS / 1000.0f)

// Standard deviations of noises
#define KF_STD_DEV_ACCEL_INPUT_NOISE (0.3f)  // Process noise std dev for accelerometer input (m/s^2)
#define KF_STD_DEV_BIAS_WALK_NOISE (0.03f)   // Process noise std dev for bias random walk (m/s^2 per sqrt(s) effect)
#define KF_STD_DEV_BARO_MEAS_NOISE (1.0f)    // Measurement noise std dev for barometer (m)

// Initial state uncertainties (std dev)
#define KF_INITIAL_UNCERTAINTY_ALTITUDE (2.0f)   // meters
#define KF_INITIAL_UNCERTAINTY_VELOCITY (1.0f)   // m/s
#define KF_INITIAL_UNCERTAINTY_ACCEL_BIAS (0.5f) // m/s^2


// --- FUNCTION PROTOTYPES ---
void kalman_filter_init_custom(kalman_filter_state_t *kf, float initial_altitude);
void kalman_filter_predict_custom(kalman_filter_state_t *kf, float measured_accel_input_mps2);
void kalman_filter_update_custom(kalman_filter_state_t *kf, float measured_altitude_m);

#endif /* KALMAN_FILTER_H_ */
