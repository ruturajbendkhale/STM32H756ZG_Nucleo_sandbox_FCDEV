#include "kalman_filter.h"
#include <string.h> // For memset
#include <math.h>   // For fabsf
#include <stdio.h>  // For sprintf, if used for debugging within these functions

/* USER CODE BEGIN Includes */
// You might need to include your main header if you need access to uart_buffer for debugging,
// or pass huart handle if you want direct UART prints from here.
// For now, assuming debug prints are minimal or handled outside.
/* USER CODE END Includes */

void kalman_filter_init_custom(kalman_filter_state_t *kf, float initial_altitude) {
    kf->t_sampl_s = KF_SAMPLING_TIME_S;
    float dt = kf->t_sampl_s;

    // Initialize matrix structures
    arm_mat_init_f32(&kf->x_bar, KF_STATE_DIM, 1, kf->x_bar_data);
    arm_mat_init_f32(&kf->x_hat, KF_STATE_DIM, 1, kf->x_hat_data);
    arm_mat_init_f32(&kf->Ad, KF_STATE_DIM, KF_STATE_DIM, kf->Ad_data);
    arm_mat_init_f32(&kf->Bd, KF_STATE_DIM, 1, kf->Bd_data);
    arm_mat_init_f32(&kf->P_bar, KF_STATE_DIM, KF_STATE_DIM, kf->P_bar_data);
    arm_mat_init_f32(&kf->P_hat, KF_STATE_DIM, KF_STATE_DIM, kf->P_hat_data);
    arm_mat_init_f32(&kf->Gd, KF_STATE_DIM, KF_PROCESS_NOISE_DIM, kf->Gd_data);
    arm_mat_init_f32(&kf->Q_source, KF_PROCESS_NOISE_DIM, KF_PROCESS_NOISE_DIM, kf->Q_source_data);
    arm_mat_init_f32(&kf->GdQGd_T, KF_STATE_DIM, KF_STATE_DIM, kf->GdQGd_T_data);
    arm_mat_init_f32(&kf->H, KF_MEASUREMENT_DIM, KF_STATE_DIM, kf->H_data);
    arm_mat_init_f32(&kf->K, KF_STATE_DIM, KF_MEASUREMENT_DIM, kf->K_data);
    arm_mat_init_f32(&kf->Ad_T, KF_STATE_DIM, KF_STATE_DIM, kf->Ad_T_data);
    arm_mat_init_f32(&kf->H_T, KF_STATE_DIM, KF_MEASUREMENT_DIM, kf->H_T_data);

    arm_mat_init_f32(&kf->temp_P_H_T, KF_STATE_DIM, KF_MEASUREMENT_DIM, kf->temp_P_H_T_data);
    arm_mat_init_f32(&kf->temp_H_P_H_T, KF_MEASUREMENT_DIM, KF_MEASUREMENT_DIM, kf->temp_H_P_H_T_data);
    arm_mat_init_f32(&kf->temp_K_H, KF_STATE_DIM, KF_STATE_DIM, kf->temp_K_H_data);
    arm_mat_init_f32(&kf->I_temp, KF_STATE_DIM, KF_STATE_DIM, kf->I_temp_data);

    // State Transition Matrix Ad = [[1, dt, dt*dt/2], [0, 1, dt], [0, 0, 1]]
    kf->Ad_data[0] = 1.0f; kf->Ad_data[1] = dt;   kf->Ad_data[2] = dt * dt / 2.0f;
    kf->Ad_data[3] = 0.0f; kf->Ad_data[4] = 1.0f; kf->Ad_data[5] = dt;
    kf->Ad_data[6] = 0.0f; kf->Ad_data[7] = 0.0f; kf->Ad_data[8] = 1.0f;
    arm_mat_trans_f32(&kf->Ad, &kf->Ad_T);

    // Control Input Matrix Bd = [[dt*dt/2], [dt], [0]]
    kf->Bd_data[0] = dt * dt / 2.0f;
    kf->Bd_data[1] = dt;
    kf->Bd_data[2] = 0.0f;

    // Process Noise Configuration
    kf->Gd_data[0] = dt;                 kf->Gd_data[1] = dt * dt / 2.0f;
    kf->Gd_data[2] = 1.0f;               kf->Gd_data[3] = dt;
    kf->Gd_data[4] = 0.0f;               kf->Gd_data[5] = 1.0f;

    float32_t q_accel_var = KF_STD_DEV_ACCEL_INPUT_NOISE * KF_STD_DEV_ACCEL_INPUT_NOISE;
    float32_t q_bias_var = KF_STD_DEV_BIAS_WALK_NOISE * KF_STD_DEV_BIAS_WALK_NOISE;
    kf->Q_source_data[0] = q_accel_var; kf->Q_source_data[1] = 0.0f;
    kf->Q_source_data[2] = 0.0f;        kf->Q_source_data[3] = q_bias_var;

    float32_t temp_Gd_Q_data[KF_STATE_DIM * KF_PROCESS_NOISE_DIM];
    arm_matrix_instance_f32 temp_Gd_Q;
    arm_mat_init_f32(&temp_Gd_Q, KF_STATE_DIM, KF_PROCESS_NOISE_DIM, temp_Gd_Q_data);
    arm_mat_mult_f32(&kf->Gd, &kf->Q_source, &temp_Gd_Q);

    float32_t Gd_T_data[KF_PROCESS_NOISE_DIM * KF_STATE_DIM];
    arm_matrix_instance_f32 Gd_T;
    arm_mat_init_f32(&Gd_T, KF_PROCESS_NOISE_DIM, KF_STATE_DIM, Gd_T_data);
    arm_mat_trans_f32(&kf->Gd, &Gd_T);
    arm_mat_mult_f32(&temp_Gd_Q, &Gd_T, &kf->GdQGd_T);

    // Measurement Matrix H = [1, 0, 0]
    kf->H_data[0] = 1.0f; kf->H_data[1] = 0.0f; kf->H_data[2] = 0.0f;
    arm_mat_trans_f32(&kf->H, &kf->H_T);

    // Measurement Noise Covariance R
    kf->R_val = KF_STD_DEV_BARO_MEAS_NOISE * KF_STD_DEV_BARO_MEAS_NOISE;

    // Initial State Estimate x_bar
    kf->x_bar_data[0] = initial_altitude;
    kf->x_bar_data[1] = 0.0f;
    kf->x_bar_data[2] = 0.0f;

    // Initial Covariance P_bar
    memset(kf->P_bar_data, 0, sizeof(kf->P_bar_data));
    kf->P_bar_data[0] = KF_INITIAL_UNCERTAINTY_ALTITUDE * KF_INITIAL_UNCERTAINTY_ALTITUDE;
    kf->P_bar_data[KF_STATE_DIM + 1] = KF_INITIAL_UNCERTAINTY_VELOCITY * KF_INITIAL_UNCERTAINTY_VELOCITY;
    kf->P_bar_data[2*KF_STATE_DIM + 2] = KF_INITIAL_UNCERTAINTY_ACCEL_BIAS * KF_INITIAL_UNCERTAINTY_ACCEL_BIAS;
    
    // Identity matrix for P_bar update: P_bar = (I - K*H) * P_hat
    memset(kf->I_temp_data, 0, sizeof(kf->I_temp_data));
    kf->I_temp_data[0] = 1.0f;
    kf->I_temp_data[KF_STATE_DIM + 1] = 1.0f;
    kf->I_temp_data[2*KF_STATE_DIM + 2] = 1.0f;

    // Optional: Print a confirmation that KF is initialized, if UART is accessible
    // char debug_buffer[100];
    // sprintf(debug_buffer, "KF Init: Alt=%.2f, dt=%.4f\r\n", kf->x_bar_data[0], dt);
    // HAL_UART_Transmit(huart_for_debug, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
}

void kalman_filter_predict_custom(kalman_filter_state_t *kf, float measured_accel_input_mps2) {
    // Predict state: x_hat = Ad * x_bar + Bd * u
    float32_t Bd_u_data[KF_STATE_DIM];
    arm_matrix_instance_f32 Bd_u;
    arm_mat_init_f32(&Bd_u, KF_STATE_DIM, 1, Bd_u_data);
    arm_mat_scale_f32(&kf->Bd, measured_accel_input_mps2, &Bd_u);

    float32_t Ad_x_bar_data[KF_STATE_DIM];
    arm_matrix_instance_f32 Ad_x_bar;
    arm_mat_init_f32(&Ad_x_bar, KF_STATE_DIM, 1, Ad_x_bar_data);
    arm_mat_mult_f32(&kf->Ad, &kf->x_bar, &Ad_x_bar);

    arm_mat_add_f32(&Ad_x_bar, &Bd_u, &kf->x_hat);

    // Predict covariance: P_hat = Ad * P_bar * Ad_T + GdQGd_T
    float32_t Ad_P_bar_data[KF_STATE_DIM * KF_STATE_DIM];
    arm_matrix_instance_f32 Ad_P_bar;
    arm_mat_init_f32(&Ad_P_bar, KF_STATE_DIM, KF_STATE_DIM, Ad_P_bar_data);
    arm_mat_mult_f32(&kf->Ad, &kf->P_bar, &Ad_P_bar);

    float32_t Ad_P_bar_Ad_T_data[KF_STATE_DIM * KF_STATE_DIM];
    arm_matrix_instance_f32 Ad_P_bar_Ad_T;
    arm_mat_init_f32(&Ad_P_bar_Ad_T, KF_STATE_DIM, KF_STATE_DIM, Ad_P_bar_Ad_T_data);
    arm_mat_mult_f32(&Ad_P_bar, &kf->Ad_T, &Ad_P_bar_Ad_T);

    arm_mat_add_f32(&Ad_P_bar_Ad_T, &kf->GdQGd_T, &kf->P_hat);
}

void kalman_filter_update_custom(kalman_filter_state_t *kf, float measured_altitude_m) {
    // Kalman Gain K = P_hat * H_T * inv(H * P_hat * H_T + R)
    arm_mat_mult_f32(&kf->P_hat, &kf->H_T, &kf->temp_P_H_T);
    arm_mat_mult_f32(&kf->H, &kf->temp_P_H_T, &kf->temp_H_P_H_T);

    float32_t innovation_cov = kf->temp_H_P_H_T_data[0] + kf->R_val;
    float32_t inv_innovation_cov = 0.0f;
    if (fabsf(innovation_cov) > 1e-9f) {
        inv_innovation_cov = 1.0f / innovation_cov;
    }
    arm_mat_scale_f32(&kf->temp_P_H_T, inv_innovation_cov, &kf->K);

    // Update state estimate: x_bar = x_hat + K * (y - H * x_hat)
    float32_t H_x_hat_data[KF_MEASUREMENT_DIM];
    arm_matrix_instance_f32 H_x_hat;
    arm_mat_init_f32(&H_x_hat, KF_MEASUREMENT_DIM, 1, H_x_hat_data);
    arm_mat_mult_f32(&kf->H, &kf->x_hat, &H_x_hat);

    float32_t measurement_residual = measured_altitude_m - H_x_hat_data[0];

    float32_t K_residual_data[KF_STATE_DIM];
    arm_matrix_instance_f32 K_residual;
    arm_mat_init_f32(&K_residual, KF_STATE_DIM, 1, K_residual_data);
    arm_mat_scale_f32(&kf->K, measurement_residual, &K_residual);

    arm_mat_add_f32(&kf->x_hat, &K_residual, &kf->x_bar);

    // Update covariance estimate: P_bar = (I - K * H) * P_hat
    arm_mat_mult_f32(&kf->K, &kf->H, &kf->temp_K_H);

    float32_t I_minus_KH_data[KF_STATE_DIM*KF_STATE_DIM];
    arm_matrix_instance_f32 I_minus_KH;
    arm_mat_init_f32(&I_minus_KH, KF_STATE_DIM, KF_STATE_DIM, I_minus_KH_data);
    arm_mat_sub_f32(&kf->I_temp, &kf->temp_K_H, &I_minus_KH);

    arm_mat_mult_f32(&I_minus_KH, &kf->P_hat, &kf->P_bar);

    // The estimated states are now in kf->x_bar_data[0] (altitude),
    // kf->x_bar_data[1] (velocity), and kf->x_bar_data[2] (accel_bias).
    // Main.c will read these directly from the kf_vertical_state instance.
}
