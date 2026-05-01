#include "fusion.h"
#include "math_lib.h"

// Used to construct an initial Q matrix (Q0)
static const float ODR = 208.0;
static const float DT_SEC = 1.0 / ODR;
static const float GYRO_RATE_VARIANCE_X = 5.5886e-6 * DT_SEC;          // based on emperical stationary gx variance         
static const float GYRO_RATE_VARIANCE_Y = 4.9785e-6 * DT_SEC;          // based on emperical stationary gy variance
static const float GYRO_RATE_VARIANCE_Z = 1.7574e-6 * DT_SEC;          // based on emperical stationary gz variance
static const float GYRO_RATE_VARIANCE = 
    (GYRO_RATE_VARIANCE_X + GYRO_RATE_VARIANCE_Y + GYRO_RATE_VARIANCE_Z) / 3.0;  // Trawney MEKF assumes isotropic noise 
static const float GYRO_BIAS_WALK_VARIANCE = GYRO_RATE_VARIANCE / 2.0;           // guess - not based on emperical data

// Used to construct an initial P matrix (P0)
static const float P0_ANGULAR = 1e-6f;                        // Guess quaternion uncertainty [rad^2]
static const float P0_BIAS    = 1e-6f;                        // Guess bias uncertainty [rad^2/s^2]

// Used to construct an initial R matrix (R0)
static const float ACCEL_VARIANCE = 1e-2f;

// Constant matrix used in MARS lab MEKF paper
const float gc_matrix[6][6] = { {-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                {0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                {0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f},
                                {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
                                {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}};

// Continuous-time process noise covariance
const float qc_matrix[6][6] = { {GYRO_RATE_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 
                                {0.0f, GYRO_RATE_VARIANCE, 0.0f, 0.0f, 0.0f, 0.0f}, 
                                {0.0f, 0.0f, GYRO_RATE_VARIANCE, 0.0f, 0.0f, 0.0f}, 
                                {0.0f, 0.0f, 0.0f, GYRO_BIAS_WALK_VARIANCE, 0.0f, 0.0f}, 
                                {0.0f, 0.0f, 0.0f, 0.0f, GYRO_BIAS_WALK_VARIANCE, 0.0f}, 
                                {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, GYRO_BIAS_WALK_VARIANCE}};


// Accelerometer noise covariance matrix R
const float r_matrix[3][3] = { {ACCEL_VARIANCE, 0.0f, 0.0f},
                               {0.0f, ACCEL_VARIANCE, 0.0f},
                               {0.0f, 0.0f, ACCEL_VARIANCE}};

void fusion_init(fusion_state_t *fusion_state){

    // Initialize P as diagonal
    fusion_state->P[0][0] = P0_ANGULAR;
    fusion_state->P[1][1] = P0_ANGULAR;
    fusion_state->P[2][2] = P0_ANGULAR;
    fusion_state->P[3][3] = P0_BIAS;
    fusion_state->P[4][4] = P0_BIAS;
    fusion_state->P[5][5] = P0_BIAS;

    // Initialize q as no-rotation quaternion (JPL convention)
    // TODO: Implement logic to initialize filter around some ground-truth orientation
    fusion_state->q_hat[0] = 0.0f;
    fusion_state->q_hat[1] = 0.0f;
    fusion_state->q_hat[2] = 0.0f;
    fusion_state->q_hat[3] = 1.0f;

    // Initialize b_hat (rps) with bias determined empirically 
    fusion_state->b_hat[0] = 0.05883807f;
    fusion_state->b_hat[1] = -0.14938366f;
    fusion_state->b_hat[2] = -0.04615213f;

    // Initialize fusion timer
    fusion_state->last_t_us = 0;

    return;
}



void fusion_propagate(fusion_state_t *fusion_state, const imu_scaled_sample_t *scaled_sample){
    
    // Zero initialize all variables used in propagate calculations
    float omega_now[3] = {0.0f};
    float b_hat_neg[3] = {0.0f};
    float omega_hat_now[3] = {0.0f};
    float fc_matrix[6][6] = {{0.0f}};
    float cap_phi[6][6] = {{0.0f}};
    float q_11_matrix[3][3] = {{0.0f}};
    float q_12_matrix[3][3] = {{0.0f}};
    float q_22_matrix[3][3] = {{0.0f}};
    float qd_matrix[6][6] = {{0.0f}};
    float cap_phi_tranpose[6][6] = {{0.0f}};
    float intermediate_product_1[6][6] = {{0.0f}};
    float intermediate_product_2[6][6] = {{0.0f}};

    // Unpack the raw gyro rate vector
    omega_now[0] = scaled_sample->gx;
    omega_now[1] = scaled_sample->gy;
    omega_now[2] = scaled_sample->gz;
    
    // Subtract out the estimated rate bias from the raw gyro vector 
    calculate_vec3_scale(fusion_state->b_hat, -1.0f, b_hat_neg);
    calculate_vec3_sum(omega_now, b_hat_neg, omega_hat_now);

    // Special first iteration logic: We need two samples (first delta) to perform filter calculations
    if (fusion_state->last_t_us == 0){
        fusion_state->last_t_us = scaled_sample->t_us;
        for (int i = 0; i < 3; ++i){
            fusion_state->omega_hat_prev[i] = omega_hat_now[i];
        }
        return;
    }

    // Compute dt since last sample
    float dt = (float)(scaled_sample->t_us - fusion_state->last_t_us) * 1e-6f; 
    
    // Propagate nominal quaternion forward by dt seconds
    calculate_integrate_quaternion_forward(fusion_state->q_hat, dt, fusion_state->omega_hat_prev, omega_hat_now);

    // Compute Fc
    calculate_assemble_Fc_matrix(omega_hat_now, fc_matrix);

    // Compute cap_phi
    calculate_matrix_6x6_scale(fc_matrix, dt, fc_matrix);
    calculate_matrix_exponential6x6(fc_matrix, cap_phi);

    // compute Qd
    // Small omega_hat_now will cause numerical instability. Guard with: 
    float omega_norm = norm_3(omega_hat_now);
    if (omega_norm < 1e-2){
        calculate_Q11_matrix_small_omega(GYRO_RATE_VARIANCE, GYRO_BIAS_WALK_VARIANCE,
            dt, omega_hat_now, q_11_matrix);
        calculate_Q12_matrix_small_omega(GYRO_BIAS_WALK_VARIANCE, dt, omega_hat_now, q_12_matrix);
    } else {
        calculate_Q11_matrix(GYRO_RATE_VARIANCE, GYRO_BIAS_WALK_VARIANCE,
            dt, omega_hat_now, q_11_matrix);
        calculate_Q12_matrix(GYRO_BIAS_WALK_VARIANCE, dt, omega_hat_now, q_12_matrix);
    }
    calculate_Q22_matrix(GYRO_BIAS_WALK_VARIANCE, dt, omega_hat_now, q_22_matrix);    
    calculate_assemble_Qd_matrix(q_11_matrix, q_12_matrix, q_22_matrix, qd_matrix);

    // Update P
    calculate_transpose_6x6(cap_phi, cap_phi_tranpose);
    calculate_matrix_multiply_6x6_6x6(fusion_state->P, cap_phi_tranpose, intermediate_product_1);
    calculate_matrix_multiply_6x6_6x6(cap_phi, intermediate_product_1, intermediate_product_2);
    calculate_matrix_6x6_sum(intermediate_product_2, qd_matrix, fusion_state->P);

    // Record timestamp 
    fusion_state->last_t_us = scaled_sample->t_us;

    // Record bias-compensated gyro
    for (int i = 0; i < 3; ++i){
        fusion_state->omega_hat_prev[i] = omega_hat_now[i];
    }
    
}

void fusion_correct(fusion_state_t *fusion_state, const imu_scaled_sample_t *scaled_sample){
    // Define some local variables
    float H_matrix[3][6] = {{0.0f}};
    float rotation_q_est_matrix[3][3] = {{0.0f}};
    float rotated_accel[3] = {0.0f};
    float neg_rotated_accel[3] = {0.0f};
    float skew_rotated_accel_matrix[3][3] = {{0.0f}};
    float residual[3] = {0.0f};
    float H_transpose_matrix[6][3] = {{0.0f}};
    float P_H_transpose_matrix[6][3] = {{0.0f}};
    float H_P_H_transpose_matrix[3][3] = {{0.0f}};
    float S_matrix[3][3] = {{0.0f}};
    float S_inv_matrix[3][3] = {{0.0f}};
    float H_transpose_S_inv_matrix[6][3] = {{0.0f}};
    float K_matrix[6][3] = {{0.0f}}; 
    float delta_x[6] = {0.0f};
    float delta_q[3] = {0.0f};
    float delta_quat[4] = {0.0f};
    float delta_b[3] = {0.0f};
    float b_hat_neg[3] = {0.0f};
    float KH_matrix[6][6] ={{0.0f}};
    float IKH_matrix[6][6] = {{0.0f}};
    float IKH_transpose_matrix[6][6] = {{0.0f}};
    float K_transpose_matrix[3][6] = {{0.0f}};
    float RKT_matrix[3][6] = {{0.0f}};
    float KRKT_matrix[6][6] = {{0.0f}};
    float nominal_quat[4] = {0.0f};
    for (int i = 0; i < 4; ++i){
        nominal_quat[i] = fusion_state->q_hat[i];
    }
    float P_matrix[6][6] = {{0.0f}};
    for (int i = 0; i < 6; ++i){
        for (int j = 0 ; j < 6; ++j){
            P_matrix[i][j] = fusion_state->P[i][j];
        }
    }
    float q_updated[4] = {0.0f};
    float b_hat[3] = {0.0f};
    for (int i = 0; i < 3; ++i){
        b_hat[i] = fusion_state->b_hat[i];
    }
    float delta_b_neg[3] = {0.0f};
    float omega_now[3] = {0.0f};
    float omega_hat_updated[3] = {0.0f};

    float PIKHT_matrix[6][6] = {{0.0f}};
    float IKHPIKHT_matrix[6][6] = {{0.0f}};
    float P_updated_matrix[6][6] = {{0.0f}};
    float g_direction_nav[3] = {0.0, 0.0, 1.0};

    // Record latest accel sample and normalize it
    float accel_meas[3] = {scaled_sample->ax, scaled_sample->ay, scaled_sample->az};
    normalize_3(accel_meas);

    // Compute the rotation matrix global->local estimate for this timestep
    calculate_rotation_matrix_from_quaternion(fusion_state->q_hat, rotation_q_est_matrix);

    // Rotated expected gravity up according to the estimated rotation matrix
    calculate_matrix_3x3_vector_3x1_product(rotation_q_est_matrix, g_direction_nav, rotated_accel);

    // Compute the skew of the rotated measurement
    calculate_skew3_matrix(rotated_accel, skew_rotated_accel_matrix);

    // Assemble the H matrix
    assemble_H_matrix(skew_rotated_accel_matrix, H_matrix);

    // Compute the residual z-zhat
    calculate_vec3_scale(rotated_accel, -1.0f, neg_rotated_accel);
    calculate_vec3_sum(accel_meas, neg_rotated_accel, residual);
    
    // Compute the covariance of the residual S
    calculate_transpose_3x6(H_matrix, H_transpose_matrix);
    calculate_matrix_multiply_6x6_6x3(P_matrix, H_transpose_matrix, P_H_transpose_matrix);
    calculate_matrix_multiply_3x6_6x3(H_matrix, P_H_transpose_matrix, H_P_H_transpose_matrix);
    calculate_matrix_3x3_sum(H_P_H_transpose_matrix, r_matrix, S_matrix);

    // Compute the Kalman gain 
    calculate_invert_3x3_matrix(S_matrix, S_inv_matrix);
    calculate_matrix_multiply_6x3_3x3(H_transpose_matrix, S_inv_matrix, H_transpose_S_inv_matrix);
    calculate_matrix_multiply_6x6_6x3(P_matrix, H_transpose_S_inv_matrix, K_matrix);

    // Compute the state correction vector
    calculate_matrix_vector_product_6x3_3x1(K_matrix, residual, delta_x);

    // Compute the correction quaternion
    extract_del_q(delta_x, delta_q);
    calculate_del_q_to_quaternion(delta_q, delta_quat); 

    // Update the nominal quaternion 
    calculate_quaternion_multiply(delta_quat, nominal_quat, q_updated);
    
    // Update the nominal bias estimate
    delta_b[0] = delta_x[3];
    delta_b[1] = delta_x[4];
    delta_b[2] = delta_x[5];
    calculate_vec3_sum(b_hat, delta_b, b_hat);

    // Compensate the gyro meas with the updated bias
    omega_now[0] = scaled_sample->gx;
    omega_now[1] = scaled_sample->gy;
    omega_now[2] = scaled_sample->gz;
    calculate_vec3_scale(b_hat, -1.0f, b_hat_neg);
    calculate_vec3_sum(omega_now, b_hat_neg, omega_hat_updated);
       
    // Update the covariance
    calculate_matrix_multiply_6x3_3x6(K_matrix, H_matrix, KH_matrix);
    calculate_matrix_6x6_subtract(eye6, KH_matrix, IKH_matrix);
    calculate_transpose_6x6(IKH_matrix, IKH_transpose_matrix);
    calculate_transpose_6x3(K_matrix, K_transpose_matrix);
    calculate_matrix_multiply_3x3_3x6(r_matrix, K_transpose_matrix, RKT_matrix);
    calculate_matrix_multiply_6x3_3x6(K_matrix, RKT_matrix, KRKT_matrix);

    calculate_matrix_multiply_6x6_6x6(P_matrix, IKH_transpose_matrix, PIKHT_matrix);
    calculate_matrix_multiply_6x6_6x6(IKH_matrix, PIKHT_matrix, IKHPIKHT_matrix);
    calculate_matrix_6x6_sum(IKHPIKHT_matrix, KRKT_matrix, P_updated_matrix);

    // Record the bias compensated gyro (omega_hat) and b_hat
    for (int i = 0; i < 3; ++i){
        fusion_state->omega_hat_prev[i] = omega_hat_updated[i];
        fusion_state->b_hat[i] = b_hat[i];
    }

    // Record q_hat updated
    for (int i = 0; i < 4; ++i){
        fusion_state->q_hat[i] = q_updated[i];
    }

    // Record P updated 
    for (int i = 0; i < 6; ++i){
        for (int j = 0; j < 6; ++j){
            fusion_state->P[i][j] = P_updated_matrix[i][j];
        }
    }



}


