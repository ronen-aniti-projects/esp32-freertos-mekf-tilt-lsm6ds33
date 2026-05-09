#pragma once 

#include <stdint.h>
#include <stdbool.h>

extern const float eye4[4][4];
extern const float eye6[6][6];
extern const float eye3[3][3];

void calculate_vec3_sum(const float a[3], const float b[3], float out[3]);
void calculate_vec3_scale(const float in[3], const float scale, float out[3]);

void calculate_vec4_sum(const float a[4], const float b[4], float out [4]);
void normalize_4(float q[4]);
float norm_3(float w[3]);
float norm_4(float q[4]);

void calculate_transpose_3x3(const float input_matrix[3][3], float output_matrix[3][3]);
void calculate_transpose_6x6(const float input_matrix[6][6], float output_matrix[6][6]);

void calculate_matrix_6x6_scale(const float mat[6][6], const float scale, float result[6][6]);
void calculate_matrix_4x4_scale(const float mat[4][4], const float scale, float result[4][4]);
void calculate_matrix_3x3_scale(const float mat[3][3], const float scale, float result[3][3]);

void calculate_omega_bar(const float omega_prev[3], const float omega[3], float omega_bar[3]);
void calculate_cap_omega(const float omega[3], float cap_omega[4][4]);

void calculate_matrix_6x6_sum(const float mat1[6][6], const float mat2[6][6], float result[6][6]); 
void calculate_matrix_4x4_sum(const float mat1[4][4], const float mat2[4][4], float result[4][4]);
void calculate_matrix_3x3_sum(const float mat1[3][3], const float mat2[3][3], float result[3][3]);

void calculate_matrix_multiply_4x4_4x4(const float mat1[4][4], const float mat2[4][4], float result[4][4]);
void calculate_matrix_multiply_6x6_6x6(const float mat1[6][6], const float mat2[6][6], float result[6][6]);
void calculate_matrix_multiply_3x3_3x3(const float mat1[3][3], const float mat2[3][3], float result[3][3]);

void calculate_matrix_exponential4x4(const float mat[4][4], float result[4][4]);
void calculate_matrix_exponential6x6(const float mat[6][6], float result[6][6]);


void calculate_multiply_matrix_4x4_vec_4x1(const float mat[4][4], const float vec[4], float result[4]);
void calculate_integrate_quaternion_forward(float q[4], const float dt, const float omega_prev[3], const float omega_now[3]);

void calculate_assemble_Fc_matrix(const float omega_hat[3], float fc_matrix[6][6]);

void calculate_skew3_matrix(const float omega[3], float skew_matrix[3][3]);

void calculate_Q11_matrix(const float gyro_rate_variance, const float gyro_walk_variance,
                          const float dt, const float omega_hat[3], float q_11_matrix[3][3]);

void calculate_Q11_matrix_small_omega(const float gyro_rate_variance, const float gyro_walk_variance,
                          const float dt, const float omega_hat[3], float q_11_matrix[3][3]);

void calculate_Q12_matrix(const float gyro_walk_variance,
                          const float dt, const float omega_hat[3], float q_12_matrix[3][3]);

void calculate_Q12_matrix_small_omega(const float gyro_walk_variance,
                          const float dt, const float omega_hat[3], float q_12_matrix[3][3]);

void calculate_Q22_matrix(const float gyro_walk_variance,
                          const float dt, const float omega_hat[3], float q_22_matrix[3][3]);

void calculate_assemble_Qd_matrix(const float q_11_matrix[3][3], const float q_12_matrix[3][3], 
                         const float q_22_matrix[3][3], float qd_matrix[6][6]);


//TODO:
float calculate_sum_squares_3(const float qv[3]);
void calculate_matrix_multiply_3x3_3x6(const float matrix_in_1[3][3], const float matrix_in_2[3][6], float matrix_result[3][6]);
void calculate_matrix_multiply_6x3_3x6(const float matrix_in_1[6][3], const float matrix_in_2[3][6], float matrix_result[6][6]);
void calculate_matrix_multiply_6x6_6x3(const float matrix_in_1[6][6], const float matrix_in_2[6][3], float matrix_result[6][3]);
void calculate_matrix_multiply_6x3_3x3(const float matrix_in_1[6][3], const float matrix_in_2[3][3], float matrix_result[6][3]);
void calculate_matrix_multiply_3x6_6x3(const float matrix_in_1[3][6], const float matrix_in_2[6][3], float matrix_result[3][3]);
void calculate_matrix_vector_product_6x3_3x1(const float matrix_in[6][3], const float vector_in[3], float vector_result[6]);
bool calculate_invert_3x3_matrix(const float input_matrix[3][3], float output_matrix[3][3]);
void calculate_transpose_3x6(const float input_matrix_3x6[3][6], float output_matrix_6x3[6][3]); 
void calculate_transpose_6x3(const float input_matrix_6x3[6][3], float output_matrix_3x6[3][6]); 
void calculate_matrix_6x6_subtract(const float input_matrix_1[6][6], float input_matrix_2[6][6], float result_matrix[6][6]);
void calculate_skew4_matrix(const float q[4], float skew_matrix[4][4]);
void calculate_quaternion_outer_product(const float qv[3], float outer_product[3][3]);
void calculate_rotation_matrix_from_quaternion(const float q[4], float R[3][3]);
void extract_del_q(const float correction_vector[6], float del_q_vector_out[3]);
void calculate_del_q_to_quaternion(const float del_qv[3], float quaternion[4]);
void calculate_quaternion_multiply(const float dq[4], const float q[4], float result_q[4]);
void calculate_matrix_3x3_vector_3x1_product(const float input_matrix[3][3], const float input_vector[3], float result[3]);
void assemble_H_matrix(const float skew_rotated_accel_matrix[3][3], float H_matrix[3][6]);
void normalize_3(float accel[3]);


void calculate_make_symmetric_6x6(float matrix_6x6[6][6]);
void calculate_make_symmetric_3x3(float matrix_3x3[3][3]);