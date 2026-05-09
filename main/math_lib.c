#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "math_lib.h"

const float eye4[4][4] = { {1.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 1.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 1.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 1.0f}};

const float eye6[6][6] = { {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f} };

const float eye3[3][3] = { {1.0f, 0.0f, 0.0f},
                           {0.0f, 1.0f, 0.0f},
                           {0.0f, 0.0f, 1.0f}};

void calculate_vec3_sum(const float a[3], const float b[3], float out[3]){
    for (int i = 0; i < 3; ++i){
        out[i] = a[i] + b[i];
    }
}

void calculate_vec3_scale(const float in[3], const float scale, float out[3]){
    for (int i = 0; i < 3; ++i){
        out[i] = in[i] * scale;
    }
}

void calculate_vec4_sum(const float a[4], const float b[4], float out [4]){
    for (int i = 0; i < 4; ++i){
        out[i] = a[i] + b[i];
    }
}

void normalize_4(float q[4]){
    float sum_of_squares = 0.0f;
    for (int i = 0; i < 4; ++i){
        sum_of_squares += q[i] * q[i];
    }

    float norm = sqrtf(sum_of_squares);


    if (norm > 0.0f){
        for (int i = 0; i < 4; ++i){
            q[i] /= norm;
        }
    }

}

void normalize_3(float accel[3]){
    float sum_of_squares = 0.0f;
    for (int i = 0; i < 3; ++i){
        sum_of_squares += accel[i] * accel[i];
    }

    float norm = sqrtf(sum_of_squares);


    if (norm > 0.0f){
        for (int i = 0; i < 3; ++i){
            accel[i] /= norm;
        }
    }

}

float norm_3(float w[3]){
    return sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
}

float norm_4(float q[4]){
    return sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

void calculate_transpose_3x3(const float mat[3][3], float result[3][3]){
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            result[j][i] = mat[i][j];
        }
    }
}

void calculate_transpose_6x6(const float input_matrix[6][6], float output_matrix[6][6]){
    for (int i = 0; i < 6; ++i){
        for (int j = 0; j < 6; ++j){
            output_matrix[j][i] = input_matrix[i][j];
        }
    }
}

void calculate_matrix_6x6_scale(const float mat[6][6], const float scale, float result[6][6]){
    for (int i = 0; i < 6; ++i){
        for (int j = 0; j < 6; ++j){
            result[i][j] = mat[i][j] * scale;
        }
    }
}

void calculate_matrix_4x4_scale(const float mat[4][4], const float scale, float result[4][4]){
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            result[i][j] = mat[i][j] * scale;
        }
    }
}

void calculate_matrix_3x3_scale(const float mat[3][3], const float scale, float result[3][3]){
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            result[i][j] = mat[i][j] * scale;
        }
    } 
}

void calculate_omega_bar(const float omega_prev[3], const float omega[3], float omega_bar[3]){
    for (int i = 0; i < 3; ++i){
        omega_bar[i] = (omega_prev[i] + omega[i]) / 2.0f;
    }
}

void calculate_cap_omega(const float omega[3], float cap_omega[4][4]){
    float wx = omega[0];
    float wy = omega[1];
    float wz = omega[2];

    cap_omega[0][0] = 0.0f;
    cap_omega[0][1] = wz;
    cap_omega[0][2] = -wy;
    cap_omega[0][3] = wx;

    cap_omega[1][0] = -wz;
    cap_omega[1][1] = 0.0f;
    cap_omega[1][2] = wx;
    cap_omega[1][3] = wy;

    cap_omega[2][0] = wy;
    cap_omega[2][1] = -wx;
    cap_omega[2][2] = 0.0f;
    cap_omega[2][3] = wz;

    cap_omega[3][0] = -wx;
    cap_omega[3][1] = -wy;
    cap_omega[3][2] = -wz;
    cap_omega[3][3] = 0.0f;
}

void calculate_matrix_6x6_sum(const float mat1[6][6], const float mat2[6][6], float result[6][6]){
    for (int i = 0; i < 6; ++i){
        for (int j = 0; j < 6; ++j){
            result[i][j] = mat1[i][j] + mat2[i][j];
        }
    }
}

void calculate_matrix_4x4_sum(const float mat1[4][4], const float mat2[4][4], float result[4][4]){
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            result[i][j] = mat1[i][j] + mat2[i][j];
        }
    }
}

void calculate_matrix_3x3_sum(const float mat1[3][3], const float mat2[3][3], float result[3][3]){
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            result[i][j] = mat1[i][j] + mat2[i][j];
        }
    } 
}

void calculate_matrix_multiply_4x4_4x4(const float mat1[4][4], const float mat2[4][4], float result[4][4]){
    const int mat1_rows = 4;
    const int mat1_cols = 4;
    const int mat2_cols = 4;

    for (int i = 0; i < mat1_rows; ++i){
        for (int j = 0; j < mat2_cols; ++j){
            result[i][j] = 0.0f;
            for (int k = 0; k < mat1_cols; ++k){
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void calculate_matrix_multiply_6x6_6x6(const float mat1[6][6], const float mat2[6][6], float result[6][6]){
    const int mat1_rows = 6;
    const int mat1_cols = 6;
    const int mat2_cols = 6;

    for (int i = 0; i < mat1_rows; ++i){
        for (int j = 0; j < mat2_cols; ++j){
            result[i][j] = 0.0f;
            for (int k = 0; k < mat1_cols; ++k){
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void calculate_matrix_6x6_subtract(const float input_matrix_1[6][6], float input_matrix_2[6][6], float result_matrix[6][6]){
    for (int i = 0; i < 6; ++i){
        for (int j = 0; j < 6; ++j){
            result_matrix[i][j] = input_matrix_1[i][j] - input_matrix_2[i][j]; 
        }
    }
}

void calculate_matrix_multiply_3x3_3x3(const float mat1[3][3], const float mat2[3][3], float result[3][3]){
    const int mat1_rows = 3;
    const int mat1_cols = 3;
    const int mat2_cols = 3;

    for (int i = 0; i < mat1_rows; ++i){
        for (int j = 0; j < mat2_cols; ++j){
            result[i][j] = 0.0f;
            for (int k = 0; k < mat1_cols; ++k){
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    } 
}

void calculate_matrix_exponential4x4(const float mat[4][4], float result[4][4]){
    // Compute three-term truncated matrix exponential: I + A + 1/2 * A^2
    float term3[4][4] = {{0.0}};
    calculate_matrix_multiply_4x4_4x4(mat, mat, term3);
    calculate_matrix_4x4_scale(term3, 0.5f, term3);
    calculate_matrix_4x4_sum(eye4, mat, result);
    calculate_matrix_4x4_sum(result, term3, result);
}

void calculate_matrix_exponential6x6(const float mat[6][6], float result[6][6]){
    // Compute three-term truncated matrix exponential: I + A + 1/2 * A^2
    float term3[6][6] = {{0.0}};
    calculate_matrix_multiply_6x6_6x6(mat, mat, term3);
    calculate_matrix_6x6_scale(term3, 0.5f, term3);
    calculate_matrix_6x6_sum(eye6, mat, result);
    calculate_matrix_6x6_sum(result, term3, result);
}

void calculate_multiply_matrix_4x4_vec_4x1(const float mat[4][4], const float vec[4], float result[4]){
    for (int i = 0; i < 4; ++i){
        result[i] = 0.0f;
        for (int k = 0; k < 4; ++k){
            result[i] += mat[i][k] * vec[k];
        }
    }
}

void calculate_integrate_quaternion_forward(float q[4], const float dt, const float omega_prev[3], const float omega_now[3]){
    float omega_bar[3] = {0.0f};
    float cap_omega_t0[4][4] = {{0.0f}};
    float cap_omega_t1[4][4] = {{0.0f}};
    float cap_omega_bar[4][4] = {{0.0f}};
    float arg1[4][4] = {{0.0f}};
    float exp_arg[4][4] = {{0.0f}};
    float q1[4] = {0.0f};
    float arg2[4][4] = {{0.0f}};
    float arg2_scaled[4][4] = {{0.0f}};
    float q2[4] = {0.0f};
    float arg3[4][4] = {{0.0f}};
    float arg3_scaled[4][4] = {{0.0f}};
    float q3[4] = {0.0f};
    float q_sum_1[4] = {0.0f};
    float q_sum[4] = {0.0f};
    
    calculate_omega_bar(omega_prev, omega_now, omega_bar);
    calculate_cap_omega(omega_prev, cap_omega_t0);
    calculate_cap_omega(omega_now, cap_omega_t1);
    calculate_cap_omega(omega_bar, cap_omega_bar);

    // q1
    calculate_matrix_4x4_scale(cap_omega_bar, (dt * 0.5f), exp_arg);
    calculate_matrix_exponential4x4(exp_arg, arg1);
    calculate_multiply_matrix_4x4_vec_4x1(arg1, q, q1);

    // q2
    calculate_matrix_multiply_4x4_4x4(cap_omega_t1, cap_omega_t0, arg2);
    calculate_matrix_4x4_scale(arg2, (1.0f/48.0f * dt * dt), arg2_scaled);
    calculate_multiply_matrix_4x4_vec_4x1(arg2_scaled, q, q2);

    // q3
    calculate_matrix_multiply_4x4_4x4(cap_omega_t0, cap_omega_t1, arg3);
    calculate_matrix_4x4_scale(arg3, (-1.0f/48.0f * dt * dt), arg3_scaled);
    calculate_multiply_matrix_4x4_vec_4x1(arg3_scaled, q, q3);

    // sum and normalize
    calculate_vec4_sum(q1, q2, q_sum_1);
    calculate_vec4_sum(q_sum_1, q3, q_sum);
    normalize_4(q_sum);

    // update q
    for (int i = 0; i < 4; ++i){
        q[i] = q_sum[i];
    }

}

void calculate_assemble_Fc_matrix(const float omega_hat[3], float fc_matrix[6][6]){
    float wx = omega_hat[0];
    float wy = omega_hat[1];
    float wz = omega_hat[2];
    
    for (int i = 0; i < 6; ++i){ 
        for (int j = 0; j < 6; ++j){
            fc_matrix[i][j] = 0.0f;
        }
    }

    fc_matrix[0][0] = 0.0f;
    fc_matrix[0][1] = wz;
    fc_matrix[0][2] = -wy;
    fc_matrix[1][0] = -wz;
    fc_matrix[1][1] = 0.0f;
    fc_matrix[1][2] = wx;
    fc_matrix[2][0] = wy;
    fc_matrix[2][1] = -wx;
    fc_matrix[2][2] = 0.0f;
    fc_matrix[0][3] = -1.0f;
    fc_matrix[1][4] = -1.0f;
    fc_matrix[2][5] = -1.0f;


}

void calculate_skew3_matrix(const float omega[3], float skew_matrix[3][3]){
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            skew_matrix[i][j] = 0.0f;
        }
    }

    float wx = omega[0];
    float wy = omega[1];
    float wz = omega[2];
    
    skew_matrix[0][0] = 0.0f;
    skew_matrix[0][1] = -wz;
    skew_matrix[0][2] = wy;
    skew_matrix[1][0] = wz;
    skew_matrix[1][1] = 0.0f;
    skew_matrix[1][2] = -wx;
    skew_matrix[2][0] = -wy;
    skew_matrix[2][1] = wx;
    skew_matrix[2][2] = 0.0f;
    
}

void calculate_Q11_matrix(const float gyro_rate_variance, const float gyro_walk_variance,
                          const float dt, const float omega_hat[3], float q_11_matrix[3][3]){
    
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            q_11_matrix[i][j] = 0.0f;
        }
    }

    float term1[3][3] = {{0.0f}};
    float term2[3][3] = {{0.0f}};
    float omega_skew[3][3] = {{0.0f}};
    float omega_skew_squared[3][3] = {{0.0f}};
    float term3[3][3] = {{0.0f}};

    float wx = omega_hat[0];
    float wy = omega_hat[1];
    float wz = omega_hat[2];
    float w_norm = sqrtf(wx * wx + wy * wy + wz * wz);
    
    // term 1
    calculate_matrix_3x3_scale(eye3, (dt * gyro_rate_variance), term1);

    // term 2
    calculate_matrix_3x3_scale(eye3, (gyro_walk_variance * dt * dt * dt / 3.0f), term2);

    // term3
    float num = (powf(w_norm * dt, 3.0f) / 3.0f) + 
                 (2.0f * sinf(w_norm * dt)) - 
                 (2.0f * w_norm * dt);
    float denom = powf(w_norm, 5.0f);
    float factor = gyro_walk_variance * num / denom;
    calculate_skew3_matrix(omega_hat, omega_skew); 
    calculate_matrix_multiply_3x3_3x3(omega_skew, omega_skew, omega_skew_squared);
    calculate_matrix_3x3_scale(omega_skew_squared, factor, term3); 
    
    // sum 
    calculate_matrix_3x3_sum(term1, term2, term2);
    calculate_matrix_3x3_sum(term2, term3, q_11_matrix);
}

void calculate_Q11_matrix_small_omega(const float gyro_rate_variance, const float gyro_walk_variance,
                          const float dt, const float omega_hat[3], float q_11_matrix[3][3]){

    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            q_11_matrix[i][j] = 0.0f;
        }
    }
    
    float term1[3][3] = {{0.0f}};
    float term2[3][3] = {{0.0f}};
    float term3[3][3] = {{0.0f}};
    float omega_skew_squared[3][3] = {{0.0f}};
    float omega_skew[3][3] = {{0.0f}};

    float wx = omega_hat[0];
    float wy = omega_hat[1];
    float wz = omega_hat[2];
    float w_norm = sqrtf(wx * wx + wy * wy + wz * wz);

    // term 1
    calculate_matrix_3x3_scale(eye3, gyro_rate_variance * dt, term1);
    
    // term 2 
    calculate_matrix_3x3_scale(eye3, gyro_walk_variance * powf(dt, 3.0f) / 3.0f, term2);

    // term 3
    calculate_skew3_matrix(omega_hat, omega_skew);
    calculate_matrix_multiply_3x3_3x3(omega_skew, omega_skew, omega_skew_squared);
    calculate_matrix_3x3_scale(omega_skew_squared, gyro_walk_variance * 2.0f * powf(dt, 5.0f)/ 120.0f, term3);
    
    // sum
    calculate_matrix_3x3_sum(term1, term2, term2);
    calculate_matrix_3x3_sum(term2, term3, q_11_matrix);
}


void calculate_Q12_matrix(const float gyro_walk_variance,
                          const float dt, const float omega_hat[3], float q_12_matrix[3][3]){

    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            q_12_matrix[i][j] = 0.0f;
        }
    }

    float term1[3][3] = {{0.0f}};
    float term2[3][3] = {{0.0f}};
    float term3[3][3] = {{0.0f}};
    float omega_skew[3][3] = {{0.0f}};
    float omega_skew_squared[3][3] = {{0.0f}};

    float wx = omega_hat[0];
    float wy = omega_hat[1];
    float wz = omega_hat[2];

    float w_norm = sqrtf(wx * wx + wy * wy + wz * wz);

    calculate_skew3_matrix(omega_hat, omega_skew);
    calculate_matrix_multiply_3x3_3x3(omega_skew, omega_skew, omega_skew_squared);

    // term 1
    calculate_matrix_3x3_scale(eye3, dt * dt / 2.0f * -gyro_walk_variance, term1);

    // term 2
    float numerator_1 = w_norm * dt - sinf(w_norm * dt);
    float denominator_1 = powf(w_norm, 3.0f);
    float factor1 = gyro_walk_variance * numerator_1 / denominator_1;
    calculate_matrix_3x3_scale(omega_skew, factor1, term2);

    // term 3
    float numerator_2 = powf(w_norm * dt, 2.0f) / 2.0f + cosf(w_norm * dt) - 1.0f;
    float denominator_2 = powf(w_norm, 4.0f);
    float factor2 = -gyro_walk_variance * numerator_2 / denominator_2; 
    calculate_matrix_3x3_scale(omega_skew_squared, factor2, term3);

    // sum
    calculate_matrix_3x3_sum(term1, term2, term2);
    calculate_matrix_3x3_sum(term2, term3, q_12_matrix);
}

void calculate_Q12_matrix_small_omega(const float gyro_walk_variance,
                          const float dt, const float omega_hat[3], float q_12_matrix[3][3]){
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            q_12_matrix[i][j] = 0.0f;
        }
    }
    float omega_skew[3][3] = {{0.0f}};
    float omega_skew_squared[3][3] = {{0.0f}};
    float term1[3][3] = {{0.0f}};
    float term2[3][3] = {{0.0f}};
    float term3[3][3] = {{0.0f}};

    float wx = omega_hat[0];
    float wy = omega_hat[1];
    float wz = omega_hat[2];
    float w_norm = sqrtf(wx * wx + wy * wy + wz * wz);

    calculate_skew3_matrix(omega_hat, omega_skew);
    calculate_matrix_multiply_3x3_3x3(omega_skew, omega_skew, omega_skew_squared);

    // term 1
    calculate_matrix_3x3_scale(eye3, -gyro_walk_variance * dt * dt / 2.0f, term1);

    // term 2
    calculate_matrix_3x3_scale(omega_skew, gyro_walk_variance * powf(dt, 3.0f) / 6.0f, term2);

    // term3 
    calculate_matrix_3x3_scale(omega_skew_squared, powf(dt, 4.0f) / 24.0f * -gyro_walk_variance, term3);

    // sum
    calculate_matrix_3x3_sum(term1, term2, term2);
    calculate_matrix_3x3_sum(term2, term3, q_12_matrix);

}

void calculate_Q22_matrix(const float gyro_walk_variance,
                          const float dt, const float omega_hat[3], float q_22_matrix[3][3]){

        calculate_matrix_3x3_scale(eye3, dt * gyro_walk_variance, q_22_matrix);
}



void calculate_assemble_Qd_matrix(const float q_11_matrix[3][3], const float q_12_matrix[3][3], 
                         const float q_22_matrix[3][3], float qd_matrix[6][6]){
    
    for (int i = 0; i < 6; ++i){
        for (int j = 0; j < 6; ++j){
            qd_matrix[i][j] = 0.0f;
        }
    }

    float q_12_transpose[3][3] = {{0.0f}};
    calculate_transpose_3x3(q_12_matrix, q_12_transpose);

    qd_matrix[0][0] = q_11_matrix[0][0];
    qd_matrix[0][1] = q_11_matrix[0][1];
    qd_matrix[0][2] = q_11_matrix[0][2];
    qd_matrix[0][3] = q_12_matrix[0][0];
    qd_matrix[0][4] = q_12_matrix[0][1];
    qd_matrix[0][5] = q_12_matrix[0][2];

    qd_matrix[1][0] = q_11_matrix[1][0];
    qd_matrix[1][1] = q_11_matrix[1][1];
    qd_matrix[1][2] = q_11_matrix[1][2];
    qd_matrix[1][3] = q_12_matrix[1][0];
    qd_matrix[1][4] = q_12_matrix[1][1];
    qd_matrix[1][5] = q_12_matrix[1][2];

    qd_matrix[2][0] = q_11_matrix[2][0];
    qd_matrix[2][1] = q_11_matrix[2][1];
    qd_matrix[2][2] = q_11_matrix[2][2];
    qd_matrix[2][3] = q_12_matrix[2][0];
    qd_matrix[2][4] = q_12_matrix[2][1];
    qd_matrix[2][5] = q_12_matrix[2][2];

    qd_matrix[3][0] = q_12_transpose[0][0];
    qd_matrix[3][1] = q_12_transpose[0][1];
    qd_matrix[3][2] = q_12_transpose[0][2];
    qd_matrix[3][3] = q_22_matrix[0][0];
    qd_matrix[3][4] = q_22_matrix[0][1];
    qd_matrix[3][5] = q_22_matrix[0][2];

    qd_matrix[4][0] = q_12_transpose[1][0];
    qd_matrix[4][1] = q_12_transpose[1][1];
    qd_matrix[4][2] = q_12_transpose[1][2];
    qd_matrix[4][3] = q_22_matrix[1][0];
    qd_matrix[4][4] = q_22_matrix[1][1];
    qd_matrix[4][5] = q_22_matrix[1][2];

    qd_matrix[5][0] = q_12_transpose[2][0];
    qd_matrix[5][1] = q_12_transpose[2][1];
    qd_matrix[5][2] = q_12_transpose[2][2];
    qd_matrix[5][3] = q_22_matrix[2][0];
    qd_matrix[5][4] = q_22_matrix[2][1];
    qd_matrix[5][5] = q_22_matrix[2][2];
        
}


// Correction step
float calculate_sum_squares_3(const float qv[3]){
    return qv[0] * qv[0] + qv[1] * qv[1] + qv[2] * qv[2];
}

void calculate_matrix_multiply_3x3_3x6(const float matrix_in_1[3][3], const float matrix_in_2[3][6], float matrix_result[3][6]){
    const int mat1_rows = 3;
    const int mat1_cols = 3;
    const int mat2_cols = 6;

    for (int i = 0; i < mat1_rows; ++i){
        for (int j = 0; j < mat2_cols; ++j){
            matrix_result[i][j] = 0.0f;
            for (int k = 0; k < mat1_cols; ++k){
                matrix_result[i][j] += matrix_in_1[i][k] * matrix_in_2[k][j];
            }
        }
    }
}
void calculate_matrix_multiply_6x3_3x6(const float matrix_in_1[6][3], const float matrix_in_2[3][6], float matrix_result[6][6]){
    const int mat1_rows = 6;
    const int mat1_cols = 3;
    const int mat2_cols = 6;

    for (int i = 0; i < mat1_rows; ++i){
        for (int j = 0; j < mat2_cols; ++j){
            matrix_result[i][j] = 0.0f;
            for (int k = 0; k < mat1_cols; ++k){
                matrix_result[i][j] += matrix_in_1[i][k] * matrix_in_2[k][j];
            }
        }
    }
}

void calculate_matrix_multiply_3x6_6x3(const float matrix_in_1[3][6], const float matrix_in_2[6][3], float matrix_result[3][3]){
    const int mat1_rows = 3;
    const int mat1_cols = 6;
    const int mat2_cols = 3;

    for (int i = 0; i < mat1_rows; ++i){
        for (int j = 0; j < mat2_cols; ++j){
            matrix_result[i][j] = 0.0f;
            for (int k = 0; k < mat1_cols; ++k){
                matrix_result[i][j] += matrix_in_1[i][k] * matrix_in_2[k][j];
            }
        }
    }
}


void calculate_matrix_multiply_6x6_6x3(const float matrix_in_1[6][6], const float matrix_in_2[6][3], float matrix_result[6][3]){
    const int mat1_rows = 6;
    const int mat1_cols = 6;
    const int mat2_cols = 3;

    for (int i = 0; i < mat1_rows; ++i){
        for (int j = 0; j < mat2_cols; ++j){
            matrix_result[i][j] = 0.0f;
            for (int k = 0; k < mat1_cols; ++k){
                matrix_result[i][j] += matrix_in_1[i][k] * matrix_in_2[k][j];
            }
        }
    }
}
void calculate_matrix_multiply_6x3_3x3(const float matrix_in_1[6][3], const float matrix_in_2[3][3], float matrix_result[6][3]){
    const int mat1_rows = 6;
    const int mat1_cols = 3;
    const int mat2_cols = 3;

    for (int i = 0; i < mat1_rows; ++i){
        for (int j = 0; j < mat2_cols; ++j){
            matrix_result[i][j] = 0.0f;
            for (int k = 0; k < mat1_cols; ++k){
                matrix_result[i][j] += matrix_in_1[i][k] * matrix_in_2[k][j];
            }
        }
    }
}
void calculate_matrix_vector_product_6x3_3x1(const float matrix_in[6][3], const float vector_in[3], float vector_result[6]){
        for (int i = 0; i < 6; ++i){
        vector_result[i] = 0.0f;
        for (int k = 0; k < 3; ++k){
            vector_result[i] += matrix_in[i][k] * vector_in[k];
        }
    }
}

bool calculate_invert_3x3_matrix(const float input_matrix[3][3], float output_matrix[3][3]){
    // computes the inverse of a matrix input_matrix
    float det = input_matrix[0][0] * (input_matrix[1][1] * input_matrix[2][2] - input_matrix[2][1] * input_matrix[1][2]) -
                input_matrix[0][1] * (input_matrix[1][0] * input_matrix[2][2] - input_matrix[1][2] * input_matrix[2][0]) +
                input_matrix[0][2] * (input_matrix[1][0] * input_matrix[2][1] - input_matrix[1][1] * input_matrix[2][0]);

    if (fabs(det) < 1e-9f){
        return true;
    }

    float invdet = 1.0f / det;

    output_matrix[0][0] = (input_matrix[1][1] * input_matrix[2][2] - input_matrix[2][1] * input_matrix[1][2]) * invdet;
    output_matrix[0][1] = (input_matrix[0][2] * input_matrix[2][1] - input_matrix[0][1] * input_matrix[2][2]) * invdet;
    output_matrix[0][2] = (input_matrix[0][1] * input_matrix[1][2] - input_matrix[0][2] * input_matrix[1][1]) * invdet;
    output_matrix[1][0] = (input_matrix[1][2] * input_matrix[2][0] - input_matrix[1][0] * input_matrix[2][2]) * invdet;
    output_matrix[1][1] = (input_matrix[0][0] * input_matrix[2][2] - input_matrix[0][2] * input_matrix[2][0]) * invdet;
    output_matrix[1][2] = (input_matrix[1][0] * input_matrix[0][2] - input_matrix[0][0] * input_matrix[1][2]) * invdet;
    output_matrix[2][0] = (input_matrix[1][0] * input_matrix[2][1] - input_matrix[2][0] * input_matrix[1][1]) * invdet;
    output_matrix[2][1] = (input_matrix[2][0] * input_matrix[0][1] - input_matrix[0][0] * input_matrix[2][1]) * invdet;
    output_matrix[2][2] = (input_matrix[0][0] * input_matrix[1][1] - input_matrix[1][0] * input_matrix[0][1]) * invdet;
    return false;
}

void calculate_transpose_3x6(const float input_matrix_3x6[3][6], float output_matrix_6x3[6][3]){
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 6; ++j){
            output_matrix_6x3[j][i] = input_matrix_3x6[i][j];
        }
    }
}
void calculate_transpose_6x3(const float input_matrix_6x3[6][3], float output_matrix_3x6[3][6]){
    for (int i = 0; i < 6; ++i){
        for (int j = 0; j < 3; ++j){
            output_matrix_3x6[j][i] = input_matrix_6x3[i][j];
        }
    }
}

void calculate_skew4_matrix(const float q[4], float skew4_matrix[4][4]){
    float wx = q[0];
    float wy = q[1];
    float wz = q[2];
    
    skew4_matrix[0][0] = 0.0f;
    skew4_matrix[0][1] = wz;
    skew4_matrix[0][2] = -wy;
    skew4_matrix[0][3] = wx;

    skew4_matrix[1][0] = -wz;
    skew4_matrix[1][1] = 0.0f;
    skew4_matrix[1][2] = wx;
    skew4_matrix[1][3] = wy;

    skew4_matrix[2][0] = wy;
    skew4_matrix[2][1] = -wx;
    skew4_matrix[2][2] = 0.0f;
    skew4_matrix[2][3] = wz;

    skew4_matrix[3][0] = -wx;
    skew4_matrix[3][1] = -wy;
    skew4_matrix[3][2] = -wz;
    skew4_matrix[3][3] = 0.0f;
}

void calculate_quaternion_outer_product(const float qv[3], float outer_product[3][3]){
    outer_product[0][0] = qv[0] * qv[0];
    outer_product[0][1] = qv[0] * qv[1];
    outer_product[0][2] = qv[0] * qv[2];

    outer_product[1][0] = qv[1] * qv[0];
    outer_product[1][1] = qv[1] * qv[1];
    outer_product[1][2] = qv[1] * qv[2];

    outer_product[2][0] = qv[2] * qv[0];
    outer_product[2][1] = qv[2] * qv[1];
    outer_product[2][2] = qv[2] * qv[2];

}
void calculate_rotation_matrix_from_quaternion(const float q[4], float R[3][3]){
    float term_1[3][3] = {{0.0f}};
    float term_2[3][3] = {{0.0f}};
    float term_3[3][3] = {{0.0f}};
    float qv[3] = {q[0], q[1], q[2]};
    float skew3_matrix[3][3] = {{0.0f}};    
    float outer_prod_matrix[3][3] = {{0.0f}};
    
    // term 1
    calculate_matrix_3x3_scale(eye3, (2 * q[3] * q[3] - 1.0f), term_1);
    // term 2
    calculate_skew3_matrix(qv, skew3_matrix);
    calculate_matrix_3x3_scale(skew3_matrix, -2.0f * q[3], term_2);
    // term3
    calculate_quaternion_outer_product(qv, outer_prod_matrix);
    calculate_matrix_3x3_scale(outer_prod_matrix, 2.0f, term_3);

    // sum
    calculate_matrix_3x3_sum(term_1, term_2, term_2);
    calculate_matrix_3x3_sum(term_2, term_3, R);
}

void extract_del_q(const float correction_vector[6], float del_q_vector_out[3]){
    del_q_vector_out[0] = correction_vector[0] / 2.0f;
    del_q_vector_out[1] = correction_vector[1] / 2.0f;
    del_q_vector_out[2] = correction_vector[2] / 2.0f;
}

void calculate_del_q_to_quaternion(const float del_qv[3], float quaternion[4]){
    float qv_norm = norm_3(del_qv);
    float qv_norm_sq = qv_norm * qv_norm; 

    if (qv_norm_sq <= 1.0f){
        float angle_term = sqrtf(1.0f - qv_norm_sq);
        quaternion[0] = del_qv[0];
        quaternion[1] = del_qv[1];
        quaternion[2] = del_qv[2];
        quaternion[3] = angle_term;
    } else {
        float denom = sqrtf(1.0f + qv_norm_sq);
        quaternion[0] = del_qv[0] / denom;
        quaternion[1] = del_qv[1] / denom;
        quaternion[2] = del_qv[2] / denom;
        quaternion[3] = 1.0f/ denom;
    }

    normalize_4(quaternion);
}

void calculate_quaternion_multiply(const float dq[4], const float q[4], float result_q[4]){
    result_q[0] = dq[3] * q[0] + dq[2] * q[1] - dq[1] * q[2] + dq[0] * q[3];
    result_q[1] = -dq[2] * q[0] + dq[3] * q[1] + dq[0] * q[2] + dq[1] * q[3];
    result_q[2] = dq[1] * q[0] - dq[0] * q[1] + dq[3] * q[2] + dq[2] * q[3];
    result_q[3] = -dq[0] * q[0] - dq[1] * q[1] -dq[2] * q[2] + dq[3] * q[3];
    normalize_4(result_q);
}

void calculate_matrix_3x3_vector_3x1_product(const float input_matrix[3][3], const float input_vector[3], float result[3]){
    for (int i = 0; i < 3; ++i){
        result[i] = 0.0f;
        for (int k = 0; k < 3; ++k){
            result[i] += input_matrix[i][k] * input_vector[k];
        }
    }
}

void assemble_H_matrix(const float skew_rotated_accel_matrix[3][3], float H_matrix[3][6]){
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            H_matrix[i][j] = skew_rotated_accel_matrix[i][j];
        }
    }
}

void calculate_make_symmetric_6x6(float matrix_6x6[6][6]){
    
    float matrix_A[6][6] = {{0.0f}};
    float matrix_A_transpose[6][6] = {{0.0f}};

    
    for (int i = 0; i < 6; ++i){
        for (int j = 0; j < 6; ++j){
            matrix_A[i][j] = matrix_6x6[i][j];
            matrix_A_transpose[i][j] = matrix_6x6[j][i];
        }
    }

    for (int i = 0; i < 6; ++i){
        for (int j = 0; j < 6; ++j){
            matrix_6x6[i][j] = 0.5f * (matrix_A[i][j] + matrix_A_transpose[i][j]); 
        }
    }
}

void calculate_make_symmetric_3x3(float matrix_3x3[3][3]){
    
    float matrix_A[3][3] = {{0.0f}};
    float matrix_A_transpose[3][3] = {{0.0f}};

    
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            matrix_A[i][j] = matrix_3x3[i][j];
            matrix_A_transpose[i][j] = matrix_3x3[j][i];
        }
    }

    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 3; ++j){
            matrix_3x3[i][j] = 0.5f * (matrix_A[i][j] + matrix_A_transpose[i][j]); 
        }
    }
}