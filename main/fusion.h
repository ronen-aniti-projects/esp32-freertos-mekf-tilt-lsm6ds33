#pragma once 

#include <stdint.h> 
#include "imu.h"

typedef struct {
    float q_hat[4];            
    float b_hat[3];
    float P[6][6];
    float omega_hat_prev[3];
    int64_t last_t_us; 
} fusion_state_t;

void fusion_init(fusion_state_t *fusion_state);
void fusion_propagate(fusion_state_t *fusion_state, const imu_scaled_sample_t *scaled_sample);
void fusion_correct(fusion_state_t *fusion_state, const imu_scaled_sample_t *scaled_sample);




