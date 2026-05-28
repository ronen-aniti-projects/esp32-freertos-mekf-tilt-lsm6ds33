#pragma once 

#include <stdint.h>

typedef struct{
    int64_t t_us;
    float q_hat[4];
    float b_hat[3];
    float P[6][6];
    float S[3][3];
    float r[3];
    float Qd[6][6];
} fusion_debug_t;