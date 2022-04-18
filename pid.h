#ifndef __PID_H
#define __PID_H

#include "math.h"

struct pid
{

    // Plant Inputs and state
    float* state;
    float* output;
    float* reference;

    // Plant Gains
    float kp;
    float ki;
    float kd;

    // Plant Output Clamp
    float output_max;
    float output_min;

    // Plant Variables
    float integral_sum;
    float previous_error;
    float previous_filtered_error;

    // Plant Windup
    float windup_limit;

    // Plant LPF
    float smoothing_factor; // Exponential Moving Average Filter b/w [0,1]

};

typedef struct pid* pid_;

#ifdef	__cplusplus
extern "C" {
#endif

    pid_ make_pid_controller(pid_ plant,
        float* _state, float* _output, float* _reference, 
        float _kp, float _ki, float _kd, 
        float _output_max, float _output_min, float _windup_limit, 
        float _smoothing_factor
    );

    void pid_compute(pid_ plant);

#ifdef	__cplusplus
}
#endif

#endif