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

};

typedef struct pid pid;

#ifdef	__cplusplus
extern "C" {
#endif

    void make_pid_controller(pid* plant,
        float* _state, float* _output, float* _reference, 
        float _kp, float _ki, float _kd);

    void pid_compute(pid* plant, 
        void (*AntiWindup)(pid*, float*), 
        void (*LowPassFilter)(pid*, float*),
        float (*FeedForward)(pid*),
        void (*PlantLimits)(pid*)
    );

#ifdef	__cplusplus
}
#endif

#endif