#include "pid.h"

void make_pid_controller(pid* plant,
    float* _state, float* _output, float* _reference, 
    float _kp, float _ki, float _kd
){
    plant->state = _state;
    plant->output = _output;
    plant->reference = _reference;

    plant->kp = _kp;
    plant->kd = _kd;
    plant->ki = _ki;
}

void pid_compute(pid* plant, 
    void (*AntiWindup)(pid*, float*), 
    void (*LowPassFilter)(pid*, float*),
    void (*PlantLimits)(pid*)
)
{

    static float integral_sum;
    static float previous_error;

    float error = *(plant->reference)-*(plant->state);

    integral_sum += plant->ki*error;

    (*AntiWindup)(plant, &integral_sum);

    float derivative_error = error - previous_error;

    (*LowPassFilter)(plant, &derivative_error);

    *(plant->output) = plant->kp*error + integral_sum + 
        plant->kd * derivative_error;

    (*PlantLimits)(plant);

    previous_error = error;

}