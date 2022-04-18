#include "pid.h"

    pid_ make_pid_controller(pid_ plant,
        float* _state, float* _output, float* _reference, 
        float _kp, float _ki, float _kd, 
        float _output_max, float _output_min, float _windup_limit, 
        float _smoothing_factor
    ){
        plant->state = _state;
        plant->output = _output;
        plant->reference = _reference;

        plant->kp = _kp;
        plant->kd = _kd;
        plant->ki = _ki;

        plant->output_max = _output_max;
        plant->output_min = _output_min;
        plant->windup_limit = _windup_limit;
        plant->smoothing_factor = _smoothing_factor;

        plant->integral_sum = 0;
        plant->previous_error = 0;
        plant->previous_filtered_error = 0;

        return plant;
    }

    void pid_compute(pid_ plant)
    {

        float error = *(plant->reference)-*(plant->state);

        plant->integral_sum += plant->ki*error;

        if(plant->integral_sum > fabsf(plant->windup_limit))
        {
            plant->integral_sum = fabsf(plant->windup_limit);
        }

        if(plant->integral_sum < -fabsf(plant->windup_limit))
        {
            plant->integral_sum = -fabsf(plant->windup_limit);
        }

        float derivative_error = error - plant->previous_error;

        float filtered_derivative_error = 
            (1-plant->smoothing_factor)*plant->previous_filtered_error + 
            plant->smoothing_factor * derivative_error;

        *(plant->output) = plant->kp*error + plant->integral_sum + 
            plant->kd * filtered_derivative_error;

        if(*(plant->output) > plant->output_max)
        {
            *(plant->output) = plant->output_max;
        }

        if(*(plant->output) < plant->output_min)
        {
            *(plant->output) = plant->output_min;
        }

        plant->previous_error = error;
        plant->previous_filtered_error = filtered_derivative_error;

    }