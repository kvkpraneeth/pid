#include "pid.h"
#include "stdio.h"
#include "unistd.h"

// Example Usage

#define kp 0.5
#define ki 0.01
#define kd 0.1
#define kf 0.01 //Feedforward Gain

#define output_max 1
#define output_min -1

#define windup_limit 0.3
#define smoothing_factor 0.2

void SimpleWindupScheme(pid* plant, float* integral_sum)
{
    if((*integral_sum) > fabsf(windup_limit))
    {
        (*integral_sum) = fabsf(windup_limit);
    }

    if((*integral_sum) < -fabsf(windup_limit))
    {
        (*integral_sum) = -fabsf(windup_limit);
    }
}

void ComplementaryFilter(pid* plant, float* error)
{
    static float previous_filtered_error;

    (*error) = (1-smoothing_factor)*previous_filtered_error + 
        smoothing_factor * (*error);

    previous_filtered_error = (*error);
}

void SimpleOutputClamp(pid* plant)
{
    if(*(plant->output) > output_max)
    {
        *(plant->output) = output_max;
    }

    if(*(plant->output) < output_min)
    {
        *(plant->output) = output_min;
    }       
}

void SimpleFeedForward(pid* plant)
{
    (*plant->output) += (kf*(*plant->reference));
}

int main(int argc, char **argv)
{

    float state = 0;
    float reference = 5;
    float output;

    pid plant;
    pid* plant_t = &plant;

    make_pid_controller(plant_t, &state, &output, &reference, kp, ki, kd);

    while(fabsf(plant_t->reference - plant_t->output) > 0.1)
    {
        
        pid_compute(
            plant_t, 
            &SimpleWindupScheme, 
            &ComplementaryFilter, 
            &SimpleFeedForward,
            &SimpleOutputClamp
        );
        
        *plant_t->state += *plant_t->output;

        printf("%f \n", *(plant_t->state));
        fflush(stdout);
        sleep(1);
    }

    return 0;

}