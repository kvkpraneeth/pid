#include "pid.h"
#include "stdio.h"
#include <unistd.h>

// Example Usage

#define kp 0.5
#define ki 0.01
#define kd 0.1

#define output_max 1
#define output_min -1

#define windup_limit 0.3
#define smoothing_factor 0.2

float state = 0;
float reference = 5;
float output;

struct pid plant;
pid_ pid;

int main()
{

    pid = make_pid_controller(&plant, &state, &output, &reference, 
        kp, ki, kd, output_max, output_min, windup_limit, smoothing_factor);

    while(fabsf(pid->reference - pid->output) > 0.1)
    {
        pid_compute(pid);
        *pid->state += *pid->output;
        printf("%f \n", *(pid->state));
        fflush(stdout);
        sleep(1);
    }

    return 0;

}