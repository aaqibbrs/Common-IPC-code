#include "tuning_pid.h"

T::tuner::tuner():
            integral_prior(0), error_prior(0)
{

}

float T::tuner::pid_tuner(float KP, float KI, float KD, double integral_end, float err)
{
    double iteration_time;
    double integral_start = ros::Time::now().toSec();
    if(integral_end != -1.0)
    {
        iteration_time = integral_start - integral_end;
    }
    else
    {
        iteration_time = 0.0;
    }
    float integral = integral_prior + (err * iteration_time);
    float derivative = (err - error_prior)/iteration_time;
    float op = KP*err + KI*integral + KD*derivative;

    error_prior = err;
    integral_prior = integral;
    return op;
}