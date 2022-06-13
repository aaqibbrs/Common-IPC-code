#ifndef TUNING
#define TUNING

#include <ros/ros.h>

namespace T
{
    class tuner
    {
        private:
            float integral_prior, error_prior;
        public:
            float pid_tuner(float KP, float KI, float KD, double integral_end, float err);
            tuner();
    };
}

#endif