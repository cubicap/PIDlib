#pragma once

#include <stdint.h>
#include <cmath>

class PID {
    private:
		float kp;
		float ki;
        float kd;
        double integration;
        int16_t lastError;
        int16_t minOutput;
        int16_t maxOutput;
        int16_t target;
        float period;
    public:
        PID(float, float, float, int16_t, int16_t, int32_t);
        PID(const float*);
        void setConstants(float, float, float);
        void setConstants(const float*);
        void setOutputLimits(int16_t, int16_t);
        void setTarget(uint16_t);
        void resetIntegrative();
        void setSamplePeriod(float, bool = true);
        int16_t calculate(int16_t);
        int16_t calculate(int16_t, int16_t);
        int16_t calculate(int16_t, int16_t, float);
};
