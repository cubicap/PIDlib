#include "PIDlib.h"

PID::PID(float kp, float ki, float kd, int16_t minOutput, int16_t maxOutput, int32_t t):
    kp(kp),
    ki(ki),
    kd(kd),
    minOutput(minOutput),
    maxOutput(maxOutput),
    period(t)
{}

PID::PID(const float* constants):
    kp(constants[0]),
    ki(constants[1]),
    kd(constants[2]),
    minOutput(constants[3]),
    maxOutput(constants[4]),
    period(constants[5])
{}

void PID::setConstants(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
} 

void PID::setConstants(const float* constants) {
    kp = constants[0];
    ki = constants[1];
    kd = constants[2];
} 

void PID::setOutputLimits(int16_t min, int16_t max) {
    minOutput = min;
    maxOutput = max;
}

void PID::setTarget(uint16_t tgt) {
    target = tgt;
}

void PID::resetIntegrative() {
    integration = 0;
}

void PID::setSamplePeriod(float t, bool recalculate) {
    period = t;
}

int16_t PID::calculate(int16_t input) {
    return calculate(input, target, period);
}

int16_t PID::calculate(int16_t input, int16_t tgt) {
    return calculate(input, tgt, period);
}

int16_t PID::calculate(int16_t input, int16_t tgt, float period) {
    int16_t error = tgt - input;

    integration += (double)error * period;
    integration = (integration > minOutput / 4 ? (integration < maxOutput / 4 ? integration : maxOutput / 2) : minOutput / 2);

    double derivation = ((double)(error - lastError)) / period;

    int16_t Poutput = round(kp*(float)error);
    int16_t Ioutput = round(ki*integration);
    int16_t Doutput = round(kd*derivation);
  
    int16_t output = Poutput + Ioutput + Doutput;
    output = (output > minOutput ? (output < maxOutput ? output : maxOutput) : minOutput);
    
    lastError = error;

    return output;
}