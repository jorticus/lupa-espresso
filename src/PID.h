#pragma once

#include <array>
#include <value_array.h>

class fPID {
public:
    fPID() : di_history(), setpoint(0.0f)
    {
        reset();
    }

    void reset();

    void setSampleTime(unsigned long time);
    void setParameters(float kp, float ki, float kd);
    void setWindupLimits(float min, float max);
    void setOutputLimits(float min, float max);

    float getKp();
    float getKd();
    float getKi();

    void setSetpoint(float setpoint);
    float getSetpoint();

    void setPerturbationOffset(float offset);

    float calculateTick(float input);

protected:
    unsigned long last_t;
    unsigned long last_t_kd;
    unsigned long sample_time;
    float kp, ki, kd;
    float setpoint;
    float offset;
    float last_input;
    float accum;
    float out_min, out_max;

    ValueArray<float, 10> di_history;
};
