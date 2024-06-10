#pragma once

#include <array>
#include <value_array.h>
#include <math.h>

/// @brief Floating-point PID library
/// @details Implemented specifically for heater/pump control
class fPID {
public:
    fPID() : 
        di_history(), 
        sample_time(1000),
        kp(0), ki(0), kd(0),
        setpoint(0.0f), 
        p_offset(0.0f), static_offset(0.0f),
        last_input(0.0f),
        accum(0.0f),
        out_min(-infinityf()), out_max(infinityf()),
        range(infinityf()),
        en_integral(true),
        en_debug(false),
        is_out_negative(false)
    {
        reset();
    }

    /// @brief Reset the system including any integral windup or derivative averaging.
    void reset();

    /// @brief Set the sample update time. This MUST match the same rate that you call calculateTick().
    void setSampleTime(unsigned long time);

    /// @brief Enable/disable debug prints of internal state
    /// @param en 
    void setDebugPrints(bool en);

    /// @brief Apply new coefficients. System will be reset to initial values, including any integral windup.
    /// @param kp Proportional coefficient
    /// @param ki Integral coefficient
    /// @param kd Derivative coefficient
    void setParameters(float kp, float ki, float kd);

    // TODO
    void setWindupLimits(float min, float max);
    
    /// @brief Set the limits of the system output
    /// @param min Minimum allowable output value
    /// @param max Maximum allowable output value
    void setOutputLimits(float min, float max);

    void setRegulationRange(float range);

    /// @brief Enable/disable integral windup, effectively setting Ki=0. Enabled by default.
    void enableIntegral(bool en);

    float getKp();
    float getKd();
    float getKi();

    float getP();
    float getI();
    float getD();

    /// @brief Target setpoint for the control loop
    /// @details The PID loop will try to bring the system as close to this value as possible. It should be in the same units as the input reading (not the output).
    /// @param setpoint 
    void setSetpoint(float setpoint);
    float getSetpoint();

    /// @brief Apply a perturbation to the proportional term.
    /// @details This is useful when you know ahead of time that the system may experience a large offset, and applying this offset allows you to get the control loop ready for it.
    /// @param offset 
    void setPerturbationOffset(float offset);

    /// @brief Apply a constant offset to the output.
    /// @details This is useful when you need the integral term to get the plant to steady state, and when you know this term is constant.
    /// @param offset 
    void setPlantOffset(float offset);

    /// @brief Calculate the next step
    /// @param input System input (sensor reading)
    /// @return System output (to apply to plant)
    float calculateTick(float input);

protected:
    unsigned long last_t;
    unsigned long last_t_kd;
    unsigned long sample_time;
    float kp, ki, kd;
    float out_p, out_i, out_d;
    float setpoint;
    float p_offset, static_offset;
    float last_input;
    float accum;
    float out_min, out_max;
    float range;
    bool en_integral;
    bool en_debug;
    bool is_out_negative;

    // Sample history for computing the derivative term
    ValueArray<float, 10> di_history;
};
