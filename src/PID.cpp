#include <PID.h>
#include <Arduino.h>

void fPID::reset() {
    this->offset = 0.0f;
    this->setpoint = 0.0f;
    this->last_input = 0.0f;
    this->accum = 0.0f;
    this->sample_time = 1000; // ms
    this->last_t = millis();
}

void fPID::setSampleTime(unsigned long time) {
    this->sample_time = time;

    float ratio = (float)time / (float)this->sample_time;
    if (ratio > 0.0f) {
        this->ki *= ratio;
        this->kd /= ratio;
    }

    // Invalidate any accumulated state
    reset();
}

void fPID::setParameters(float kp, float ki, float kd) {

    float sampleTimeSec = (float)sample_time * 0.001f;

    this->kp = kp;
    this->kd = kd * sampleTimeSec;
    this->ki = ki / sampleTimeSec;

}

void fPID::setWindupLimits(float min, float max) {

}

void fPID::setOutputLimits(float min, float max) {
    this->out_min = min;
    this->out_max = max;
}

float fPID::getKp() {
    return this->kp;
}
float fPID::getKd() {
    return this->kd;
}
float fPID::getKi() {
    return this->ki;
}

void fPID::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
}
float fPID::getSetpoint() {
    return this->setpoint;
}

void fPID::setPerturbationOffset(float offset) {
    this->offset = offset;
}

static inline void clamp(float &value, float min, float max) {
    if (value > max) value = max;
    else if (value < min) value = min;
}

float fPID::calculateTick(float input) {
    float error = (setpoint - input);
    float output = 0.0f;
    float out_d = 0.0f;
    float out_i = 0.0f;
    float out_p = 0.0f;

    // Calculate integral
    this->accum += (ki * error);
    clamp(accum, out_min, out_max);
    out_i = accum;

    // Calculate derivative
    //if (last_input > 0.0f) {
    if (di_history.is_full()) {
        // Get the sample from 10 seconds ago, from a circular buffer of samples.
        // This provides a more stable derivative calculation and avoids
        // spikes introduced by very small changes in input.
        float last_input = di_history.last();
        float di = (last_input - input);

        Serial.printf("Curr:%.1f First:%.1f Last:%.1f\n", input, di_history.first(), di_history.last());

        // Positive di means input is falling, and we need to add a positive offset to correct for it.
        out_d = kd * di;
    }

    // Store a history of last N samples for computing derivative
    auto t_now = millis();
    if ((t_now - last_t) >= 1000) {
        last_t = t_now;
        di_history.add(input);
    }

    // Add perturbation offset (affects Kp only)
    error += this->offset;

    // Calculate proportional
    out_p = error * kp;

    output = out_p + out_i + out_d;

    Serial.printf("PID: I:%.1f -> (P:%.1f,I:%.1f,D:%.1f,X:%.1f) -> O:%.1f\n", 
        input, out_p, out_i, out_d, offset, output);

    // Clamp output
    clamp(output, out_min, out_max);
    
    // this->last_input = input;
    // this->last_t = millis();

    return output;
}
