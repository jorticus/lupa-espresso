#pragma once

namespace SensorSampler {
    void Initialize();
    void Start();
    void Stop();
    void Process();

    float getTemperature();
    float getPressure();
    float getFlowRate();
    float getTotalFlowVolume();

    void resetFlowCounter();

    bool isTemperatureValid();
    bool isPressureValid();
    bool isFlowRateValid();
}