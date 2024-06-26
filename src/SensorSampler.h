#pragma once

#include "value_array.h"

/// @brief Sensor sample collection and filtering
namespace SensorSampler {

    /// @brief Initialize sensors
    /// @return false if failed to initialize sensors
    bool initialize();

    /// @brief Begin sampling sensors
    void start();

    /// @brief Stop sampling sensors
    void stop();

    /// @brief Process incoming samples
    void process();

// Accessors

    /// @brief Current boiler temperature
    /// @return Temperature, in degC
    float getTemperature();

    /// @brief Current boiler temperature
    /// @return Temperature, in degC
    float getTemperature2();

    /// @brief Whether temperature is at target and has stabilized
    bool isTemperatureStabilized();

    /// @brief Estimated grouphead temperature,
    /// derived from the boiler temperature.
    /// @return Temperature, in degC
    float getEstimatedGroupheadTemperature();

    /// @brief Current grouphead pressure
    /// @return Pressure, in Bar
    float getPressure();

    /// @brief Current input flowrate
    /// @return Flowrate, in mL/s
    float getFlowRate();

    /// @brief Whether water is currently flowing
    bool isFlowing();

    /// @brief Accumulated flow volume
    /// @return Volume, in mL
    float getTotalFlowVolume();

    /// @brief Reset accumulated flow volume to 0
    void resetFlowCounter();

    /// @brief Temperature reading is valid
    /// @return true if valid, false if sensor error or out of range
    bool isTemperatureValid();

    /// @brief Pressure reading is valid
    /// @return true if valid, false if sensor error or out of range
    bool isPressureValid();

    /// @brief Flow rate reading is valid
    /// @return true if valid, false if sensor error or out of range
    bool isFlowRateValid();

    static const int numSamples = 300;
    
    /// @brief Temperature readings
    extern ValueArray<float, numSamples> temperatureSamples;

    /// @brief Temperature readings
    extern ValueArray<float, numSamples> temperatureSamples2;

    /// @brief Pressure readings
    extern ValueArray<float, numSamples> pressureSamples;

    /// @brief Flow rate readings
    extern ValueArray<float, numSamples> flowSamples;
}