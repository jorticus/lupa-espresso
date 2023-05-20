#pragma once

/// @brief Boiler heater control loop
namespace HeatControl {

    /// @brief Initialize control loop parameters
    void initControlLoop();

    /// @brief Calculate next tick of the control loop
    void processControlLoop();

}
