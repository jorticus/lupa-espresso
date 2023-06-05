#pragma once

/// @brief Boiler heater control loop
namespace HeatControl {

    enum class Mode {
        Off,
        Sleep,
        Brew,
        Steam
    };

    /// @brief Initialize control loop parameters
    void initControlLoop();

    /// @brief Calculate next tick of the control loop
    void processControlLoop();

    /// @brief Set the operating mode
    void setMode(Mode mode);

    /// @brief Get the operating mode
    Mode getMode();
}
