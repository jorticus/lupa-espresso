#pragma once

/// @brief Boiler heater control loop
namespace HeatControl {

    enum class BoilerProfile {
        Off,    // Heater off
        Idle,   // Idle temperature profile
        Brew,   // Brewing temperature profile
        Steam   // Steaming temperature profile
    };

    /// @brief Initialize control loop parameters
    void initControlLoop();

    /// @brief Calculate next tick of the control loop
    void processControlLoop();

    /// @brief Set the target boiler temperature profile
    void setProfile(BoilerProfile profile);

    /// @brief Get the current boiler temperature profile
    BoilerProfile getProfile();
}
