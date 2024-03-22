#pragma once

/// @brief Grouphead pressure control loop
namespace PressureControl {

    enum class PressureProfile {
        // Constant pressure, begin/end with lever
        Manual,

        // PID tuning
        Tuning,

        // Constant pressure, ends automatically when target extraction reached
        AutoConstant
    };

    /// @brief Initialize control loop parameters
    void initControlLoop();

    /// @brief Calculate next tick of the control loop
    void processControlLoop();

    void start();

    void stop();

    bool isProfileComplete();

    /// @brief Set the target profile
    void setProfile(PressureProfile profile);

    /// @brief Get the current profile
    PressureProfile getProfile();
}
