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

    /// @brief Set the target brew pressure (manual pressure profile)
    void setPressure(float sp);

    // void updateParameters(float p, float i, float d);

    // void getParameters(float* p, float* i, float* d);

    /// @brief Initialize control loop parameters
    void initControlLoop();

    /// @brief Calculate next tick of the control loop
    void processControlLoop();

    /// @brief Start pressure profiling (activate pump)
    void start();

    /// @brief Stop pressure profiling (deactivate pump)
    void stop();

    /// @brief Whether the profile is complete
    /// @return true if the profile has finished (shot has been completed)
    bool isProfileComplete();

    /// @brief Set the pressure profile to follow during the brew
    void setProfile(PressureProfile profile);

    /// @brief Get the current pressure profile
    PressureProfile getProfile();
}
