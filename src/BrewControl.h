#pragma once

#include <string>

/// @brief Grouphead pressure control loop
namespace BrewControl {

    enum class BrewProfile {
        // Constant pressure, begin/end with lever
        ManualPressure,
        // Constant flow rate, begin/end with lever
        ManualFlow,

        // PID tuning
        TuningPressure,
        TuningFlow,

        // Dynamic brew profiles
        DynamicProfile,
    };

    /// @brief Set the target brew pressure (manual pressure profile)
    void setPressure(float sp);

    /// @brief Set the target flow rate (manual flow profile)
    void setFlowRate(float sp);

    /// @brief Initialize control loop parameters
    void initControlLoop();

    /// @brief Calculate next tick of the control loop
    void processControlLoop();

    /// @brief Start brew profiling (activate pump)
    void start();

    /// @brief Stop brew profiling (deactivate pump)
    void stop();

    /// @brief Whether the profile is complete
    /// @return true if the profile has finished (shot has been completed)
    bool isProfileComplete();

    /// @brief Set the brew profile to follow during the brew
    void setProfile(BrewProfile profile);

    /// @brief Get the current brew profile
    BrewProfile getProfile();

    /// @brief Get the current brew profile as a string, including dynamic profiles
    std::string getProfileString();

    /// @brief Return the average delta between the process target and actual value (pressure or flowrate),
    // to determine how closely the shot tracked the desired profile (lower = better)
    float getTargetError();
}
