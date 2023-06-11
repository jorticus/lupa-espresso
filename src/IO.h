#pragma once

/// @brief Interface for the machine's hardware
namespace IO {

    /// @brief Initialize GPIO
    void initGpio();

    /// @brief Put the machine into a safe configuration (heaters off, etc)
    void failsafe();

    
// Inputs

    /// @brief Water tank low status indicator
    /// @return true if water tank is low
    bool isWaterTankLow();

    /// @brief Lever pull state
    /// @return true if lever is pulled
    bool isLeverPulled();

// Outputs

    /// @brief Set heater power duty cycle
    /// @param duty Power between 0.0 and 1.0
    void setHeatPower(float duty);

    /// @brief Set heater power state immediately
    /// @param en Heater power on/off
    void setHeat(bool en);

    /// @brief Set pump state immediately
    /// @param en Pump on/off
    void setPump(bool en);

    /// @brief Set water fill solenoid state immediately
    /// @param en Solenoid on/off
    void setWaterFillSolenoid(bool en);

// Output State

    /// @brief Current heater power output
    /// @return Power between 0.0 and 1.0
    float getHeatPower();

    /// @brief Current heater state (on/off)
    /// reflects instantaneous state of the heater
    bool isHeaterOn();
}