#pragma once

namespace State {
    
    typedef struct {
        unsigned long start_brew_time;
        unsigned long end_brew_time;

        float total_flow;
    } BrewStats;

    enum class MachineState {
        Off,
        Init,
        Preheat,
        Ready,
        Fault,
        Brewing,
        PostBrew,
        SensorTest,
        FirmwareUpdate,
        Sleep
    };


    enum class FaultState {
        NoFault,
        LowWater,
        OverTemp,
        SensorFailure,
        NotHeating,
        FirmwareUpdateFailure
    };

    /// @brief Current state of the UI, controlling what is shown
    extern MachineState uiState;

    /// @brief If uiState is Fault, this is the specific fault that occurred.
    extern FaultState uiFault;

    /// @brief Statistics of the current brew operation
    extern BrewStats brewStats;


    /// @brief Set the UI state, changing what is shown
    void setState(MachineState state);

    MachineState getState();

    /// @brief Set a fault state
    void setFault(FaultState fault);

    /// @brief Set power control state
    /// @param pwr On/off
    void setPowerControl(bool pwr);

    /// @brief Update internal UI state machine
    void processState();

}