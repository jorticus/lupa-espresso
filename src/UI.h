#pragma once

#include "Display.h"
#include "UIWidgets.h"

#define TFT_RGB656(r,g,b)      ((((r & 0xFF) >> 3) << 11) | (((g & 0xFF) >> 2) << 5) | ((b & 0xFF) >> 3))
#define TFT_DARK_GOLDENROD  (TFT_RGB656(0xB8,0x86,0x0B))
#define TFT_ORANGERED       (TFT_RGB656(0xFF,0x45,0x00))
#define TFT_DARKESTGREY     0x38E7

/// @brief UI state and rendering
namespace UI {

    typedef struct {
        unsigned long start_brew_time;
        unsigned long end_brew_time;

        float total_flow;
    } BrewStats;

    enum class UiState {
        Init,
        Preheat,
        Ready,
        Fault,
        Brewing,
        PostBrew,
        SensorTest,
        FirmwareUpdate,
        Sleep,
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
    extern UiState uiState;

    /// @brief If uiState is Fault, this is the specific fault that occurred.
    extern FaultState uiFault;

    /// @brief Statistics of the current brew operation
    extern BrewStats brewStats;


    /// @brief Set the UI state, changing what is shown
    void setState(UiState state);

    /// @brief Set a fault state
    void setFault(FaultState fault);

    /// @brief Update internal UI state machine
    void processState();

    /// @brief Render UI to the left & right canvas and update the display.
    void render();
}