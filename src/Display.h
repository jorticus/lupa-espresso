#pragma once

#include <TFT_eSPI.h>

#define ENABLE_DISPLAY
// #define DUAL_BUFFERS

typedef TFT_eSprite GfxCanvas;

#pragma once

/// @brief Handles updating the TFT display, but not rendering of UI
namespace Display {

    enum class ActiveBuffer {
        Left, Right, Both
    };

    #ifdef DUAL_BUFFERS
    extern GfxCanvas gfx_left;
    extern GfxCanvas gfx_right;
    #else
    extern GfxCanvas gfx;
    // extern ActiveBuffer gfxActiveBuffer;
    #endif

    /// @brief Initialize the TFT display
    bool initDisplay();

    /// @brief Clear the canvas (does not update TFT)
    void tftClearCanvas();

    /// @brief Update TFT with contents of the left/right canvases
    void tftUpdateDisplay(ActiveBuffer activeBuffer = ActiveBuffer::Both);

    /// @brief Set the display backlight brightness
    /// @param brightness 0.0 to 1.0
    void setBrightness(float brightness);

    void turnOff();

    /// @brief Get the SPIClass instance used by the TFT
    SPIClass& getSPIInstance();
}
