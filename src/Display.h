#pragma once

#include <TFT_eSPI.h>

typedef TFT_eSprite GfxCanvas;

#pragma once

/// @brief Handles updating the TFT display, but not rendering of UI
namespace Display {
    
    extern GfxCanvas gfx_left;
    extern GfxCanvas gfx_right;

    /// @brief Initialize the TFT display
    void initDisplay();

    /// @brief Clear the canvas (does not update TFT)
    void tftClearCanvas();

    /// @brief Update TFT with contents of the left/right canvases
    void tftUpdateDisplay();

    /// @brief Get the SPIClass instance used by the TFT
    SPIClass& getSPIInstance();
}
