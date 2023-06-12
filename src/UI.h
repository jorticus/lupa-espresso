#pragma once

#include "Display.h"
#include "UIWidgets.h"

#define TFT_RGB656(r,g,b)      ((((r & 0xFF) >> 3) << 11) | (((g & 0xFF) >> 2) << 5) | ((b & 0xFF) >> 3))
#define TFT_DARK_GOLDENROD  (TFT_RGB656(0xB8,0x86,0x0B))
#define TFT_ORANGERED       (TFT_RGB656(0xFF,0x45,0x00))
#define TFT_DARKESTGREY     0x38E7

/// @brief UI state and rendering
namespace UI {

    enum class Anim {
        PowerOn,
        PowerOff
    };

    /// @brief Render UI to the left & right canvas and update the display.
    void render();

    void uiFreezeGraphs();

    void triggerAnimation(Anim anim);
}
