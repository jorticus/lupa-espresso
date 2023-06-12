#pragma once

#include "Display.h"
#include "value_array.h"

/// @brief Common UI code
namespace UI::Widgets {

/// @brief Render a horizontally-centered label
/// @param gfx Canvas
/// @param s String to print
/// @param y Vertical position
/// @param color Color of text
void uiRenderLabelCentered(GfxCanvas& gfx, int16_t y, uint16_t color, const char* s);

// For compatibility
void uiRenderLabelCentered(GfxCanvas& gfx, const char* s, int16_t y, uint16_t color = TFT_WHITE);

/// @brief Render a horizontally-centered label
/// @param gfx Canvas
/// @param y Vertical position
/// @param color Color of text
/// @param fmt Formatter
/// @param Args...
void uiRenderLabelFormattedCentered(GfxCanvas& gfx, int16_t y, uint16_t color, const char* fmt, ...);


/// @brief Render a circular gauge
/// @param gfx Canvas
/// @param value_norm Normalized value (0.0 to 1.0)
/// @param fg_color Foreground color
/// @param margin Offset from outer edge
void uiRenderGauge(GfxCanvas& gfx, float value_norm, uint32_t fg_color, int32_t margin = 0);


// TODO: Move to code-behind and use generic iterator that provides normalized samples (0.0-1.0)
/// @brief Render a simple graph. No axis/ticks/labels.
/// @tparam T Sample type
/// @tparam N Sample count
/// @param gfx Canvas
/// @param samples Samples array
/// @param min_value Y axis minimum
/// @param max_value Y axis maximum
/// @param color Color of the line
template <typename T, size_t N>
static void uiRenderGraph(GfxCanvas& gfx, ValueArray<T,N>& samples, float min_value, float max_value, uint16_t color = TFT_WHITE) {

    int32_t margin = 40;
    int32_t x = margin;
    int32_t y = TFT_HEIGHT / 2; // Vertically centered
    int32_t h = TFT_HEIGHT / 3;
    int32_t w = TFT_WIDTH - (margin * 2);

    // The sample array may be greater than the width.
    // Offset into the array so that the last sample drawn is the most recent available.
    int start = samples.size() - w;
    if (start < 0) start = 0;

    // Align the graph to the right side of the screen
    int offset = w - samples.size();
    if (offset < 0) offset = 0;
    x += offset;

    int32_t last_x = -1;
    int32_t last_y = -1;
    for (int i = start; i < samples.size(); i++) {
        auto sample = samples[i];

        if (sample < min_value) sample = min_value;
        else if (sample > max_value) sample = max_value;

        int32_t offset = (((sample - min_value) / (max_value - min_value)) * h * 2);
        int32_t yy = y + h - offset;
        int32_t d = last_y - yy;
        if ((last_y != -1) && (d > 1 || d < -1)) {
            gfx.drawLine(last_x, last_y, x, yy, color);
        }
        else {
            gfx.drawPixel(x, yy, color);
        }
        last_x = x;
        last_y = yy;
        x++;
    }
}

}