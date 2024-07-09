#pragma once

#include "Display.h"
#include "lvgl/lvgl.h"
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
static void uiRenderGraph(GfxCanvas& gfx, ValueArray<T,N>& samples, float min_value, float max_value, uint16_t color = TFT_WHITE, float ref_value = INFINITY) {

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

    // Dashed horizontal line to mark target value
    w = TFT_WIDTH - margin;
    if (ref_value >= min_value && ref_value <= max_value) {
        int32_t offset = (((ref_value - min_value) / (max_value - min_value)) * h * 2);
        int32_t yy = y + h - offset;
        int32_t x = margin;
        while (x < w) {
            //gfx.drawFastHLine(x, yy, 8, TFT_DARKGREY);
            gfx.drawPixel(x, yy, TFT_DARKGREY);
            x += 3;
        }
    }
}



static void uiRenderImage(GfxCanvas& gfx, int32_t x, int32_t y, const lv_image_dsc_t& img, uint16_t color = TFT_WHITE) {

    //gfx.drawBitmap(x, y, (const uint8_t*)img.data, w, h, color, TFT_YELLOW);
    //gfx.pushImage(x, y, img.header.w, img.header.h, (uint16_t*)img.data);

    if (img.header.cf == LV_COLOR_FORMAT_A1 || img.header.cf == LV_COLOR_FORMAT_I1) {
        const uint8_t* bitmap = img.data;
        int32_t dx = 0;
        int32_t dy = 0;
        for (int i = 0; i < img.data_size; i++) {
            uint8_t b = *bitmap++;
            for (int j = 7; j >= 0; j--, b >>= 1) {
                if (b & 1) {
                    gfx.drawPixel(x + dx + j, y + dy, color);
                }
            }
            dx += 8;
            if (dx >= img.header.w) {
                dx = 0;
                dy++;
            }
        }
    } 
    else if (img.header.cf == LV_COLOR_FORMAT_RGB565 || img.header.cf == LV_COLOR_FORMAT_BGR565) {
        gfx.setSwapBytes(true);
        gfx.pushImage(x, y, img.header.w, img.header.h, (uint16_t*)img.data);
    }
    else {
        // Unsupported image type, show red square
        gfx.fillRect(x, y, img.header.w, img.header.h, TFT_RED);
    }
}

static void uiRenderImageCentered(GfxCanvas& gfx, int32_t x, int32_t y, const lv_image_dsc_t& img, uint16_t color = TFT_WHITE) {
    int32_t w = img.header.w;
    int32_t h = img.header.h;
    x -= w / 2;
    h -= h / 2;
    uiRenderImage(gfx, x, y, img, color);
}

// static void uiRenderImage(GfxCanvas& gfx, int32_t x, int32_t y, const lv_image_dsc_t& img, uint16_t color = TFT_WHITE) {
//     lv_img_dsc_t _img;
//     _img.data = img.data;
//     _img.data_size = img.data_size;
//     _img.header.cf = img.header.cf;
//     _img.header.w = img.header.w;
//     _img.header.h = img.header.h;
//     uiRenderImage(gfx, x, y, _img, color);
// }

}