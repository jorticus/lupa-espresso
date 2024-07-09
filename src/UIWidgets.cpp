#include "UI.h"
#include "value_array.h"

namespace UI::Widgets {

void uiRenderLabelCentered(GfxCanvas& gfx, const char* s, int16_t y, uint16_t color) {
    uiRenderLabelCentered(gfx, y, color, s);
}
void uiRenderLabelCentered(GfxCanvas& gfx, int16_t y, uint16_t color, const char* s) {
    // Place below the status label
    int16_t tw = gfx.textWidth(s);
    int16_t th = 14; // TODO: Measure from font
    gfx.setCursor(TFT_WIDTH/2 - tw/2, TFT_HEIGHT/2 - th/2 + y);
    gfx.setTextColor(color);
    gfx.print(s);
}

void uiRenderLabelFormattedCentered(GfxCanvas& gfx, int16_t y, uint16_t color, const char* fmt, ...) {
    char buffer[40];
    
    va_list arg;
    va_start(arg, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, arg);
    if (len >= 0 && len <= sizeof(buffer)) {
        buffer[len] = '\0';
        uiRenderLabelCentered(gfx, y, color, buffer);
    }
    va_end(arg);
}

void uiRenderValueCentered(GfxCanvas& gfx, const char* fmt, const char* units, float value, bool is_valid, int16_t y, uint16_t color) {
    char buffer[20];
    if (is_valid) {
        snprintf(buffer, sizeof(buffer), "%.2f %s", value, units);
    } else {
        snprintf(buffer, sizeof(buffer), "- %s", units);
    }
    buffer[sizeof(buffer)-1] = '\0';

    uiRenderLabelCentered(gfx, buffer, y, color);
}


void uiRenderGauge(GfxCanvas& gfx, float value_norm, uint32_t fg_color, int32_t margin) {
    int32_t ring_w = 10;

    if (value_norm < 0.0f) value_norm = 0.0f;
    if (value_norm > 1.0f) value_norm = 1.0f;
    uint32_t min_angle = 90-30;
    uint32_t max_angle = 270+30;
    uint32_t angle = (value_norm * (max_angle - min_angle)) + min_angle;
    // if (angle > max_angle) angle = max_angle;
    // if (angle < min_angle) angle = min_angle;

    int32_t x = TFT_WIDTH/2;
    int32_t y = TFT_HEIGHT/2;
    int32_t r1 = TFT_WIDTH/2 - margin;
    int32_t r2 = r1 - ring_w;
    gfx.drawSmoothArc(x, y, r1, r2, min_angle, max_angle, TFT_DARKESTGREY, TFT_BLACK, true);
    if (angle > min_angle) {
        gfx.drawSmoothArc(x, y, r1, r2, min_angle, angle, fg_color, TFT_BLACK, true);
    }
}

void uiRenderImage(GfxCanvas& gfx, int32_t x, int32_t y, const lv_image_dsc_t& img, uint16_t color) {
    if (img.header.cf == LV_COLOR_FORMAT_A1 || img.header.cf == LV_COLOR_FORMAT_I1) {
        //gfx.drawBitmap(x, y, (const uint8_t*)img.data, img.header.w, img.header.h, color);
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

}
