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

}
