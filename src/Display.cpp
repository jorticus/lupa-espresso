#include <Arduino.h>
#include <TFT_eSPI.h>

#include "Display.h"
#include "hardware.h"

namespace Display {

TFT_eSPI    tft;
TFT_eSprite gfx_left  { &tft };
TFT_eSprite gfx_right { &tft };

float display_brightness = 1.0;
bool backlight_en = false;

void initDisplay() {
    Serial.println("Initialize LCD...");

    digitalWrite(TFT_CS_LEFT, LOW);
    tft.begin();
    tft.setRotation(2);
    tft.fillScreen(TFT_BLACK);
    digitalWrite(TFT_CS_LEFT, HIGH);

    digitalWrite(TFT_CS_RIGHT, LOW);
    tft.begin();
    tft.setRotation(2);
    tft.fillScreen(TFT_BLACK);
    digitalWrite(TFT_CS_RIGHT, HIGH);

    // Allocate a buffer for the display
    gfx_left.setColorDepth(8);
    gfx_right.setColorDepth(8);
    if (gfx_left.createSprite(TFT_WIDTH, TFT_HEIGHT) == nullptr || 
        gfx_right.createSprite(TFT_WIDTH, TFT_HEIGHT) == nullptr)
    {
        Serial.println("ERROR: display buffer allocation failed!");
        return;
    }

    gfx_left.setTextSize(2);
    gfx_right.setTextSize(2);

    backlight_en = false;

    digitalWrite(TFT_BL, LOW);
    ledcAttachPin(TFT_BL, 0);
    ledcSetup(0, 4000, 8); // 12Khz 8bit resolution
}

void tftClearCanvas() {
    gfx_left.fillSprite(TFT_BLACK);
    gfx_right.fillSprite(TFT_BLACK);
}

void tftUpdateDisplay() {
    digitalWrite(MAX_CS, HIGH);

    digitalWrite(TFT_CS_RIGHT, LOW);
    gfx_right.pushSprite(0,0);
    digitalWrite(TFT_CS_RIGHT, HIGH);

    digitalWrite(TFT_CS_LEFT, LOW);
    gfx_left.pushSprite(0,0);
    digitalWrite(TFT_CS_LEFT, HIGH);

    // Enable backlight
    if (!backlight_en) {
        backlight_en = true;
        setBrightness(display_brightness);
    }
    
}

// def cie1931(L):
//     L = L*100.0
//     if L <= 8:
//         return (L/902.3)
//     else:
//         return ((L+16.0)/116.0)**3
static uint8_t cie1931_table[] = {
    0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 15, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 39, 40, 41, 42, 43, 43, 44, 45, 46, 47, 47, 48, 49, 50, 51, 52, 53, 54, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 70, 71, 72, 73, 74, 75, 76, 77, 79, 80, 81, 82, 83, 85, 86, 87, 88, 90, 91, 92, 94, 95, 96, 98, 99, 100, 102, 103, 105, 106, 108, 109, 110, 112, 113, 115, 116, 118, 120, 121, 123, 124, 126, 128, 129, 131, 132, 134, 136, 138, 139, 141, 143, 145, 146, 148, 150, 152, 154, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 185, 187, 189, 191, 193, 196, 198, 200, 202, 204, 207, 209, 211, 214, 216, 218, 220, 223, 225, 228, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252, 255
};

void setBrightness(float brightness) {
    uint8_t b = brightness * 0xFF;
    Serial.printf("Brightness: %d\n", b);
    
    if (b > 0) {
        // Max duty is 8 bits (0xFF)
        ledcWrite(0, cie1931_table[b]);
    }
    else {
        ledcWrite(0, 0);
        digitalWrite(TFT_BL, LOW);
    }

    display_brightness = brightness;
}

void turnOff() {
    tftClearCanvas();
    tftUpdateDisplay();

    ledcWrite(0, 0);
    digitalWrite(TFT_BL, LOW);
}

SPIClass& getSPIInstance() {
    return tft.getSPIinstance();
}

}