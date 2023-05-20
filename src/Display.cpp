#include <Arduino.h>
#include <TFT_eSPI.h>

#include "Display.h"
#include "hardware.h"

namespace Display {

TFT_eSPI    tft;
TFT_eSprite gfx_left  { &tft };
TFT_eSprite gfx_right { &tft };

void initDisplay() {
    Serial.println("Initialize LCD...");

    digitalWrite(TFT_CS_LEFT, LOW);
    tft.begin();
    tft.fillScreen(TFT_BLACK);
    digitalWrite(TFT_CS_LEFT, HIGH);

    digitalWrite(TFT_CS_RIGHT, LOW);
    tft.begin();
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
    digitalWrite(TFT_BL, HIGH);
}

SPIClass& getSPIInstance() {
    return tft.getSPIinstance();
}

}