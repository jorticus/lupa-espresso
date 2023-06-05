#include <array>
#include <Arduino.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_GC9A01A.h>
//#include <Arduino_GFX.h>
#include <Wire.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_MAX31865.h>
#include <WiFi.h>
#include "secrets.h"
#include "PressureTransducer.h"
#include "PulseCounter.h"
#include <TFT_eSPI.h>
#include "config.h"
#include "hardware.h"
#include "value_array.h"
#include "SensorSampler.h"
#include "Machine.h"
#include "HeatControl.h"
#include "Display.h"
#include "UI.h"
#include "Network.h"
#include "OTA.h"
#include "HomeAssistant.h"

Stream& Debug = Serial;

/// @brief Network name of the device
const char* DEVICE_NAME = "LUPA";

Adafruit_MAX31865   rtd(MAX_CS, &Display::getSPIInstance());
PressureTransducer  pressure(PRESSURE_FULL_SCALE);

using namespace Display;

void setup() {
    Serial.begin(9600);

    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();

    // The following devices share the same SPI bus. 
    // Ensure all CS pins are de-asserted.
    pinMode(TFT_CS_LEFT, OUTPUT);
    pinMode(TFT_CS_RIGHT, OUTPUT);
    pinMode(MAX_CS, OUTPUT);
    digitalWrite(MAX_CS, HIGH);
    digitalWrite(TFT_CS_LEFT, HIGH);
    digitalWrite(TFT_CS_RIGHT, HIGH);

    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);

    Machine::initGpio();
    Display::initDisplay();
    Network::initWiFi();
    OTA::initOTA();
    HomeAssistant::init();

    bool isSensorsInitialized = SensorSampler::initialize();

    HeatControl::initControlLoop();

    SensorSampler::start();
    
    UI::setState(UI::UiState::Preheat);
    //UI::setState(UiState::SensorTest);

    // If sensors could not be initialized, indicate fault
    if (UI::uiState != UI::UiState::SensorTest && (!isSensorsInitialized))
    {
        UI::setFault(UI::FaultState::SensorFailure);
        return;
    }


    Serial.println("Done!");
}

void loop()
{
    OTA::handle();

    if (WiFi.status() == WL_CONNECTED) {
        HomeAssistant::process();
    }

    if (UI::uiState != UI::UiState::FirmwareUpdate && 
        UI::uiState != UI::UiState::Off)
    {
        UI::processState();

        HeatControl::processControlLoop();

        UI::render();
    }
}
