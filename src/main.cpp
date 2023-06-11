//#include <array>
#include <Arduino.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_GC9A01A.h>
//#include <Arduino_GFX.h>
#include <Wire.h>
//#include <Adafruit_SPIDevice.h>
#include <Adafruit_MAX31865.h>
#include <WiFi.h>
#include "secrets.h"
#include "PressureTransducer.h"
//#include "PulseCounter.h"
//#include <TFT_eSPI.h>
#include "config.h"
#include "hardware.h"
//#include "value_array.h"
#include "SensorSampler.h"
#include "IO.h"
#include "StateMachine.h"
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

    IO::initGpio();
    Display::initDisplay();
    Network::initWiFi();
    OTA::initOTA();
    HomeAssistant::init();

    bool isSensorsInitialized = SensorSampler::initialize();

    HeatControl::initControlLoop();

    SensorSampler::start();
    
    State::setState(State::MachineState::Preheat);
    //State::setState(MachineState::SensorTest);

    // If sensors could not be initialized, indicate fault
    if (State::uiState != State::MachineState::SensorTest && (!isSensorsInitialized))
    {
        State::setFault(State::FaultState::SensorFailure);
        return;
    }


    Serial.println("Done!");
}

void loop()
{
    Network::handle();
    OTA::handle();
    IO::process();

    if (WiFi.status() == WL_CONNECTED) {
        HomeAssistant::process();
    }

    if (State::uiState != State::MachineState::FirmwareUpdate && 
        State::uiState != State::MachineState::Off)
    {
        State::processState();

        HeatControl::processControlLoop();

        UI::render();
    }
}
