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
#include <esp_task_wdt.h>

#define WDT_TIMEOUT 1

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

    // Enter fail-safe mode if we've encountered a firmware bug,
    // before proceeding further. This allows us to recover via OTA.
    auto reason = esp_reset_reason();
    Serial.printf("Reset Reason: %d\n", reason);
    if (reason == ESP_RST_PANIC || 
        reason == ESP_RST_INT_WDT ||
        reason == ESP_RST_WDT ||
        reason == ESP_RST_TASK_WDT)
    {
        Serial.println("Last reset was due to FW panic, entering failsafe mode...");

        State::setFault(State::FaultState::SoftwarePanic);
        return;
    }

    HomeAssistant::init();

    bool isSensorsInitialized = SensorSampler::initialize();

    HeatControl::initControlLoop();

    SensorSampler::start();
    
    // Power-on state:
    //State::setState(State::MachineState::Preheat);
    State::setState(State::MachineState::Off);
    //State::setState(MachineState::SensorTest);

    // If sensors could not be initialized, indicate fault
    if (State::uiState != State::MachineState::SensorTest && (!isSensorsInitialized))
    {
        State::setFault(State::FaultState::SensorFailure);
        return;
    }

    // Enable watchdog timer
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL); //add current thread to WDT watch

    Serial.println("Done!");
}

void loop()
{
    esp_task_wdt_reset();

    Network::handle();
    OTA::handle();

    if (WiFi.status() == WL_CONNECTED) {
        HomeAssistant::process();
    }

    IO::process();
    SensorSampler::process();
    State::processState();

    if (State::uiState != State::MachineState::FirmwareUpdate && 
        State::uiState != State::MachineState::Off)
    {
        HeatControl::processControlLoop();
    }

    UI::render();
}
