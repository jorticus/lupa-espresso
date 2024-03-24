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
#include "PressureControl.h"
#include "Display.h"
#include "UI.h"
#include "Network.h"
#include "OTA.h"
#include "HomeAssistant.h"
#include <esp_task_wdt.h>

#define WDT_TIMEOUT 3

Stream& Debug = Serial;

/// @brief Network name of the device
const char* DEVICE_NAME = "LUPA";

Adafruit_MAX31865   rtd(MAX_CS, &Display::getSPIInstance());
PressureTransducer  pressure(PRESSURE_FULL_SCALE);

static bool s_failsafe = true;

using namespace Display;

// int custom_vprintf(const char* str, va_list l) {
//     static char buf[128];
//     auto len = vsnprintf(buf, sizeof(buf), str, l);
//     Debug.write("ESP:");
//     return Debug.write(buf, len);
// }

extern "C" {
    void clear_panic_buffer();
    void print_panic_buffer();
    const char* get_panic_buffer();
}

bool handle_reset() {
    auto reason = esp_reset_reason();
    Serial.printf("Reset Reason: %d\n", reason);
    if (reason == ESP_RST_PANIC || 
        reason == ESP_RST_INT_WDT ||
        reason == ESP_RST_WDT ||
        reason == ESP_RST_TASK_WDT)
    {
        Serial.print("Last reset was due to ");
        switch (reason) {
            case ESP_RST_PANIC: Serial.print("FW Panic"); break;
            case ESP_RST_INT_WDT: Serial.print("INT WDT"); break;
            case ESP_RST_WDT: Serial.print("WDT"); break;
            case ESP_RST_TASK_WDT: Serial.print("Task WDT"); break;
            default: Serial.print(reason);
        }
        Serial.println(", entering failsafe mode...");

        if (reason == ESP_RST_PANIC) {
            print_panic_buffer();
            State::setFault(State::FaultState::SoftwarePanic, "PANIC");
        }
        else {
            State::setFault(State::FaultState::SoftwarePanic, "WATCHDOG");
        }

        clear_panic_buffer();

        return true;
    }
    else {
        clear_panic_buffer();
        return false;
    }
}

void setup() {
    Serial.begin(9600);

    // Disable watchdog during init to give system time to boot.
    // The WiFi especially can take a significant amount of time to connect.
    esp_task_wdt_init(10, false);

    IO::initGpio();
    Display::initDisplay();
    Network::initWiFi();
    OTA::initOTA();

    // Enter fail-safe mode if we've encountered a firmware bug,
    // before proceeding further. This allows us to recover via OTA.
    if (handle_reset() || (digitalRead(PIN_IN_POWER_BTN) == HIGH)) {
        return;
    }
#if defined(FAILSAFE_RECOVERY)
    // Failsafe build
    return;
#endif
    s_failsafe = false;

    IO::initPwm();
    HomeAssistant::init();

    bool isSensorsInitialized = SensorSampler::initialize();

    HeatControl::initControlLoop();
    PressureControl::initControlLoop();

    SensorSampler::start();
    
    // Power-on state:
    //State::setState(State::MachineState::Preheat);
    State::setState(State::MachineState::Off);
    //State::setState(MachineState::SensorTest);

    // If sensors could not be initialized, indicate fault
    if (State::uiState != State::MachineState::SensorTest && (!isSensorsInitialized))
    {
        State::setFault(State::FaultState::SensorFailure, "FAIL_INIT");
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

    if (!s_failsafe) {
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
            PressureControl::processControlLoop();
        }
    
        UI::render();
    }
}
