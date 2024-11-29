//#include <array>
#include <Arduino.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_GC9A01A.h>
//#include <Arduino_GFX.h>
#include <Wire.h>
//#include <Adafruit_SPIDevice.h>
#include <Adafruit_MAX31865.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include "secrets.h"
#include "PressureTransducer.h"
//#include "PulseCounter.h"
//#include <TFT_eSPI.h>
#include "config.h"
#include "hardware.h"
//#include "value_array.h"
#include "SensorSampler.h"
#include "IO.h"
#include "Debug.h"
#include "StateMachine.h"
#include "HeatControl.h"
#include "PressureControl.h"
#include "Display.h"
#include "UI.h"
#include "Network.h"
#include "OTA.h"
#include "HomeAssistant.h"
#include "Panic.h"
#include "Task.h"
#include "WebSrv.h"
#include <esp_task_wdt.h>

// NOTE: WiFiClient default timeout is 3 seconds, WDT should probably be longer than this.
const uint32_t WDT_TIMEOUT_SEC = 4;

Adafruit_MAX31865   rtd1(MAX1_CS, &Display::getSPIInstance());
Adafruit_MAX31865   rtd2(MAX2_CS, &Display::getSPIInstance());
PressureTransducer  pressure(PRESSURE_FULL_SCALE);


static TaskHandle_t task_core;
static TaskHandle_t task_network;
static TaskHandle_t task_ui;
static TaskHandle_t task_display;

static bool s_failsafe = true;

using namespace Display;

bool handle_reset() {
    auto reason = esp_reset_reason();
    Debug.printf("Reset Reason: %d\n", reason);
    if (reason == ESP_RST_PANIC || 
        reason == ESP_RST_INT_WDT ||
        reason == ESP_RST_WDT ||
        reason == ESP_RST_TASK_WDT)
    {
        Debug.print("Last reset was due to ");
        switch (reason) {
            case ESP_RST_PANIC:     Debug.print("FW Panic"); break;
            case ESP_RST_INT_WDT:   Debug.print("INT WDT"); break;
            case ESP_RST_WDT:       Debug.print("WDT"); break;
            case ESP_RST_TASK_WDT:  Debug.print("Task WDT"); break;
            default:                Debug.print(reason);
        }
        Debug.println(", entering failsafe mode...");

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

/**
 * Criticial initialization
 * Only initialize subsystems required for setting up OTA update and UI feedback
*/
void initCritical() {
    Serial.begin(9600);

    IO::initGpio();
    DebugLogger::init();

    Display::initDisplay();
    Network::initWiFi();
    OTA::initOTA();
}

/**
 * Network Task (CORE 1, PRIORITY 2)
 * Handles all network connection logic.
 * Must be higher priority so it can preempt other tasks.
*/
void taskNetworkFunc(void* ctx) {
    Debug.println("Network Task Started");

    HomeAssistant::init();
    WebSrv::setup();

    while (true) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        Network::handle();

        if (Network::isConnected())
        {
            // TODO: This causes problems when activating OTA...
            OTA::handle(); // May block

            DebugLogger::process();

            HomeAssistant::process();

            WebSrv::process();
        }
    }
}

/**
 * Core Task (CORE 1, PRIORITY 3) HIGH PRIORITY
 * Handles I/O, State Machine, Control Loops
*/
void taskCoreFunc(void* ctx) {
    Debug.println("Core Task Started");

    while (true) {
        vTaskDelay(1 / portTICK_PERIOD_MS);

        IO::process();

        SensorSampler::process();
        State::processState();

        if (State::uiState != State::MachineState::FirmwareUpdate && 
            State::uiState != State::MachineState::Off)
        {
            HeatControl::processControlLoop();
            PressureControl::processControlLoop();
        }

    }
    //vTaskDelete(nullptr);
}

/**
 * Render Task (CORE 1, PRIORITY 1)
 * Handles rendering of UI.
 * Must be low priority so other tasks can preempt it.
*/
void taskRenderUiFunc(void* ctx) {
    Debug.println("Render Task Started");

    while (true) {
        taskYIELD();
        UI::render();
    }
}



void initSystem() {

    //
    // Non-critical initialization
    //

    IO::initPwm();
    //HomeAssistant::init();

    bool isSensorsInitialized = SensorSampler::initialize();

    HeatControl::initControlLoop();
    PressureControl::initControlLoop();

    SensorSampler::start();

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("An error occurred while mounting SPIFFS");
    }
    
    // Power-on state:
    //State::setState(State::MachineState::Preheat);
    State::setState(State::MachineState::Off);
    //State::setState(State::MachineState::SensorTest);

    // If sensors could not be initialized, indicate fault
    if (State::uiState != State::MachineState::SensorTest && 
       (!isSensorsInitialized))
    {
        State::setFault(State::FaultState::SensorFailure, "INIT FAILURE");
    }

    xTaskCreatePinnedToCore(taskCoreFunc,     "CoreTask", TASK_STACK_SIZE, nullptr, 3, &task_core,    CORE1);
    xTaskCreatePinnedToCore(taskNetworkFunc,  "NetTask",  TASK_STACK_SIZE, nullptr, 2, &task_network, CORE1);
    xTaskCreatePinnedToCore(taskRenderUiFunc, "UiTask",   TASK_STACK_SIZE, nullptr, 1, &task_ui,      CORE1);
}

/**
 * Main setup entrypoint
*/
void setup() {
    // Disable watchdog during init to give system time to boot.
    // The WiFi especially can take a significant amount of time to connect.
    esp_task_wdt_init(10, false);

    initCritical();

    // Enter fail-safe mode if we've encountered a firmware bug,
    // before proceeding further. This allows us to recover via OTA.
    if (handle_reset()) {
        return;
    }
    // Holding power button on reset will also enter recovery mode.
    if (digitalRead(PIN_IN_POWER_BTN) == HIGH) {
        while (digitalRead(PIN_IN_POWER_BTN) == HIGH) continue;
        State::setFault(State::FaultState::FailsafeRecovery);
        return;
    }
#if defined(FAILSAFE_RECOVERY)
    // Building with this flag set will always enter recovery mode.
    State::setFault(State::FaultState::FailsafeRecovery);
    return;
#endif
    s_failsafe = false;

    initSystem();

    // Enable watchdog timer
    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
    esp_task_wdt_add(NULL); //add current thread to WDT watch

    Debug.println("Done!");
}

/**
 * Background RTOS task (CORE 1, PRIORITY 1)
*/
void loop()
{
    esp_task_wdt_reset();
    taskYIELD();

    // DO NOT MODIFY
    // This ensures the system can continue to receive firmware updates.
    if (s_failsafe) {
        IO::process();
        Network::handle();

        if (Network::isConnected())
        {
            OTA::handle();
            DebugLogger::process();
        }

        UI::renderFailsafe();
    }
}
