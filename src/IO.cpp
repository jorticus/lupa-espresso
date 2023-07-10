#include <Arduino.h>
#include "IO.h"
#include "StateMachine.h"
#include "hardware.h"
#include "button.h"
#include "driver/touch_sensor.h"

// Temporary: For lever pull detection
#include "UI.h"
#include "SensorSampler.h"

static bool s_isHeaterOn = false;
static float s_heaterPower = 0.0;

static Buttons<
    Btn<PIN_IN_POWER_BTN, HIGH>
> buttons;

namespace IO {

void onButtonPress(int pin) {
    switch (pin) {
        case 0: // POWER_BTN
            Serial.println("PWR BTN PRESSED\n");

            if (State::uiState == State::MachineState::Fault) {
                esp_restart();
            }
            else {
                bool pwr = (State::uiState == State::MachineState::Off  || State::uiState == State::MachineState::Sleep);
                State::setPowerControl(pwr);
            }
            break;
    }
}

/// @brief Reset device into a fail-safe mode
/// where any outputs are turned off.
void failsafe() {
    digitalWrite(PIN_OUT_HEAT, LOW);
    digitalWrite(PIN_OUT_PUMP, LOW);
    digitalWrite(PIN_OUT_FILL_SOLENOID, LOW);

    //digitalWrite(TFT_BL, LOW);

    s_heaterPower = 0.0f;
}

void initGpio() {
    // BOOT button used for debugging
    pinMode(0, INPUT);

    pinMode(PIN_IN_POWER_BTN, INPUT_PULLDOWN);
    pinMode(PIN_IN_LEVER, INPUT_PULLDOWN);
    pinMode(PIN_IN_WATER_LOW, INPUT_PULLUP);
    //pinMode(PIN_IN_WATER_FULL, INPUT_PULLDOWN);
    pinMode(PIN_OUT_HEAT, OUTPUT);
    pinMode(PIN_OUT_PUMP, OUTPUT);
    pinMode(PIN_OUT_FILL_SOLENOID, OUTPUT);
    //pinMode(PIN_IN_WATER_FULL, INPUT);

    // ledcAttachPin(PIN_IN_WATER_FULL, 1);
    // ledcSetup(1, 333, 8);
    // ledcWrite(1, 0x7F);

    touchSetCycles(0xF000, 0xF000);

    buttons.onButtonPress(onButtonPress);

    // Initialize touch sensor input
    // https://github.com/ESP32DE/esp-iot-solution-1/blob/master/documents/touch_pad_solution/touch_sensor_design_en.md
    touchRead(T0);
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
    

    failsafe();
}

void process() {
    buttons.process();

    static unsigned long t_last = 0;
    static int cycle = 0;
    if ((millis() - t_last) > 1000) {
        t_last = millis();
        
        switch (cycle++) {
            case 0: // Begin sampling touch channel
                //touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER); 
                touch_pad_filter_start(10);
                touch_pad_sw_start();
                break;

            case 1: // Read touch channel and turn off sampling
                Serial.printf("Touch: %d\n", touchRead(T0));

                // TODO: Need to average 10 samples to filter out spikes
                // Generally this reads 0 when full, and ~80 when empty

                touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
                touch_pad_filter_stop();
                cycle = 0;
                break;

            default:
                cycle = 0;
                break;
        }
        

        
    }
}

bool isWaterTankLow() {
    return (digitalRead(PIN_IN_WATER_LOW) == HIGH);
}

bool isLeverPulled() {

    //return (digitalRead(PIN_IN_LEVER) == HIGH);

    // Detect lever by measuring water flow
    return (
        (State::uiState != State::MachineState::Preheat) && 
        SensorSampler::isFlowing()
    );

    //return (digitalRead(PIN_IN_LEVER) == HIGH);
    // TODO: Debouncing
    return false;
}

void setHeatPower(float duty) {
    // TODO
    s_heaterPower = duty;
}

void setHeat(bool en) {
    static bool prev_value = LOW;

    if (en) {
        if (en != prev_value) {
            Serial.println("HEAT: ON");
        }
        digitalWrite(PIN_OUT_HEAT, HIGH);
    }
    else {
        if (en != prev_value) {
            Serial.println("HEAT: OFF");
        }
        digitalWrite(PIN_OUT_HEAT, LOW);
    }

    prev_value = en;
    s_isHeaterOn = en;
}

void setWaterFillSolenoid(bool en) {
    digitalWrite(PIN_OUT_FILL_SOLENOID, en);
    // TODO: Set a watchdog timer that turns this off after X seconds
}

float getHeatPower() {
    return s_heaterPower;
}

bool isHeaterOn() {
    return s_isHeaterOn;
}

}