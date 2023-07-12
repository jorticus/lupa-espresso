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
    pinMode(PIN_IN_WATER_LOW, INPUT_PULLDOWN);
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

    auto state = State::getState();

    if (state == State::MachineState::Fault)
        return;

    static unsigned long t_last = 0;
    static unsigned long fill_counter = 0;
    static int cycle = 0;
    static bool level_state = false;
    if ((millis() - t_last) > 500) {
        t_last = millis();
        
        switch (cycle++) {
            case 0: // Begin sampling touch channel
                //touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER); 
                touch_pad_filter_start(10);
                touch_pad_sw_start();
                break;

            case 1: // Read touch channel and turn off sampling
            {
                auto water_level_raw = touchRead(T0);
                Serial.printf("WaterLevel: %d\n", water_level_raw);

                const touch_value_t water_threshold_high = 400;
                const touch_value_t water_threshold_low = 200;
                if (water_level_raw > water_threshold_high) {
                    if (fill_counter >= 5) {
                        if (!level_state) {
                            level_state = true;
                            Serial.printf("Water level low!\n");

                            // Activate water fill cycle after 5 seconds of this reading low
                            // TODO: Move this into StateMachine.cpp
                            State::setState(State::MachineState::FillTank);
                        }
                    }
                    else {
                        fill_counter++;
                    }
                }
                else if (water_level_raw < water_threshold_low) {
                    if (state == State::MachineState::FillTank) {
                        State::setState(State::MachineState::Ready);
                        IO::setPump(false);
                        IO::setWaterFillSolenoid(false);
                        fill_counter = 0;
                        level_state = false;
                    }
                }

                touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
                touch_pad_filter_stop();
                cycle = 0;
                break;
            }

            default:
                cycle = 0;
                break;
        }
    }

    
    // if (state == State::MachineState::Ready ||
    //     state == State::MachineState::Brewing)
    if (state != State::MachineState::Off)
    {
        bool lever = (digitalRead(PIN_IN_LEVER) == HIGH);
        digitalWrite(PIN_OUT_PUMP, lever);
    }
    else{
        digitalWrite(PIN_OUT_PUMP, LOW);
    }
}

bool isWaterTankLow() {
    return (digitalRead(PIN_IN_WATER_LOW) == LOW);
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

void setPump(bool en) {
    digitalWrite(PIN_OUT_PUMP, en);
    // TODO: Set a watchdog timer that turns this off after X seconds
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