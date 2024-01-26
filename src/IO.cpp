#include <Arduino.h>
#include "IO.h"
#include "StateMachine.h"
#include "hardware.h"
#include "button.h"
#include "driver/touch_sensor.h"

// Temporary: For lever pull detection
#include "UI.h"
#include "SensorSampler.h"

#define USE_WATERLEVEL

// Reading is typically 0 when water is filled,
// and ~500 when it needs filling
const touch_value_t water_threshold_high = 20;
const touch_value_t water_threshold_low = 10;

static bool s_isHeaterOn = false;
static float s_heaterPower = 0.0;
static bool s_waterLow = false;

extern "C" {
    uint8_t temprature_sens_read();
}

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

    s_heaterPower = 0.0f;
}

void initGpio() {
    // BOOT button used for debugging
    pinMode(0, INPUT);

    pinMode(PIN_IN_POWER_BTN, INPUT_PULLDOWN);
    pinMode(PIN_IN_LEVER, INPUT_PULLDOWN);
    pinMode(PIN_IN_WATER_LOW, INPUT_PULLDOWN);
    pinMode(PIN_OUT_HEAT, OUTPUT);
    pinMode(PIN_OUT_PUMP, OUTPUT);
    pinMode(PIN_OUT_FILL_SOLENOID, OUTPUT);

    buttons.onButtonPress(onButtonPress);

#ifdef USE_WATERLEVEL
    // Initialize touch sensor input, used to detect boiler water level
    // https://github.com/ESP32DE/esp-iot-solution-1/blob/master/documents/touch_pad_solution/touch_sensor_design_en.md
    touchSetCycles(0xF000, 0xF000);
    touchRead(T0);
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
#endif

    failsafe();
}

#ifdef USE_WATERLEVEL
void readWaterLevel() {
    // Detect the boiler water level using the touch peripheral
    static unsigned long t_last = 0;
    static unsigned long fill_counter = 0;
    static int cycle = 0;
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

                if (water_level_raw > water_threshold_high) {
                    if (fill_counter >= 5) {
                        s_waterLow = true;
                    }
                    else {
                        fill_counter++;
                    }
                }
                else if (water_level_raw < water_threshold_low) {
                    fill_counter = 0;
                    s_waterLow = false;
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
}

void disableWaterLevel() {
    touch_pad_filter_stop();
}
#endif

void process() {
    buttons.process();

    auto state = State::getState();

    static unsigned long t_last2 = 0;
    if ((millis() - t_last2) > 10000) {
        t_last2 = millis();

        auto itemp = temprature_sens_read();
        Serial.printf("iTemp: %d\n", itemp);
    }

    if (state == State::MachineState::Fault) {
        s_waterLow = false;
        return;
    }

    // Activate pump when lever pulled.
    // This is separate from the Brewing state logic to keep things simple.
    // In the future we could modulate the pump to give flow or pressure control.
    if (state == State::MachineState::Ready ||
        state == State::MachineState::Brewing ||
        state == State::MachineState::Preheat)
    {
        IO::setPump(IO::isLeverPulled());
    }
    else if (state != State::MachineState::FillTank) {
        IO::setPump(false);
    }

#ifdef USE_WATERLEVEL
    if (state == State::MachineState::FillTank ||
        state == State::MachineState::Ready ||
        state == State::MachineState::Brewing ||
        state == State::MachineState::Preheat)
    {
        readWaterLevel();
    }
    else
    {
        // Disable water level sensor to prevent
        // corrosion of metal probe
        disableWaterLevel();
    }
#endif
}

bool isWaterTankLow() {
    return (digitalRead(PIN_IN_WATER_LOW) == LOW);
}

bool isBoilerTankLow() {
    return s_waterLow;
}

bool isLeverPulled() {
    return (digitalRead(PIN_IN_LEVER) == HIGH);
}

bool isBrewing() {
    return (
        (State::uiState != State::MachineState::Preheat) && 
        (State::uiState != State::MachineState::FillTank) &&
        //SensorSampler::isFlowing()
        isLeverPulled()
    );
}

void setHeatPower(float duty) {
    // TODO: Use this for setting heater PWM
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