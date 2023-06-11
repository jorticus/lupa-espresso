#include <Arduino.h>
#include "IO.h"
#include "StateMachine.h"
#include "hardware.h"
#include "button.h"

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
            bool pwr = (State::uiState == State::MachineState::Off  || State::uiState == State::MachineState::Sleep);
            State::setPowerControl(pwr);
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
    pinMode(PIN_IN_WATER_FULL, INPUT_PULLDOWN);
    pinMode(PIN_OUT_HEAT, OUTPUT);
    pinMode(PIN_OUT_PUMP, OUTPUT);
    pinMode(PIN_OUT_FILL_SOLENOID, OUTPUT);

    buttons.onButtonPress(onButtonPress);

    failsafe();
}

void process() {
    buttons.process();
}

bool isWaterTankLow() {
    return (digitalRead(PIN_IN_WATER_LOW) == HIGH);
}

bool isLeverPulled() {

    //return (digitalRead(PIN_IN_LEVER) == HIGH);

    // Detect lever by measuring water flow
    return (
        (State::uiState != State::MachineState::Preheat) && 
        SensorSampler::isFlowRateValid() && 
        (SensorSampler::getFlowRate() > 1.0f)
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