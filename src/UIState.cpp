#include "UI.h"
#include "Machine.h"
#include "SensorSampler.h"
#include "config.h"

namespace UI {

void uiFreezeGraphs(); // UI.cpp

BrewStats brewStats = {0};

const float preheat_temperature = CONFIG_BOILER_PREHEAT_TEMPERATURE_C;

static const char* UiState_Str[] = {
    "Init",
    "Pre-Heat",
    "Ready",
    "Fault",
    "Brewing",
    "Post-Brew",
    "Sensor Test",
    "Firmware Update"
};

void setState(UiState state) {
    uiState = state;
}

void setFault(FaultState state) {
    uiState = UiState::Fault;
    uiFault = state;
}


static void printState(UiState uiState) {
    Serial.print("State: ");
    int s = (int)uiState;
    if (s < sizeof(UiState_Str)) {
        Serial.println(UiState_Str[s]);
    }
    else {
        Serial.println(s);
    }
}


void processState()
{
    static UiState _lastUiState = UiState::Init;
    if (uiState != _lastUiState) {
        _lastUiState = uiState;
        printState(uiState);
    }

    // NOTE: Process faults first.

/*
    if (uiState != UiState::SensorTest && 
        (!values.f_valid || !values.p_valid || !values.t_valid))
    {
        uiFault = FaultState::SensorFailure;
        uiState = UiState::Fault;
        return;
    }
*/

    // If water tank is low at any point, indicate fault
    if (Machine::isWaterTankLow()) {
        uiFault = FaultState::LowWater;
        uiState = UiState::Fault;
        return;
    }

    if (uiState == UiState::Preheat) {
        // Device is ready once temperature rises above the configured threshold
        if (SensorSampler::isTemperatureValid() && (SensorSampler::getTemperature() > preheat_temperature)) {
            uiState = UiState::Ready;
            return;
        }
    }

    if (uiState == UiState::Fault) {
        // If fault was low tank, and tank is no longer low, clear the fault
        if (uiFault == FaultState::LowWater && !Machine::isWaterTankLow()) {
            uiFault = FaultState::NoFault;
            uiState = UiState::Ready;
        }
    }

    // If lever is actuated at any time, move to brew phase.
    if ((uiState != UiState::Brewing) && Machine::isLeverPulled()) {
        uiState = UiState::Brewing;

        // Restart brew timer
        brewStats.start_brew_time = millis();
        brewStats.end_brew_time = 0;

        // Reset flow accumulation
        SensorSampler::resetFlowCounter();
        return;
    }

    // If lever is released, stop brewing
    if (uiState == UiState::Brewing && !Machine::isLeverPulled()) {
        //uiState = UiState::PostBrew;
        uiState = UiState::Ready;

        uiFreezeGraphs();

        // Stop brew timer
        brewStats.end_brew_time = millis();
        return;
    }

    if (uiState == UiState::Ready && ((millis() - brewStats.end_brew_time) > 60000)) {
        // Reset ready page after some timeout
        brewStats.end_brew_time = 0;
        brewStats.start_brew_time = 0;
    }
}


}