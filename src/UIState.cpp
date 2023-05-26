#include "UI.h"
#include "Machine.h"
#include "SensorSampler.h"
#include "config.h"
#include "HeatControl.h"

namespace UI {

void uiFreezeGraphs(); // UI.cpp

BrewStats brewStats = {0};

const float preheat_temperature = CONFIG_BOILER_PREHEAT_TEMPERATURE_C;

const unsigned long lever_debounce_interval_ms = 500;
static unsigned long t_idle_start = 0;
static unsigned long t_steam_start = 0;

static const char* UiState_Str[] = {
    "Init",
    "Pre-Heat",
    "Ready",
    "Fault",
    "Brewing",
    "Post-Brew",
    "Sensor Test",
    "Firmware Update",
    "Sleep",
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

void resetIdleTimer() {
    t_idle_start = millis();
}

void onStateChanged() {

    // Detect and handle sleep state
    switch (uiState) {
        case UiState::Sleep:
            Display::setBrightness(CONFIG_IDLE_BRIGHTNESS);
            HeatControl::setMode(HeatControl::Mode::Sleep);
            break;
        default:
            Display::setBrightness(CONFIG_FULL_BRIGHTNESS);
            if (HeatControl::getMode() == HeatControl::Mode::Sleep) {
                HeatControl::setMode(HeatControl::Mode::Brew);
            }
            break;
    }
}

void processState()
{
    static UiState _lastUiState = UiState::Init;
    if (uiState != _lastUiState) {
        _lastUiState = uiState;
        printState(uiState);
        onStateChanged();
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
            if (CONFIG_SLEEP_AFTER_PREHEAT) {
                uiState = UiState::Sleep;
            }
            else {
                uiState = UiState::Ready;
            }

            resetIdleTimer();
            return;
        }
    }

    if (uiState == UiState::Fault) {
        // If fault was low tank, and tank is no longer low, clear the fault
        if (uiFault == FaultState::LowWater && !Machine::isWaterTankLow()) {
            uiFault = FaultState::NoFault;
            uiState = UiState::Ready;
            resetIdleTimer();
        }
    }

    // If lever is actuated at any time, move to brew phase.
    if ((uiState != UiState::Brewing) && Machine::isLeverPulled()) {
        uiState = UiState::Brewing;
        resetIdleTimer();
        auto t_now = millis();

        // De-bounce
        // (prevent another shot from being registered if lever is quickly released and pulled again)
        if ((brewStats.end_brew_time > 0) && ((t_now - brewStats.end_brew_time) < lever_debounce_interval_ms)) {
            return;
        }

        // Restart brew timer
        brewStats.start_brew_time = t_now;
        brewStats.end_brew_time = 0;

        // Reset flow accumulation
        SensorSampler::resetFlowCounter();
        return;
    }

    // If lever is released, stop brewing
    if (uiState == UiState::Brewing && !Machine::isLeverPulled()) {
        //uiState = UiState::PostBrew;
        uiState = UiState::Ready;
        resetIdleTimer();

        uiFreezeGraphs();

        // Stop brew timer
        brewStats.end_brew_time = millis();

        // Switch into steaming mode
        HeatControl::setMode(HeatControl::Mode::Steam);
        return;
    }

    if (uiState == UiState::Ready) {

        if ((brewStats.end_brew_time > 0) && 
            ((millis() - brewStats.end_brew_time) >= (unsigned long)CONFIG_BREW_FINISH_TIMEOUT_MS))
        {
            // Reset ready page after some timeout
            brewStats.end_brew_time = 0;
            brewStats.start_brew_time = 0;

            // Return to brew mode
            HeatControl::setMode(HeatControl::Mode::Brew);
            resetIdleTimer();
        }

        if ((t_idle_start > 0) &&
            ((millis() - t_idle_start) >= (unsigned long)CONFIG_IDLE_TIMEOUT_MS))
        {
            // Go to sleep after idle timeout
            Serial.println("Idle timeout - going to sleep");
            uiState = UiState::Sleep;
        }
    }
}


}