#include "UI.h"
#include "Machine.h"
#include "SensorSampler.h"
#include "config.h"
#include "HeatControl.h"
#include "Machine.h"
#include "HomeAssistant.h"

namespace UI {

void uiFreezeGraphs(); // UI.cpp

BrewStats brewStats = {0};

#define PREHEAT_TEMPERATURE_C (CONFIG_BOILER_TEMPERATURE_C - 5.0f)

const unsigned long lever_debounce_interval_ms = 500;
static unsigned long t_idle_start = 0;
static unsigned long t_steam_start = 0;
static bool power_state = true;

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

UiState getState() {
    return uiState;
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

            // TODO: Should we report sleep as ON or OFF?
            //HomeAssistant::reportPowerControlState(true);
            break;
        case UiState::Off:
            Display::setBrightness(0.0f);
            Display::turnOff();
            HeatControl::setMode(HeatControl::Mode::Off);
            Machine::failsafe();
            //HomeAssistant::reportPowerControlState(false);
            break;
        default:
            Display::setBrightness(CONFIG_FULL_BRIGHTNESS);

            auto heatMode = HeatControl::getMode();
            if (heatMode == HeatControl::Mode::Sleep ||
                heatMode == HeatControl::Mode::Off)
            {
                HeatControl::setMode(HeatControl::Mode::Brew);
                //HomeAssistant::reportPowerControlState(true);
            }
            break;
    }
}

void setPowerControl(bool pwr)
{
    Serial.print("POWER: ");
    Serial.println(pwr ? "ON" : "OFF");

    if (pwr) {
        if (uiState == UiState::Off || uiState == UiState::Sleep) {
            uiState = UiState::Preheat;
            resetIdleTimer();
        }
    }
    else {
        uiState = UiState::Off;
    }

    // Force an update of state before returning to ensure
    // we're in the right state before updating the UI,
    // as machine may be faulted, or already preheated, etc.
    processState();
}

void processState()
{
    static UiState _lastUiState = UiState::Init;
    if (uiState != _lastUiState) {
        _lastUiState = uiState;
        printState(uiState);
        onStateChanged();
    }

    if (uiState == UiState::Off) {
        return;
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
        if (SensorSampler::isTemperatureValid()) {
            auto t = SensorSampler::getTemperature();
            // Up to sleeping temperature, go to sleep
            if (t > CONFIG_BOILER_SLEEP_TEMPERATURE_C && CONFIG_SLEEP_AFTER_PREHEAT) {
                uiState = UiState::Sleep;
            }
            // Close to boiler temperature, go to ready
            else if (t >= PREHEAT_TEMPERATURE_C) {
                uiState = UiState::Ready;
                resetIdleTimer();
            }
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
        if (uiState == UiState::Sleep) {
            // Wake from sleep, go to preheat if not yet hot enough
            if (SensorSampler::isTemperatureValid() && (SensorSampler::getTemperature() < PREHEAT_TEMPERATURE_C)) {
                uiState = UiState::Preheat;
                resetIdleTimer();
                return;
            }
        }

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

        // Switch into steaming mode (only if brew was longer than 10sec)
        if ((brewStats.end_brew_time - brewStats.start_brew_time) > 10000) {
            HeatControl::setMode(HeatControl::Mode::Steam);
        }
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