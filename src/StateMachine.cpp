#include <Arduino.h>
#include "StateMachine.h"
#include "Display.h"
#include "IO.h"
#include "UI.h"
#include "SensorSampler.h"
#include "config.h"
#include "HeatControl.h"
#include "HomeAssistant.h"

namespace State {

MachineState uiState = MachineState::Init;
FaultState uiFault = FaultState::NoFault;
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

void setState(MachineState state) {
    uiState = state;
}

MachineState getState() {
    return uiState;
}

void setFault(FaultState state) {
    uiState = MachineState::Fault;
    uiFault = state;
}


static void printState(MachineState uiState) {
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
        case MachineState::Sleep:
            Display::setBrightness(CONFIG_IDLE_BRIGHTNESS);
            HeatControl::setMode(HeatControl::Mode::Sleep);
            break;
            
        case MachineState::Off:
            Display::setBrightness(0.0f);
            Display::turnOff();
            HeatControl::setMode(HeatControl::Mode::Off);
            IO::failsafe();
            break;

        default:
            Display::setBrightness(CONFIG_FULL_BRIGHTNESS);

            auto heatMode = HeatControl::getMode();
            if (heatMode == HeatControl::Mode::Sleep ||
                heatMode == HeatControl::Mode::Off)
            {
                HeatControl::setMode(HeatControl::Mode::Brew);
            }
            break;
    }
}

void setPowerControl(bool pwr)
{
    Serial.print("POWER: ");
    Serial.println(pwr ? "ON" : "OFF");

    if (pwr) {
        if (uiState == MachineState::Off || uiState == MachineState::Sleep) {
            uiState = MachineState::Preheat;
            resetIdleTimer();
        }
    }
    else {
        uiState = MachineState::Off;
    }

    // Force an update of state before returning to ensure
    // we're in the right state before updating the UI,
    // as machine may be faulted, or already preheated, etc.
    processState();
}

void processState()
{
    static MachineState _lastUiState = MachineState::Init;
    if (uiState != _lastUiState) {
        _lastUiState = uiState;
        printState(uiState);
        onStateChanged();
    }

    if (uiState == MachineState::Off) {
        return;
    }

    // NOTE: Process faults first.

/*
    if (uiState != MachineState::SensorTest && 
        (!values.f_valid || !values.p_valid || !values.t_valid))
    {
        uiFault = FaultState::SensorFailure;
        uiState = MachineState::Fault;
        return;
    }
*/

    // If water tank is low at any point, indicate fault
    // State {Any -> Fault}
    if (IO::isWaterTankLow()) {
        uiFault = FaultState::LowWater;
        uiState = MachineState::Fault;
        return;
    }


    // State {Preheat -> Sleep|Ready}
    if (uiState == MachineState::Preheat) {
        // Device is ready once temperature rises above the configured threshold
        if (SensorSampler::isTemperatureValid()) {
            auto t = SensorSampler::getTemperature();
            // Up to sleeping temperature, go to sleep
            // State {Preheat -> Sleep}
            if (t > CONFIG_BOILER_SLEEP_TEMPERATURE_C && CONFIG_SLEEP_AFTER_PREHEAT) {
                uiState = MachineState::Sleep;
            }
            // Close to boiler temperature, go to ready
            // State {Preheat -> Ready}
            else if (t >= PREHEAT_TEMPERATURE_C) {
                uiState = MachineState::Ready;
                resetIdleTimer();
            }
        }
    }

    if (uiState == MachineState::Fault) {
        // If fault was low tank, and tank is no longer low, clear the fault
        // State {Fault -> Preheat}
        if (uiFault == FaultState::LowWater && !IO::isWaterTankLow()) {
            uiFault = FaultState::NoFault;
            uiState = MachineState::Preheat;
            resetIdleTimer();
        }
    }

    // If lever is actuated at any time, move to brew phase.
    if ((uiState != MachineState::Brewing) && IO::isLeverPulled()) {
        if (uiState == MachineState::Sleep) {
            // Wake from sleep, go to preheat if not yet hot enough
            // State {Sleep -> Preheat}
            if (SensorSampler::isTemperatureValid() && (SensorSampler::getTemperature() < PREHEAT_TEMPERATURE_C)) {
                uiState = MachineState::Preheat;
                resetIdleTimer();
                return;
            }
        }

        // State {Preheat|Ready -> Brewing}
        uiState = MachineState::Brewing;
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
    // State {Brewing -> Ready}
    if (uiState == MachineState::Brewing && !IO::isLeverPulled()) {
        //uiState = MachineState::PostBrew;
        uiState = MachineState::Ready;
        resetIdleTimer();

        UI::uiFreezeGraphs();

        // Stop brew timer
        brewStats.end_brew_time = millis();

        // Switch into steaming mode (only if brew was longer than 10sec)
        if ((brewStats.end_brew_time - brewStats.start_brew_time) > 10000) {
            HeatControl::setMode(HeatControl::Mode::Steam);
        }
        return;
    }

    // State {Ready -> Ready|Sleep}
    if (uiState == MachineState::Ready) {

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
            uiState = MachineState::Sleep;
        }
    }
}


}