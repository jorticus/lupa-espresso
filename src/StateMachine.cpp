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

MachineState uiState    = MachineState::Init;
FaultState uiFault      = FaultState::NoFault;
BrewStats brewStats     = {0};

#define PREHEAT_TEMPERATURE_C (CONFIG_BOILER_TEMPERATURE_C - 5.0f)

const unsigned long lever_debounce_interval_ms = 500;
static unsigned long t_idle_start = 0;
static unsigned long t_steam_start = 0;
static bool power_state = true;

static const char* UiState_Str[] = {
    "Off",
    "Init",
    "Pre-Heat",
    "Ready",
    "Fault",
    "Brewing",
    "Sensor Test",
    "Firmware Update",
    "Sleep",
    "Tuning"
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
    int s = (int)uiState;
    if (s < sizeof(UiState_Str)) {
        Serial.print(UiState_Str[s]);
    }
    else {
        Serial.print(s);
    }
}

void resetIdleTimer() {
    Serial.println("Reset idle timer\n");
    t_idle_start = millis();
}

void onStateChanged(MachineState lastState, MachineState newState) {

    // Detect and handle sleep state
    switch (newState) {
        case MachineState::Sleep:
            Display::setBrightness(CONFIG_IDLE_BRIGHTNESS);
            HeatControl::setProfile(HeatControl::BoilerProfile::Idle);
            break;
            
        case MachineState::Off:
            // Display::setBrightness(0.0f);
            // Display::turnOff();
            HeatControl::setProfile(HeatControl::BoilerProfile::Off);
            IO::failsafe();

            // When transitioning to the off state, start the power-off animation.
            // Display will be turned off when the animation completes.
            UI::triggerAnimation(UI::Anim::PowerOff);
            break;

        case MachineState::Tuning:
            HeatControl::setProfile(HeatControl::BoilerProfile::Tuning);
            // Fall through to default...

        default:
            if (lastState == MachineState::Off) {
                // When transitioning from Off->On, start the power-on animation.
                // This will set the display brightness as needed.
                UI::triggerAnimation(UI::Anim::PowerOn);
            }
            // else {
            //     Display::setBrightness(CONFIG_FULL_BRIGHTNESS);
            // }

            // Set boiler to the brew profile if waking up from an idle/off state
            // (ie, not in the Steam profile)
            auto profile = HeatControl::getProfile();
            if (profile == HeatControl::BoilerProfile::Idle ||
                profile == HeatControl::BoilerProfile::Off)
            {
                HeatControl::setProfile(HeatControl::BoilerProfile::Brew);
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
        printState(_lastUiState); Serial.print("->"); printState(uiState); Serial.println();
        onStateChanged(_lastUiState, uiState);
        _lastUiState = uiState;
    }

    if (uiState == MachineState::Off || uiState == MachineState::FirmwareUpdate) {
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


    // If water tank is low, indicate fault
    // State {Ready|Preheat -> Fault}
    if (uiState != MachineState::Off && uiState != MachineState::Brewing) {
        if (IO::isWaterTankLow()) {
            uiFault = FaultState::LowWater;
            uiState = MachineState::Fault;
            return;
        }
    }


    // State {Preheat -> Sleep|Ready}
    if (uiState == MachineState::Preheat) {
        // Device is ready once temperature rises above the configured threshold
        if (SensorSampler::isTemperatureValid()) {
            auto t = SensorSampler::getTemperature();
            // PID tuning mode, wait until we get close to the PID control range
            if (CONFIG_DO_PID_TUNE) {
                if (t >= (CONFIG_BOILER_TUNING_TEMPERATURE_C - CONFIG_BOILER_PID_RANGE_C)) {
                    Serial.println("Begin PID tuning...");
                    uiState = MachineState::Tuning;
                    return;
                }
            }
            // Up to sleeping temperature, go to sleep
            // State {Preheat -> Sleep}
            else if (t > CONFIG_BOILER_IDLE_TEMPERATURE_C && CONFIG_IDLE_AFTER_PREHEAT) {
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

    // Deactivate regular machine logic in PID tuning mode
    if (CONFIG_DO_PID_TUNE) {
        return;
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
        brewStats.preinfuse_volume = 0;
        brewStats.total_volume = 0;
        brewStats.avg_brew_pressure = 0.0f;
        brewStats.brew_pressure_avg_count = 0;

        // Reset flow accumulation
        SensorSampler::resetFlowCounter();

        return;
    }

    if (uiState == MachineState::Brewing) {
        float pressure = SensorSampler::getPressure();

        // Detect end of pre-infusion by looking at when the pressure goes above a threshold
        const float brew_pressure_threshold = 7.5f;
        if ((brewStats.preinfuse_volume < 0.01f) && (pressure > brew_pressure_threshold)) {
            brewStats.preinfuse_volume = SensorSampler::getTotalFlowVolume();
            SensorSampler::resetFlowCounter();
            brewStats.avg_brew_pressure = 0.0f;
            brewStats.brew_pressure_avg_count = 0;
            // TODO: With blank inserted, this still accumulates about 6mL
        }

        // Detect not getting up to pressure
        // Usually preinfusion lasts 6-7 seconds, but this is mechanically controlled.
        const unsigned long max_preinfuse_time_ms = 8000;
        if ((millis() - brewStats.start_brew_time) > max_preinfuse_time_ms) {
            if (pressure < brew_pressure_threshold) {
                //State::setFault(FaultState::NotHeating);
                // TODO: How to indicate this to the user?
            }
        }

        // Accumulate statistics
        brewStats.avg_brew_pressure += pressure;
        brewStats.brew_pressure_avg_count++;

        // If lever is released, stop brewing
        // State {Brewing -> Ready}
        if (!IO::isLeverPulled()) {
            uiState = MachineState::Ready;
            resetIdleTimer();

            UI::uiFreezeGraphs();

            // Stop brew timer
            brewStats.end_brew_time = millis();

            brewStats.total_volume = SensorSampler::getTotalFlowVolume();

            // if (brewStats.brew_pressure_avg_count > 0) {
            //     brewStats.avg_brew_pressure /= (float)brewStats.brew_pressure_avg_count;
            // }

            // Switch into steaming mode (only if brew was longer than 10sec)
            if ((brewStats.end_brew_time - brewStats.start_brew_time) > 10000) {
                HeatControl::setProfile(HeatControl::BoilerProfile::Steam);
            }
            return;
        }
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
            HeatControl::setProfile(HeatControl::BoilerProfile::Brew);
            resetIdleTimer();
        }

        if ((t_idle_start > 0) && (CONFIG_IDLE_TIMEOUT_MS > 0) &&
            ((millis() - t_idle_start) >= (unsigned long)CONFIG_IDLE_TIMEOUT_MS))
        {
            // Go to sleep after idle timeout
            Serial.println("Idle timeout - going to sleep");
            uiState = MachineState::Sleep;
        }
    }
}


}