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
    "Tuning",
    "Fill Tank"
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

/// @brief Detect faults and enter a faulted state if needed
void detectFaults()
{
    if (IO::isWaterTankLow()) {
        uiFault = FaultState::LowWater;
        uiState = MachineState::Fault;
        return;
    }
}

/// @brief Determine if temperature is within the allowable PID tuning range
/// @param t Current boiler temperature
/// @return true if temperature is in range of PID tune
bool isTemperatureInTuningRange(float t) {
    // PID tuning mode, wait until we get close to the PID control range
    return (t >= (CONFIG_BOILER_TUNING_TEMPERATURE_C - CONFIG_BOILER_PID_RANGE_C));
}

/// @brief Begin a brew
void beginBrew()
{
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
}

/// @brief End a brew
void endBrew()
{
    resetIdleTimer();

    UI::uiFreezeGraphs();

    // Stop brew timer
    brewStats.end_brew_time = millis();

    brewStats.total_volume = SensorSampler::getTotalFlowVolume();

    // Switch into steaming mode (only if brew was longer than 10sec)
    if ((brewStats.end_brew_time - brewStats.start_brew_time) > 10000) {
        HeatControl::setProfile(HeatControl::BoilerProfile::Steam);
    }
}

/// @brief Process brewing state
void stateBrew()
{
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
}

/// @brief Exit the post-brew phase
void resetPostBrew() {
    // Reset ready page after some timeout
    brewStats.end_brew_time = 0;
    brewStats.start_brew_time = 0;

    resetIdleTimer();
}

bool isIdleTimeoutElapsed() {
    return ((t_idle_start > 0) && (CONFIG_IDLE_TIMEOUT_MS > 0) &&
        ((millis() - t_idle_start) >= (unsigned long)CONFIG_IDLE_TIMEOUT_MS));
}

bool isPostBrewTimeoutElapsed() {
    return ((brewStats.end_brew_time > 0) && 
        ((millis() - brewStats.end_brew_time) >= (unsigned long)CONFIG_BREW_FINISH_TIMEOUT_MS));
}

void beginFillTankCycle()
{
    Serial.println("Boiler tank is low, activating fill cycle\n");

    IO::setWaterFillSolenoid(true);
    IO::setPump(true);
}

/// @brief One time trigger on state transition
/// @param lastState The previous state
/// @param newState The new state
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

        case MachineState::Fault:
            IO::failsafe();
            break;

        case MachineState::Brewing:
            beginBrew();
            break;

        case MachineState::FillTank:
            beginFillTankCycle();
            break;

        case MachineState::Tuning:
            HeatControl::setProfile(HeatControl::BoilerProfile::Tuning);
            // Fall through to default...
        default:
            if (lastState == MachineState::Off || lastState == MachineState::Sleep) {
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

/// @brief Main state machine processing block
void processState()
{
    static MachineState _lastUiState = MachineState::Init;
    if (uiState != _lastUiState) {
        printState(_lastUiState); Serial.print("->"); printState(uiState); Serial.println();
        onStateChanged(_lastUiState, uiState);
        _lastUiState = uiState;
    }

    // The state machine is structured by processing each state,
    // determining if a state change needs to happen,
    // and optionally performing an action on the transition to the new state.
    switch (uiState) {
        case MachineState::Init:
        case MachineState::Off:
            // Machine is off. State change to on is handled in IO.cpp via button event
            break;

        case MachineState::FirmwareUpdate:
            // Nothing to do in firmware update mode
            break;

        // NOTE: Sleep state is currently unused.
        // This is supposed to put the device into a lower power state that keeps the boiler warm but ready to heat back up to full temperature.
        // Measurements showed this doesn't save much power.
        case MachineState::Sleep:
            if (IO::isBrewing() || IO::isLeverPulled()) {
                // When waking from sleep, go to preheat state if we're not yet up to temperature
                if (SensorSampler::isTemperatureValid() && (!SensorSampler::isTemperatureStabilized())) {
                    uiState = MachineState::Preheat;
                    resetIdleTimer();
                }
                else {
                    uiState = MachineState::Brewing;
                }
            }
            break;


        case MachineState::Preheat:
            detectFaults();

            if (SensorSampler::isTemperatureValid()) {
                auto t = SensorSampler::getTemperature();
                if (CONFIG_DO_PID_TUNE && isTemperatureInTuningRange(t)) {
                    Serial.println("Begin PID tuning...");
                    uiState = MachineState::Tuning;
                }
                // Up to sleeping temperature, go to sleep
                else if (CONFIG_IDLE_AFTER_PREHEAT && (t > CONFIG_BOILER_IDLE_TEMPERATURE_C)) {
                    uiState = MachineState::Sleep;
                }
                // Close to boiler temperature, go to ready
                else if (SensorSampler::isTemperatureStabilized()) {
                    uiState = MachineState::Ready;
                    resetIdleTimer();
                }
                // TODO: Timeout fault if temperature never stabilizes...
            }

            if (IO::isBoilerTankLow()) {
                uiState = MachineState::FillTank;
            }
            if (IO::isBrewing()) {
                // (Won't result in a good brew, but allow this for testing / flushing the system)
                uiState = MachineState::Brewing;
            }
            break;


        case MachineState::Ready:
            detectFaults();

            if (isIdleTimeoutElapsed()) {
                Serial.println("Idle timeout - going to sleep");
                uiState = MachineState::Sleep;
            }

            // After post-brew timeout, return from steam to regular brew profile
            if (isPostBrewTimeoutElapsed()) {
                resetPostBrew();
                HeatControl::setProfile(HeatControl::BoilerProfile::Brew);
            }

            if (IO::isBoilerTankLow()) {
                uiState = MachineState::FillTank;
            }
            if (IO::isBrewing()) {
                uiState = MachineState::Brewing;
            }
            break;


        case MachineState::Brewing:
            // NOTE: Deliberately not checking boiler tank or faults
            // so we can finish the shot.
            // TODO: But we may still want to detect any absolutely critical faults...
            //detectFaults();
            stateBrew();

            // If lever is released, stop brewing
            if (!IO::isBrewing()) {
                endBrew();
                uiState = MachineState::Ready;
            }
            break;


        case MachineState::Fault:
            // If fault was low tank, and tank is no longer low, clear the fault.
            // Go directly to the preheat phase in case we're not up to temp,
            // since preheat will transition to ready if we are able to.
            // Transition Fault -> Preheat (-> Ready)
            if (uiFault == FaultState::LowWater && !IO::isWaterTankLow()) {
                uiFault = FaultState::NoFault;
                uiState = MachineState::Preheat;
                resetIdleTimer();
            }
            break;


        case MachineState::SensorTest:
            detectFaults();
            break;


        case MachineState::FillTank:
            // This state is entered when we need to fill the boiler tank.
            // Brewing is not allowed during this time.
            detectFaults();

            if (!IO::isBoilerTankLow()) {
                // Boiler tank is now full.
                // Go to preheat in case we are still preheating,
                // otherwise preheat will transition directly to Ready.
                uiState = MachineState::Preheat;

                IO::setPump(false);
                IO::setWaterFillSolenoid(false);
            }
            break;
    }
}


}