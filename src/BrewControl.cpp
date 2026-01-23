#include <Arduino.h>
#include "SensorSampler.h"
#include "BrewControl.h"
#include "BrewProfiles.h"
#include "MqttParamManager.h"
//#include "HomeAssistant.h"
#include "PID.h"
#include "IO.h"
#include "Debug.h"
#include "hardware.h"
#include "config.h"

#include <span>
#include <array>

// Default tunings
namespace Defaults {
    // How often PID is calculated
    // NOTE: If this is changed, the coefficients will need to be updated too
    static const unsigned long UpdatePeriodMs = 100;

    static const float Kp = 0.1f;
    static const float Ki = 0.01f; //Kp / 13.0f; // Ki = Kp / Tn
    static const float Kd = 0.0f;  //Kp * 5.0f; // Kd = Kp * Tv
    
    //static const float SetPoint = CONFIG_TARGET_BREW_PRESSURE;

    // Static offset needed to reach steady-state, determined empirically
    // Helps prevent integral windup
    static const float PlantOffset = 0.660f;

    // Only apply integral when within this range of setpoint,
    // to avoid integral windup during initial ramp up of pressure
    static const float RegulationRange = 2.0f;

    // Pressure which we must reach before switching on PID regulation
    static const float BeginRegulationPressure = 2.0f;
    static const float EndRegulationPressure = 1.0f;
};

static fPID pid;

static bool s_run = false;
static bool s_inrange = false;

namespace BrewControl {

enum class ControlMode {
    Disabled,
    Pressure,
    FlowRate
};

static BrewMode s_brewMode = BrewMode::ManualPressure;
static ControlMode s_controlMode = ControlMode::Pressure;

static uint64_t s_startTime = 0;

static float s_meanErrorPressure = 0.0f;
static int s_meanCount = 0;
static bool s_triggerBrewEnd = false;

static int s_paramInitCount = 8;
static float s_pidSetpointPressure = 0.0f;
static float s_pidSetpointFlowRate = 0.0f;

static void updatePidCoefficients();

// Configuration parameters to expose to MQTT
MqttParam::Parameter<float> param_bar_kp("pid/bar/kp", Defaults::Kp,            [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_bar_ki("pid/bar/ki", Defaults::Ki,            [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_bar_kd("pid/bar/kd", Defaults::Kd,            [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_bar_po("pid/bar/po", Defaults::PlantOffset,   [] (float val) { updatePidCoefficients(); });

MqttParam::Parameter<float> param_flow_kp("pid/flow/kp", Defaults::Kp,            [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_flow_ki("pid/flow/ki", Defaults::Ki,            [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_flow_kd("pid/flow/kd", Defaults::Kd,            [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_flow_po("pid/flow/po", 0.0f,                    [] (float val) { updatePidCoefficients(); });

MqttParam::Parameter<float> param_brewPressure("brew/pressure", CONFIG_TARGET_BREW_PRESSURE, [] (float val) { 
    if (s_brewMode == BrewMode::ManualPressure) setPressure(val);
});
MqttParam::Parameter<float> param_brewFlowRate("brew/flow",     1.0f, [] (float val) {
    if (s_brewMode == BrewMode::ManualFlow) setFlowRate(val);
});

static ProfileDefs::Profile s_dynamicProfile;
static int s_profileStageIndex = 0;
static int s_profileStageLastIndex = -1;

void initControlLoop()
{
    pid.reset();

    // Output between 0-100% duty cycle of pump
    pid.setOutputLimits(0.0f, 1.0f);

    s_pidSetpointPressure = param_brewPressure.value();
    s_pidSetpointFlowRate = param_brewFlowRate.value();

    // pid.setParameters(Defaults::Kp, Defaults::Ki, Defaults::Kd);
    // pid.setSetpoint(CONFIG_TARGET_BREW_PRESSURE);
    // pid.setPlantOffset(Defaults::PlantOffset);
    pid.setRegulationRange(Defaults::RegulationRange);
    //pid.setSampleTime(Defaults::UpdatePeriodMs); // TODO: tunings were calculated with 1000ms

    pid.setDebugPrints(true);

    updatePidCoefficients();

    pid.setSetpoint((s_controlMode == ControlMode::Pressure) ? s_pidSetpointPressure : s_pidSetpointFlowRate);
}

static void updatePidCoefficients() {
    // Skip initial update of coeffs
    if (s_paramInitCount > 0) {
        s_paramInitCount--;
        return;
    }

    // TODO: We're probably going to want two PID controllers,
    // so that in flow-regulation mode we can still limit the maximum pressure to 10Bar or so.

    bool sel = (s_controlMode == ControlMode::Pressure);
    float kp = sel ? param_bar_kp.value() : param_flow_kp.value();
    float ki = sel ? param_bar_ki.value() : param_flow_ki.value();
    float kd = sel ? param_bar_kd.value() : param_flow_kd.value();
    float po = sel ? param_bar_po.value() : param_flow_po.value();

    pid.setParameters(kp, ki, kd);
    pid.setPlantOffset(po);
    
    pid.reset();

    Debug.printf("Brew PID Parameters:\n\tKp: %.4f\n\tKi: %.4f\n\tKd: %.4f\n\tOf: %.4f\n", 
        pid.getKp(),
        pid.getKi(),
        pid.getKd(),
        po
    );
}

static void setControlMode(ControlMode mode) {
    if (s_controlMode != mode) {
        s_controlMode = mode;

        switch (mode) {
            case ControlMode::Disabled:
                Debug.println("PID Regulation: Disabled");
                break;

            case ControlMode::Pressure:
                Debug.println("PID Regulation: Pressure");
                break;

            case ControlMode::FlowRate:
                Debug.println("PID Regulation: Flow Rate");
                break;
        }

        pid.reset();
        updatePidCoefficients();

        IO::setPump(false);
    }
}


#if false // Tuning
void publishTuningData(float pid_input, float pid_output) {
    float t_sec = millis() * 0.001f;
    char s[50];

    snprintf(s, sizeof(s), "%.1f,%.2f,%.1f", 
        t_sec,
        pid_input,
        pid_output
    );
    Debug.printf("Tuning: %s\n", s);
    HomeAssistant::publishData("lupa/tuning/pressure", s);
}

const float tuningPhaseSetpoint[] = {
    20.0f,
    40.0f,
    60.0f,
    80.0f,
    100.0f,
    0.0f
};

float calculateTuningTick(float pid_input) {
    static float output = 0.0f;
    static unsigned long tuning_interval_ms = 1*60*1000;
    static float last_input = 0.0f;
    static unsigned long t_last = 0;
    static int tuning_phase = 0;
    const int n_phases = sizeof(tuningPhaseSetpoint)/sizeof(tuningPhaseSetpoint[0]) * 2;

    if (t_last == 0) {
        t_last = millis();
        last_input = pid_input;
    }

    // // 100% if below setpoint, 0% if above, with 1C hysteresis
    // if (pid_input < (CONFIG_BOILER_TUNING_TEMPERATURE_C + 1.0f)) {
    //     pid_output = 100.0f;
    // }
    // else if (pid_input > (CONFIG_BOILER_TUNING_TEMPERATURE_C - 1.0f)) {
    //     pid_output = 0.0f;
    // }

    if ((millis() - t_last) > tuning_interval_ms) {
        t_last = millis();

        if (tuning_phase == n_phases) {
            output = 0.0f;
            Debug.println("[ TUNING DONE ]");
        }
        else {
            tuning_phase++;
            Debug.printf("[ TUNING PHASE: %d ]\n", tuning_phase);
            if ((tuning_phase & 1) == 0) {
                // odd numbers
                output = 0.0f; 
                tuning_interval_ms = 2*60*1000; // cool
            }
            else {
                // even numbers
                output = tuningPhaseSetpoint[tuning_phase >> 1];
                tuning_interval_ms = 2*60*1000; // heat
            }
            Debug.printf("Setpoint: %.1f\n", output);
            Debug.printf("Interval: %dms\n", tuning_interval_ms);
        }
    }

    return output;
}
#endif

void disableOutput() {
    IO::setPump(false);
    setControlMode(ControlMode::Disabled);
}

void endBrew() {
    disableOutput();
    s_triggerBrewEnd = true;
}

void setPressure(float sp) {
    Debug.printf("Target pressure: %.1f\n", sp);

    if (sp <= __FLT_EPSILON__) {
        disableOutput();
        return;
    }

    setControlMode(ControlMode::Pressure);
    pid.setSetpoint(sp);
    s_pidSetpointPressure = sp;
}

void setFlowRate(float sp) {
    Debug.printf("Target flowrate: %.1f\n", sp);

    if (sp <= __FLT_EPSILON__) {
        disableOutput();
        return;
    }

    setControlMode(ControlMode::FlowRate);
    pid.setSetpoint(sp);
    s_pidSetpointFlowRate = sp;
}


void processDynamicProfile(const ProfileDefs::State& state)
{
    const size_t numStages = s_dynamicProfile.size();
    if (s_profileStageIndex >= numStages) {
        return; // No more steps
    }

    Debug.printf("STATE: p=%.1f f=%.1f pt=%.1f ft=%.1f\n", 
        state.currPressure, state.currFlowRate, state.setPressure, state.setFlowRate);

    // Loop until no more stages can be advanced
    bool advance = false;
    do {
        auto& stage = s_dynamicProfile[s_profileStageIndex];

        if (s_profileStageLastIndex != s_profileStageIndex) {
            s_profileStageLastIndex = s_profileStageIndex;
            Debug.printf("Profile step %d/%d: ", s_profileStageIndex+1, s_dynamicProfile.size());
            std::visit([](auto& st) { st.print(); }, stage);
            std::visit([](auto& st) { st.activate(); }, stage);
        }

        // Call per PID iteration (~100ms)
        advance = std::visit([&](auto& st) { return st.step(state); }, stage);
        if (advance) {
            s_profileStageIndex++;
        }
    }
    while (advance && (s_profileStageIndex < numStages));
}

void processControlLoop()
{
    static unsigned long t_last = 0;
    static unsigned long t_last_pid = 0;
    static float pid_output = 0.0f;
    static int tuning_phase = 0;

    auto t_now = millis();

    if (s_run)
    {
        float in_pressure = SensorSampler::getPressure();
        float in_flowrate = SensorSampler::getFlowRate();
        float pid_input = (s_controlMode == ControlMode::Pressure) ? in_pressure : in_flowrate;

        // Dynamic pressure profiling
        if (s_brewMode == BrewMode::DynamicProfile) {
            ProfileDefs::State state {
                .timeElapsed = (uint32_t)((t_now - s_startTime) / 1000),
                .currPressure = in_pressure,
                .setPressure = s_pidSetpointPressure,
                .currFlowRate = in_flowrate,
                .setFlowRate = s_pidSetpointFlowRate,
            };
            processDynamicProfile(state);
        }

        if (s_controlMode == ControlMode::Disabled) {
            IO::setPump(false);
            s_inrange = false;
        }
        else if (s_controlMode == ControlMode::Pressure) {
            // It takes several seconds for the pre-infusion chamber to fill.
            // Make sure we get past this point before we start regulating with PID,
            // otherwise the integral term will windup and cause instability through the shot.
            if (pid_input > Defaults::BeginRegulationPressure) {
                s_inrange = true;
            }
            else if (pid_input < Defaults::EndRegulationPressure) {
                // Fallen outside the range of PID regulation, set pump to 100% duty
                IO::setPump(true);
                s_inrange = false;
            }
        } else {
            // Ignore for flowrate control
            s_inrange = true;
        }

        if (s_inrange) 
        {
            if ((t_now - t_last_pid) >= Defaults::UpdatePeriodMs) {
                t_last_pid = t_now;

                if (s_brewMode == BrewMode::TuningPressure) {
                    // TODO: Implement
                    //pid_output = calculateTuningTick(pid_input);
                    //publishTuningData(pid_input, pid_output);
                }
                else {
                    pid_output = pid.calculateTick(pid_input);

                    // Calculate mean error
                    float delta = abs(pid_input - pid_output);
                    s_meanErrorPressure += delta;
                    s_meanCount++;
                }

                //Debug.printf("PID: I=%.1f, S=%.1f, O=%.1f\n", pid_input, pid.getSetpoint(), pid_output);

                IO::setPumpDuty(pid_output);
            }
        }
    }
}

void setMode(BrewMode profile) {
    s_brewMode = profile;

    Debug.print("Brew mode: ");
    switch (profile) {
        case BrewMode::TuningPressure:
            Debug.println("Tuning Pressure PID");
            s_controlMode = ControlMode::Pressure;
            break;
        case BrewMode::TuningFlow:
            Debug.println("Tuning Flow Rate PID");
            s_controlMode = ControlMode::FlowRate;
            break;
        case BrewMode::ManualPressure:
            Debug.println("Constant Pressure");
            setPressure(param_brewPressure.value());
            break;
        case BrewMode::ManualFlow:
            Debug.println("Constant Flow Rate");
            setFlowRate(param_brewFlowRate.value());
            break;
        case BrewMode::DynamicProfile:
            Debug.println("Dynamic Profile");
            break;
    }
}

bool setProfile(std::string profileName) {
    // TODO: Manual should really just be 100% pump power with mechanical pressure regulation
    if (profileName == "Manual") {
        setMode(BrewMode::ManualPressure);
        setPressure(CONFIG_TARGET_BREW_PRESSURE);
        return true;
    }
    else if (profileName == "Fixed Pressure") {
        setMode(BrewMode::ManualPressure);
        return true;
    }
    else if (profileName == "Fixed Flow Rate") {
        setMode(BrewMode::ManualFlow);
        return true;
    }

    for (auto& item : s_profiles) {
        auto name    = std::get<0>(item);
        auto profile = std::get<1>(item);
        if (name == profileName) {
            setMode(BrewMode::DynamicProfile);
            Debug.printf("Brew Profile: %s\n", profileName.c_str());
            s_dynamicProfile = profile;
            return true;
        }
    }

    Debug.printf("ERROR: Profile '%s' not known\n", profileName);
    return false;
}

void start() {
    // Start PID control loop
    s_run = true;
    s_inrange = false; // reset
    s_meanErrorPressure = 0.0f;
    s_meanCount = 0;
    s_profileStageIndex = 0;
    s_profileStageLastIndex = -1;
    s_startTime = millis();
    s_triggerBrewEnd = false;

    // Immediately turn on pump for responsiveness
    // PID will take over when it gets to it
    IO::setPump(true);

    if (s_brewMode == BrewMode::DynamicProfile) {
        ProfileDefs::State state {
            .timeElapsed = 0,
            .currPressure = 0,
            .setPressure = s_pidSetpointPressure,
            .currFlowRate = 0,
            .setFlowRate = s_pidSetpointFlowRate,
        };
        processDynamicProfile(state);
    }

    Debug.println("Start brew profile");
}

void stop() {
    // Stop PID control loop
    s_run = false;
    s_inrange = false;

    // Immediately turn off pump for responsiveness
    IO::setPump(false);

    Debug.println("Stop brew profile");

    if (s_meanCount > 0) {
        s_meanErrorPressure /= s_meanCount;
        Debug.printf("Mean Error: %.1f\n", s_meanErrorPressure);
    }
}

bool isProfileComplete() {
    return s_triggerBrewEnd;
}

BrewMode getMode() {
    return s_brewMode;
}

std::string getProfileString() {
    // switch (s_brewMode) {
    //     case BrewMode::ManualPressure:
    //         return "Pressure";
    //     case BrewMode::ManualFlow:
    //         return "Flow Rate";
    //     case BrewMode::TuningPressure:
    //         return "Tuning: Pressure";
    //     case BrewMode::TuningFlow:
    //         return "Tuning: Flow";
    //     case BrewMode::DynamicProfile:
    //         return "Dynamic"; // TODO: name of dynamic profile
    // }
    // return "";
    return "";
}

std::vector<std::string> getAvailableProfiles() {
    std::vector<std::string> profileNames;
    for (auto& el : s_profiles) {
        profileNames.push_back(std::get<0>(el));
    }
    return profileNames;
}

float getTargetError() {
    return s_meanErrorPressure / s_meanCount;
}

}