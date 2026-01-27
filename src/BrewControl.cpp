#include <Arduino.h>
#include "SensorSampler.h"
#include "BrewControl.h"
#include "BrewProfiles.h"
#include "MqttParamManager.h"
#include "IO.h"
#include "HeatControl.h"
#include "Debug.h"
#include "Task.h"
#include "hardware.h"
#include "config.h"

#include <span>
#include <array>

/// @brief Preinfusion delay
/// Don't run the control loop until preinfusion has completed
unsigned long t_shot_preinfuse_ms = 3500;


namespace BrewControl {

enum class ControlMode {
    Disabled,
    Pressure,
    FlowRate
};

static TaskHandle_t controlTaskHandle;

static BrewMode s_brewMode = BrewMode::TuningPressure;
static ControlMode s_controlMode = ControlMode::Pressure;

static uint64_t s_startTime = 0;

static bool s_run = false;
static float s_meanErrorPressure = 0.0f;
static int s_meanCount = 0;
static bool s_triggerBrewEnd = false;

static float s_setpointPressure = 0.0f;
static float s_setpointFlowRate = 0.0f;

static BrewMetrics s_metrics;

MqttParam::Parameter<float> param_brewPressure("brew/pressure", CONFIG_TARGET_BREW_PRESSURE, [] (float val) { 
    s_setpointPressure = val;
    if (s_brewMode == BrewMode::ManualPressure) setPressure(val);
});
MqttParam::Parameter<float> param_brewFlowRate("brew/flow",     1.0f, [] (float val) {
    s_setpointFlowRate = val;
    if (s_brewMode == BrewMode::ManualFlow) setFlowRate(val);
});

static std::string s_dynamicProfileName;
static ProfileDefs::Profile s_dynamicProfile;
static int s_profileStageIndex = 0;
static int s_profileStageLastIndex = -1;

static TaskHandle_t controlLoopTaskHandle;

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

        IO::setPump(false);
    }
}


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
    s_setpointPressure = sp;
}

void setFlowRate(float sp) {
    Debug.printf("Target flowrate: %.1f\n", sp);

    if (sp <= __FLT_EPSILON__) {
        disableOutput();
        return;
    }

    setControlMode(ControlMode::FlowRate);
    s_setpointFlowRate = sp;
}


void processDynamicProfile(const ProfileDefs::State& state)
{
    const size_t numStages = s_dynamicProfile.size();
    if (s_profileStageIndex >= numStages) {
        return; // No more steps
    }

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

/// @brief Advanced pressure profiling control loop
class ProController {
public:

    //Adjustable controller coefficients

    /// @brief Maximum slew rate (Bar/s)
    /// Should be slower than the pressure sensor response time
    // float dP_max_slope = 30.0f;

    /// @brief Proportional gain
    /// Adjust until steady state achieved with no oscillation
    float Kp = 0.3f; // Pressure -> slope gain

    /// @brief Leak compensation forward gain (proportional to P)
    /// Adjust until pressure matches target
    float Kff = 0.1f;

    /// @brief Leak bias integrator gain (keep very small!)
    float Ki_trim = 0.004f; // Keep very small

private:

    // Timing
    const float dt = 0.100f; // 100ms
    const float inv_dt = 1.0f / dt;

    // Pressure limits
    const float P_min = 0.0f;
    const float P_max = 12.0f;

    // Actuator limits
    const float u_min = 0.1f;
    const float u_max = 1.0f;

    // Limits for Kff
    const float u_bias_min = 0.001f; // 0.001 x 10 Bar = +0.01 u_ff bias
    const float u_bias_max = 0.2f;   // 0.1 x 10 Bar   = +1.00 u_ff bias

    // Gating/Noise control
    const float P_deadband = 0.1f; // Bar
    const float dP_cmd_tol = 0.1f; // Bar/s

    // Heuristics
    const float P_regulation_range = 5.0f; // Bar
    const float dP_drop_max = -3.0f; // bar/s (Sudden drop) // TODO: May need adjusting
    const float collapse_hold_time = 0.3f; // s
    const float u_recover = 1.0f;

    // Filtering
    // const float time_constant = 0.01f; // s (Filter time constant)
    // const float cutoff_rads = (1.0 * exp(-dt / time_constant)) / dt;  // Discrete-time form of N=1/Tau
    const float tau = 0.5f; // 0.2-0.5s
    const float alpha = dt / (tau + dt);
    const float tau2 = 10.0f; // s
    const float alpha2 = dt / (tau + dt);

    // State
    float u_prev = 0.0f;
    float P_prev = 0.0f;
    float P_filt = 0.0f;
    float G_prev = 0.0f;
    float collapse_timer = 0.0f;
    float effort_filt = 0.0f;
    float G_filt = 0.0f;
    float dev_mean = 0.0f;
    float dev_mean_count = 0;

public:
    void reset()
    {
        u_prev = 0.0f;
        P_prev = 0.0f;
        P_filt = 0.0f;
        G_prev = 0.0f;
        G_filt = 0.0f;
        effort_filt = 0.0f;
        dev_mean = 0.0f;
        dev_mean_count = 0;

        collapse_timer = 0.0f;
    }

    /// @brief Pressure-profiling control loop
    /// @param P_cmd 
    /// @param P_meas 
    /// @return 
    float tick(float P_cmd, float P_meas, unsigned long t_delta)
    {
        // The grouphead acts like a leaky integrator, where applying a constant output will cause
        // the pressure to rise until it hits saturation, and zeroing output will allow pressure to collapse
        // as water escapes the puck.
        //
        // Trying to control this with a standard PID loop will cause problems since the two integrators
        // will be unstable. The following is a customized P-I loop with a very slow tuned I term.
        // The Derivative term is only used for calculating metrics.

        // ----------------------------
        // Timebase / Input
        // ----------------------------

        if (t_delta == 0) {
            return 0.0f;
        }

        float dt_s = (float)(t_delta) / 1000.0f;
        float inv_dt_s = 1.0f / dt_s;

        P_cmd = clamp(P_cmd, P_min, P_max);

        s_metrics.pressure_target = P_cmd;

        // Drive to 100% until we get close to the setpoint
        if (P_meas < (P_cmd - P_regulation_range)) {
            return 1.0f;
        }

        if (P_cmd <= CONFIG_MIN_PRESSURE) {
            return 0.0f;
        }

        // ----------------------------
        // Proportional control on pressure error
        // ----------------------------

        float eP = P_cmd - P_meas;

        // Deadband for noise
        if (fabs(eP) < P_deadband) {
            eP = 0.0f;
        }

        float u_p = Kp * eP;

        // ----------------------------
        // Derivative with LPF
        // Used for metrics only...
        // ----------------------------

        // First-order LPF
        P_filt = P_filt + alpha * (P_meas - P_filt);

        // Derivative
        float dP_filt = (P_filt - P_prev) * inv_dt_s; 
        P_prev = P_filt;

        // ----------------------------
        // Leak compensation & Slow bias trim
        //
        // We do still use an integrating term here to try to compensate for the leaky
        // behaviour, but it is designed to ramp slowly and not impact the loop itself 
        // too much, and is only updated when it is detected that we are holding a flat pressure.
        // ----------------------------

        bool flat_hold = (fabsf(dP_filt) < dP_cmd_tol * dt_s) && (u_p > u_min && u_p < u_max);
        if (flat_hold) {
            float adj = Ki_trim * eP * dt_s;
            Kff += adj;
            Kff = clamp(Kff, u_bias_min, u_bias_max);
            Debug.printf("Adj Kff: %.3f\n", adj);
        }

        float u_ff = Kff * P_cmd;

        // ----------------------------
        // Puck collapse detection & recovery
        // TODO... test
        // ----------------------------

        bool collapsed = detectPuckCollapse(dP_filt, dt_s);
        if (collapsed) {
            // Force pump towards 100% duty
            u_p = u_recover;
        }
        
        // ----------------------------
        // Output
        // PWM duty clamped to 0.0 - 1.0
        // ----------------------------

        float u_out = u_p + u_ff;

        u_out = clamp(u_out, u_min, u_max);

        // ----------------------------
        // Metrics
        // ----------------------------

        // Permeability
        // float perm = u_out / P_cmd;

        // Idea: collapse prevention using permeability detection:
        // if (d(perm)/dt > collapse_rate) {
        //   reduce P_cmd;
        // }

        // float G = estimatePuckConductance(P_meas, u_out, dP_filt);
        // G_filt = (alpha2 * G) + ((1 - alpha2) * G_filt);

        // TODO: We could use conductance estimate to determine when preinfusion has ended:
        // if (P_cmd > min && abs(dG_filt) < G_eps) {
        //     end_ramp();
        // }

        // Metric: Control output rate of change
        // May indicate stability of the shot
        float du = (u_out - u_prev) * inv_dt_s;
        u_prev = u_out;
        float effort_rms = fabsf(du); // (alpha2 * fabsf(du)) + ((1 - alpha2) * effort_rms);
        effort_filt = (alpha2 * effort_rms) + ((1 - alpha2) * effort_filt);

        // Metric: Estimated flow
        // flow in is proportional to u_out
        // flow out is proportional to d(P_measurement)
        float Q_est = u_out - dP_filt;
        
        // Metric: Conductance/Permeability
        // TBD: Since pressure is usually constant, this value will just be a constant version of flow.
        // If pressure or flow collapses, we expect this to spike.
        // float G = Q_est / P_meas;

        // Calculate mean error between requested pressure and actual pressure
        float delta = abs(P_cmd - P_meas);
        dev_mean += delta;
        dev_mean_count++;

        // TODO: Should make this concurrency safe
        s_metrics.deviation = (dev_mean / (float)dev_mean_count);
        s_metrics.effort = effort_filt;
        s_metrics.conductance = Q_est;
        s_metrics.collapsed = collapsed;
        s_metrics.stable = flat_hold;
        s_metrics.dP = dP_filt;

        // Debug.printf("LOOP: %.1fs P=%.1f P_cmd=%.3f dP_filt=%.3f e=%.3f u_p=%.2f u_ff=%.2f G=%.2f E=%.2f fh=%d\n", 
        //     dt_s, P_meas, P_cmd, dP_filt, eP, u_p, u_ff, G, effort_filt, flat_hold);

        return u_out;
    }

private:

    inline float clamp(float f, float min, float max) {
        if (f > max) return max;
        if (f < min) return min;
        return f;
    }

    bool detectPuckCollapse(float dP_filt, float dt_s) {
        // Puck collapse detection
        // We can detect if the puck has collapsed by detecting if the rate of change of pressure (dP)
        // has exceeded some threshold value for some specified amount of time.

        if (dP_filt < dP_drop_max) {
            collapse_timer += dt_s;
            if (collapse_timer > collapse_hold_time) {
                Debug.println("SHOT COLLAPSE");
                return true;
            }
        } else {
            collapse_timer = 0.0f;
        }
        return false;
    }

    float estimatePuckConductance(float P_meas, float u_out, float dP_filt) {

        // Estimate puck conductance using flow proxy
        // Flow can be estimated (though we cannot determine an absolute mL/s value),
        // and from that we can derive the relative resistance of the puck,
        // and use that to detect information about the puck itself.
        // This is not part of the control loop.

        const float k_est = 1.0f;
        float Q_est = k_est * u_out - dP_filt;

        float G = 0.0f;
        if (P_meas > 0.5f) {
            G = Q_est / P_meas;
            // G = clamp(G, 0.0f, 2.0f); // TODO
        }

        float dG = (G - G_prev) * inv_dt;
        G_prev = G;

        // TODO: ChatGPT says I should replace conductance with a relative score:
        // when abs(dP_filt) near zero, and regulation is active, and no collapse detected, 
        // then capture a baseline `G_baseline = EMA(G)`
        // Afterwards, compute relative integrity:
        // integrity = clamp(G_baseline / (G + eps)), 0.0, 2.0)
        // 1.0 == normal puck
        // <0.7 == channeling
        // <0.4 == likely collapse
        // >1.2 == puck swelling / choking

        return G; // TODO: return dG once filtered?
    }
};

static ProController controller;

MqttParam::Parameter<float> param_pro_kp("pid/pro/kp", controller.Kp,           [] (float val) { controller.Kp = val; });
MqttParam::Parameter<float> param_pro_ki("pid/pro/ki", controller.Ki_trim,      [] (float val) { controller.Ki_trim = val; });
MqttParam::Parameter<float> param_pro_kf("pid/pro/kf", controller.Kff,          [] (float val) { controller.Kff = val; });

void notifyTick() {
    xTaskNotifyGive(controlTaskHandle);
}

static void controlLoopTask(void* pv) {
    static unsigned long t_last = 0;
    while (true) {
        // TODO: This task is starving lower priority tasks from running (eg. IO, MQTT params)

        // Wait for signal from the SensorSampler
        // Task will be woken when a pressure sample is available (typically 100ms)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!s_run || !IO::isLeverPulled()) {
            continue;
        }

        unsigned long t_now = millis();
        unsigned long t_shot_time = (t_now - s_startTime);
        unsigned long dt = (t_now - t_last);
        t_last = t_now;

        // // Preinfusion
        // // Just apply 100% duty to give system some chance to stabilize...
        // if (t_shot <= t_shot_preinfuse_ms) {
        //     IO::setPump(true);
        //     controller.reset();
        //     continue;
        // }

        float in_pressure = SensorSampler::getPressureUnfiltered();
        // Debug.printf("TICK: %u : %.2f\n", t_shot, in_pressure);

        // Dynamic pressure profiling
        if (s_brewMode == BrewMode::DynamicProfile) {
            ProfileDefs::State state {
                .timeElapsedMs = t_shot_time,
                .dt_s = (float)dt * 0.001f,
                .currPressure = in_pressure,
                .setPressure = s_setpointPressure,
                .currFlowRate = 0,
                .setFlowRate = 0,
            };

            // May update s_setpointPressure via BrewControl::setPressure()
            processDynamicProfile(state);
        }

        float pid_output = controller.tick(s_setpointPressure, in_pressure, dt);

        // Check s_run again in case brew was stopped during tick calculation
        if (s_run) {
            IO::setPumpDuty(pid_output);
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
            HeatControl::setProfile(HeatControl::BoilerProfile::Off);
            // startTuning();
            break;
        case BrewMode::TuningFlow:
            Debug.println("Tuning Flow Rate PID");
            s_controlMode = ControlMode::FlowRate;
            HeatControl::setProfile(HeatControl::BoilerProfile::Off);
            // startTuning();
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
    if (profileName == "Manual") {
        setMode(BrewMode::ManualPressure);
        setPressure(CONFIG_TARGET_BREW_PRESSURE);
        s_dynamicProfileName = profileName;
        return true;
    }
    else if (profileName == "Fixed Pressure") {
        setMode(BrewMode::ManualPressure);
        s_dynamicProfileName = profileName;
        return true;
    }
    else if (profileName == "Fixed Flow Rate") {
        s_dynamicProfileName = profileName;
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
            s_dynamicProfileName = profileName;
            return true;
        }
    }

    if (profileName == "Tuning: Pressure") {
        setMode(BrewMode::TuningPressure);
        s_dynamicProfileName = profileName;
        return true;
    }
    else if (profileName == "Tuning: Flow Rate") {
        setMode(BrewMode::TuningFlow);
        s_dynamicProfileName = profileName;
        return true;
    }

    Debug.printf("ERROR: Profile '%s' not known\n", profileName);
    // Profile remains unchanged
    return false;
}

void start() {
    Debug.println("Start brew profile");

    // Start PID control loop
    s_run = true;
    s_meanErrorPressure = 0.0f;
    s_meanCount = 0;
    s_profileStageIndex = 0;
    s_profileStageLastIndex = -1;
    s_startTime = millis();
    s_triggerBrewEnd = false;

    s_metrics = { 0 };

    controller.reset();

    // Immediately turn on pump for responsiveness
    // PID will take over when it gets to it
    IO::setPump(true);

    if (s_brewMode == BrewMode::DynamicProfile) {
        ProfileDefs::State state {
            .timeElapsedMs = 0,
            .dt_s = 0,
            .currPressure = 0,
            .setPressure = s_setpointPressure,
            .currFlowRate = 0,
            .setFlowRate = s_setpointFlowRate,
        };
        processDynamicProfile(state);
    }

    // if (s_brewMode == BrewMode::TuningPressure) {
    //     startTuning();
    // }
}

void stop() {
    // Stop control loop
    s_run = false;

    // Immediately turn off pump for responsiveness
    IO::setPump(false);

    Debug.println("Stop brew profile");
}

bool isProfileComplete() {
    return s_triggerBrewEnd;
}

BrewMode getMode() {
    return s_brewMode;
}

std::string getProfileString() {

    switch (s_brewMode) {
        case BrewMode::ManualPressure:
            return "Manual: Pressure";
        case BrewMode::ManualFlow:
            return "Manual: Flow Rate";
        case BrewMode::TuningPressure:
            return "Tuning: Pressure";
        case BrewMode::TuningFlow:
            return "Tuning: Flow";
        default:
            return s_dynamicProfileName;
    }
}

std::vector<std::string> getAvailableProfiles() {
    std::vector<std::string> profileNames;
    for (auto& el : s_profiles) {
        profileNames.push_back(std::get<0>(el));
    }
    return profileNames;
}

// float getTargetError() {
//     return s_meanErrorPressure / s_meanCount;
// }

BrewMetrics getMetrics() {
    return s_metrics; // Copy
}

float getCurrentSetpoint() {
    return s_setpointPressure;
}


void initControlLoop()
{
    // Note: Task must be high priority since we must make the sampling intervals
    xTaskCreatePinnedToCore(controlLoopTask, "BrewControl", 3*1024, nullptr, TASK_PRIORITY_BREW_CONTROL, &controlTaskHandle, CORE1);

}

}