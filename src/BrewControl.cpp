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
unsigned long t_shot_preinfuse_ms = 4000;


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
    float dP_max_slope = 30.0f;

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


    const float u_bias_min = 0.0f; // -0.2f;
    const float u_bias_max = 1.0f; // 0.2f;

    // Gating/Noise control
    const float P_deadband = 0.1f; // Bar
    const float dP_ref_tol = 0.02f; // Bar/s

    // Heuristics
    const float P_regulation_range = 3.0f; // Bar
    const float P_shot_start = 2.0f; // Bar
    const float dP_drop_max = -3.0f; // bar/s (Sudden drop)
    const float collapse_hold_time = 0.3f; // s
    const float u_recover = 1.0f;
    const float u_bias_initial = 0.0f;

    // Filtering
    const float time_constant = 0.01f; // s (Filter time constant)
    const float cutoff_rads = (1.0 * exp(-dt / time_constant)) / dt;  // Discrete-time form of N=1/Tau

    // State
    unsigned long t_shot_start = 0;
    unsigned long t_shot_last = 0;
    float P_ref = 0.0f; // Internal pressure trajectory
    float u = 0.0f; // Control output
    float u_bias = u_bias_initial; // Slow trim
    float P_prev = 0.0f;
    float dP_filt = 0.0f;
    float G_prev = 0.0f;
    float collapse_timer = 0.0f;
    // bool shot_active = false;

public:
    void reset()
    {
        P_ref = 0.0f;
        u = 0.0f;
        P_prev = 0.0f;
        dP_filt = 0.0f;
        G_prev = 0.0f;
        t_shot_start = millis();
        t_shot_last = t_shot_start;

        collapse_timer = 0.0f;
    }

    /// @brief Pressure-profiling control loop
    /// @param P_cmd 
    /// @param P_meas 
    /// @return 
    float tick(float P_cmd, float P_meas)
    {
        // The grouphead acts like a leaky integrator, where applying a constant output will cause
        // the pressure to rise until it hits saturation, and zeroing output will allow pressure to collapse. 
        //
        // Trying to control this with a standard PID loop will cause problems since the two integrators will be unstable.
        // This can be mitigated by using a very low integrator term, but it's hard to get the loop to converge.
        //
        // Instead, we use a specialized control loop where we are controlling the rate of change 
        // of pressure (dP), which itself is controlled by a simple P term. 
        //

        // Timebase
        unsigned long t_now = millis();
        unsigned long t_delta = (t_now - t_shot_last);
        t_shot_last = t_now;

        float dt_s = (float)(t_delta) / 1000.0f;
        float inv_dt_s = 1.0f / dt_s;

        // Drive to 100% until we get close to the setpoint
        if (P_meas < (P_cmd - P_regulation_range)) {
            P_ref = P_meas;
            return 1.0f;
        }

        // Pressure trajectory (slew rate limited)
        // This updates the target rate of change of pressure (dP) to move towards the desired setpoint (P_cmd)

        float dP_cmd = P_cmd - P_ref;
        float max_step = dP_max_slope * dt_s;

        dP_cmd = clamp(dP_cmd, -max_step, max_step); // Slew rate limiter

        P_ref += dP_cmd; // Accumulate internal pressure reference

        P_ref = clamp(P_ref, P_min, P_max); // Limit pressure to range of sensor

        // Proportional control term

        float e = P_ref - P_meas;

        // Deadband for noise
        if (fabs(e) < P_deadband) {
            e = 0.0f;
        }

        u = Kp * e;

        float dP_meas = (P_meas - P_prev) * inv_dt_s; // [S] Derivative
        P_prev = P_meas;

        // const float tau = 0.05f;
        // const float alpha = dt / (tau + dt);
        // dP_filt = dP_filt + alpha * (dP_meas - dP_filt);
        dP_filt = dP_filt * dt_s * cutoff_rads * (dP_meas - dP_filt); // [N / (N + S)] first order LPF

        // dP_est = (P_meas - P_ref_prev) * inv_dt_s;
        // dP_filt = lowpass(dP_est) 

        if (detectPuckCollapse(dP_filt, dt_s)) {
        //     u = u_recover;
        //     u_bias = 0.0f;
        //     return u;
        }

        // Slow bias trim
        // We do still use an integrating term (u_bias) here to try to compensate for the leaky
        // behaviour, but it is designed to ramp slowly and not impact the loop itself too much.
        // The value of u_bias is updated whenever it is detected that we are holding a flat pressure,
        // since that is the only condition under which we can estimate the leakage.
        //
        // TOOD: Just adjust Kff since that is doing the same thing...

        bool flat_hold = (fabsf(dP_cmd) < dP_ref_tol * dt_s) && (u > u_min && u < u_max);
        if (flat_hold) {
            u_bias += Ki_trim * e * dt_s;
            u_bias = clamp(u_bias, u_bias_min, u_bias_max);
        }

        // Feed-forward leak estimation
        float u_ff = Kff * P_ref;

        // Output
        // PWM duty clamped to 0.0 - 1.0

        float u_out = u + u_ff + u_bias;

        u_out = clamp(u_out, u_min, u_max);

        float G = estimatePuckConductance(P_meas, u_out, dP_filt);

        unsigned long td = millis() - t_shot_last;
        Debug.printf("LOOP: %.1fs P=%.1f P_ref=%.3f dP_cmd=%.4f dP_filt=%.3f e=%.3f u=%.2f u_ff=%.2f u_bias=%.2f G=%.2f fh=%d td=%u\n", dt_s, P_meas, P_ref, dP_cmd, dP_filt, e, u, u_ff, u_bias, G, flat_hold, td);

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

        // if (dP_meas < dP_drop_max) {
        if (dP_filt < dP_drop_max) {
            collapse_timer += dt_s;
            if (collapse_timer > collapse_hold_time) {
                Debug.println("SHOT COLLAPSE");
                return true;
            }
        }
        collapse_timer = 0.0f;
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

        return G; // TODO: return dG once filtered?
    }
};

static ProController controller;

MqttParam::Parameter<float> param_pro_dp("pid/pro/dp", controller.dP_max_slope, [] (float val) { controller.dP_max_slope = val; });
MqttParam::Parameter<float> param_pro_kp("pid/pro/kp", controller.Kp,           [] (float val) { controller.Kp = val; });
MqttParam::Parameter<float> param_pro_ki("pid/pro/ki", controller.Ki_trim,      [] (float val) { controller.Ki_trim = val; });
MqttParam::Parameter<float> param_pro_kf("pid/pro/kf", controller.Kff,          [] (float val) { controller.Kff = val; });

void notifyTick() {
    xTaskNotifyGive(controlTaskHandle);
}

static void controlLoopTask(void* pv) {
    while (true) {
        // Wait for signal from the SensorSampler
        // Task will be woken when a pressure sample is available (typically 100ms)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!s_run) {
            continue;
        }

        unsigned long t_now = millis();
        unsigned long t_shot = (t_now - s_startTime);

        // Preinfusion
        // Just apply 100% duty to give system some chance to stabilize...
        if (t_shot <= t_shot_preinfuse_ms) {
            IO::setPump(true);
            continue;
        }

        float in_pressure = SensorSampler::getPressureUnfiltered();
        // Debug.printf("TICK: %u : %.2f\n", t_shot, in_pressure);

        // Dynamic pressure profiling
        if (s_brewMode == BrewMode::DynamicProfile) {
            ProfileDefs::State state {
                .timeElapsed = (uint32_t)(t_shot / 1000),
                .currPressure = in_pressure,
                .setPressure = s_setpointPressure,
                .currFlowRate = 0,
                .setFlowRate = 0,
            };

            // May update s_setpointPressure via BrewControl::setPressure()
            processDynamicProfile(state);
        }

        float pid_output = controller.tick(s_setpointPressure, in_pressure);
        IO::setPumpDuty(pid_output);

        // Calculate mean error
        float delta = abs(s_setpointPressure - pid_output);
        s_meanErrorPressure += delta;
        s_meanCount++;
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

    controller.reset();

    // Immediately turn on pump for responsiveness
    // PID will take over when it gets to it
    IO::setPump(true);

    if (s_brewMode == BrewMode::DynamicProfile) {
        ProfileDefs::State state {
            .timeElapsed = 0,
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

float getTargetError() {
    return s_meanErrorPressure / s_meanCount;
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