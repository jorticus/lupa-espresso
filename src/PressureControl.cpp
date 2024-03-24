#include <Arduino.h>
#include "SensorSampler.h"
#include "PressureControl.h"
#include "MqttParamManager.h"
//#include "HomeAssistant.h"
#include "PID.h"
#include "IO.h"
#include "hardware.h"
#include "config.h"


// Default tunings
namespace Defaults {
    // How often PID is calculated
    // NOTE: If this is changed, the coefficients will need to be updated too
    static const unsigned long UpdatePeriodMs = 100;

    static const float Kp = 0.1f;
    static const float Ki = 0.01f; //Kp / 13.0f; // Ki = Kp / Tn
    static const float Kd = 0.0f;  //Kp * 5.0f; // Kd = Kp * Tv
    
    static const float SetPoint = CONFIG_TARGET_BREW_PRESSURE;

    // Static offset needed to reach steady-state, determined empirically
    // Helps prevent integral windup
    static const float PlantOffset = 0.660f;

    // Only apply integral when within this range of setpoint,
    // to avoid integral windup during initial ramp up of pressure
    static const float RegulationRange = 2.0f;

    // Pressure which we must reach before switching on PID regulation
    static const float BeginRegulationPressure = 3.0f;
    static const float EndRegulationPressure = 1.0f;
};

static fPID pid;

static bool s_run = false;
static bool s_inrange = false;

namespace PressureControl {

static PressureProfile operating_profile = PressureProfile::Manual;

void updatePidCoefficients();

// Configuration parameters to expose to MQTT
MqttParam::Parameter<float> param_sp("pid/bar/sp", Defaults::SetPoint,      [] (float val) { setPressure(val); });
MqttParam::Parameter<float> param_kp("pid/bar/kp", Defaults::Kp,            [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_ki("pid/bar/ki", Defaults::Ki,            [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_kd("pid/bar/kd", Defaults::Kd,            [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_po("pid/bar/po", Defaults::PlantOffset,   [] (float val) { updatePidCoefficients(); });

// Manual control of pump, for testing
//MqttParam::Parameter<float> pump_duty("pump", [] (float val) { IO::setPumpDuty(val); });

void initControlLoop()
{
    // Output between 0-100% duty cycle of pump
    pid.setOutputLimits(0.0f, 1.0f);

    pid.setParameters(Defaults::Kp, Defaults::Ki, Defaults::Kd);

    pid.reset();

    pid.setSetpoint(Defaults::SetPoint);
    pid.setPlantOffset(Defaults::PlantOffset);
    pid.setRegulationRange(Defaults::RegulationRange);

    updatePidCoefficients();
}

void updatePidCoefficients() {
    float kp = param_kp.value();
    float ki = param_ki.value();
    float kd = param_kd.value();
    float po = param_po.value();

    pid.setParameters(kp, ki, kd);

    pid.reset();
    pid.setPlantOffset(po);

    Serial.printf("Pressure PID Parameters:\n\tKp: %.4f\n\tKi: %.4f\n\tKd: %.4f\n\tOf: %.4f\n", 
        pid.getKp(),
        pid.getKi(),
        pid.getKd(),
        po
    );
}


#if false
void publishTuningData(float pid_input, float pid_output) {
    float t_sec = millis() * 0.001f;
    char s[50];

    snprintf(s, sizeof(s), "%.1f,%.2f,%.1f", 
        t_sec,
        pid_input,
        pid_output
    );
    Serial.printf("Tuning: %s\n", s);
    HomeAssistant::publishData("lupa/tuning/pressure", s);
}
#endif

#if false
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
            Serial.println("[ TUNING DONE ]");
        }
        else {
            tuning_phase++;
            Serial.printf("[ TUNING PHASE: %d ]\n", tuning_phase);
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
            Serial.printf("Setpoint: %.1f\n", output);
            Serial.printf("Interval: %dms\n", tuning_interval_ms);
        }
    }

    return output;
}
#endif

void setPressure(float sp) {
    pid.setSetpoint(sp);
    param_sp.set(sp);
    Serial.printf("Target pressure: %.1f\n", sp);
}

void processControlLoop()
{
    static unsigned long t_last = 0;
    static unsigned long t_last_pid = 0;
    static float pid_output = 0.0f;
    static int tuning_phase = 0;

    if (s_run)
    {
        float pid_input = SensorSampler::getPressure();

        // It takes several seconds for the pre-infusion chamber to fill.
        // Make sure we get past this point before we start regulating with PID,
        // otherwise the integral term will windup and cause instability through the shot.
        if (pid_input > Defaults::BeginRegulationPressure) {
            s_inrange = true;
        }
        else if (pid_input < Defaults::EndRegulationPressure) {
            IO::setPump(true);
            s_inrange = false;
        }

        if (s_inrange) 
        {
            auto t_now = millis();
            if ((t_now - t_last_pid) >= Defaults::UpdatePeriodMs) {
                t_last_pid = t_now;

                if (operating_profile == PressureProfile::Tuning) {
                    // TODO: Implement
                    //pid_output = calculateTuningTick(pid_input);
                    //publishTuningData(pid_input, pid_output);
                }
                else {
                    pid_output = pid.calculateTick(pid_input);
                }

                //Serial.printf("PID: I=%.1f, S=%.1f, O=%.1f\n", pid_input, pid.getSetpoint(), pid_output);

                IO::setPumpDuty(pid_output);
            }
        }
    }
}

void setProfile(PressureProfile mode) {
    operating_profile = mode;

    Serial.print("Pressure profile: ");
    switch (mode) {
        case PressureProfile::Tuning:
            Serial.println("Tuning");
            //pid_setpoint = CONFIG_BOILER_TUNING_TEMPERATURE_C;
            break;
        case PressureProfile::Manual:
            Serial.println("Manual: Constant Pressure");
            //pid_setpoint = CONFIG_BOILER_TEMPERATURE_C;
            break;
        case PressureProfile::AutoConstant:
            Serial.println("Auto: Constant Pressure");
    }

    pid.setSetpoint(param_sp.value());
}

void start() {
    // TODO: For profiles that evolve over time, this should start them

    // Start PID control loop
    s_run = true;
    s_inrange = false; // reset

    // Immediately turn on pump for responsiveness
    // PID will take over when it gets to it
    IO::setPump(true);

    Serial.println("Start pressure profile");
}

void stop() {
    // Stop PID control loop
    s_run = false;
    s_inrange = false;

    // Immediately turn off pump for responsiveness
    IO::setPump(false);

    Serial.println("Stop pressure profile");
}

bool isProfileComplete() {
    // TODO...
    // if (operating_profile == PressureProfile::Manual) {
    //     return false;
    // }
    return false;
}

PressureProfile getProfile() {
    return operating_profile;
}

}