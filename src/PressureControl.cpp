#include <Arduino.h>
#include "SensorSampler.h"
#include "PressureControl.h"
//#include "HomeAssistant.h"
#include "PID.h"
#include "IO.h"
#include "hardware.h"
#include "config.h"

static bool s_run = false;

static float pid_setpoint = 2.0f; //CONFIG_TARGET_BREW_PRESSURE;

// TODO: These will need adjusting
static const float Kp = 1.0f;
static const float Ki = 0.0f; //Kp / 13.0f; // Ki = Kp / Tn
static const float Kd = 0.0f; //Kp * 5.0f; // Kd = Kp * Tv

static fPID pid;

static unsigned long t_start = 0;
static unsigned long t_width = 0;

const unsigned long OUTPUT_MIN_PERIOD = 20;     // 1 cycle of 50Hz period
const unsigned long OUTPUT_PERIOD = 500;        // 50Hz * 500ms = 25 cycles
const unsigned long PID_UPDATE_PERIOD = 500;    // how often PID is calculated (ms)

//const float MAX_BOILER_TEMPERATURE = CONFIG_MAX_BOILER_TEMPERATURE_C;

namespace PressureControl {

static PressureProfile operating_profile = PressureProfile::Manual;

void initControlLoop()
{
    pid.setOutputLimits(0.0f, 1.0f);
    pid.setParameters(Kp, Ki, Kd);
    pid.setSetpoint(pid_setpoint);
    pid.reset();

    Serial.printf("Pressure PID Parameters:\n\tKp: %.2f\n\tKi: %.2f\n\tKd: %.2f\n", 
        pid.getKp(),
        pid.getKi(),
        pid.getKd()
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

void processControlLoop()
{
    static unsigned long t_last = 0;
    static unsigned long t_last_pid = 0;
    static float pid_output = 0.0f;
    static int tuning_phase = 0;

    //if (IO::isLeverPulled() && (operating_profile != PressureProfile::Off))
    if (s_run)
    {
        float pid_input = SensorSampler::getPressure();

        // // Not yet near the pid_setpoint, 100% duty until we get close
        // if (pid_input < (pid_setpoint - CONFIG_BOILER_PID_RANGE_C)) {
        //     IO::setPump(true);
        //     return;
        // }
        // else 
        {
            auto t_now = millis();
            if ((t_now - t_last_pid) >= PID_UPDATE_PERIOD) {
                t_last_pid = t_now;

                if (operating_profile == PressureProfile::Tuning) {
                    // TODO: Implement
                    //pid_output = calculateTuningTick(pid_input);
                    //publishTuningData(pid_input, pid_output);
                }
                else {
                    pid_output = pid.calculateTick(pid_input);
                }

                Serial.printf("PID: I=%.1f, S=%.1f, O=%.1f\n", pid_input, pid_setpoint, pid_output);

                if (pid_output > 0.0) {
                    if (t_start == 0) {
                        t_width = (unsigned long)(pid_output * (float)OUTPUT_PERIOD);
                        if (t_width < OUTPUT_MIN_PERIOD) {
                            t_width = 0;
                            IO::setPump(false);
                        }
                        else {
                            //IO::setHeatPower(pid_output);
                        }
                        
                    }
                }
                else {
                    IO::setPump(false);
                }
            }

            // Turn on pump every period, if non-zero
            t_now = millis();
            if ((t_now - t_last) >= OUTPUT_PERIOD) {
                t_last = t_now;
                if ((t_width >= OUTPUT_MIN_PERIOD) && (t_start == 0)) {
                    t_start = t_now;
                    Serial.printf("Pressure: %d/%d\n", t_width, OUTPUT_PERIOD);
                    IO::setPump(true);
                }
            }

            // Turn off pump after defined period
            if ((t_start > 0) && ((t_now - t_start) > t_width)) {
                // Keep pump on if at 100% duty (only turn off if width is less than max period)
                if (t_width < (OUTPUT_PERIOD - OUTPUT_MIN_PERIOD)) {
                    IO::setPump(false);
                }

                t_width = 0;
                t_start = 0;
            }

        }

    }
}

void setProfile(PressureProfile mode) {
    operating_profile = mode;

    Serial.print("Pressure profile: ");
    switch (mode) {
        // case PressureProfile::Off:
        //     Serial.println("Off");
        //     break;
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

    pid.setSetpoint(pid_setpoint);
}

void start() {
    // TODO: For profiles that evolve over time, this should start them
    s_run = true;
    Serial.println("Start pressure profile");
}

void stop() {
    s_run = false;
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