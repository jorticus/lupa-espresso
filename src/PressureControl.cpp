#include <Arduino.h>
#include "SensorSampler.h"
#include "PressureControl.h"
//#include "HomeAssistant.h"
#include "PID.h"
#include "IO.h"
#include "hardware.h"
#include "config.h"

static bool s_run = false;
static bool s_inrange = false;

static float pid_setpoint = CONFIG_TARGET_BREW_PRESSURE;

static const float Kp = 0.1f;
static const float Ki = 0.01f; //Kp / 13.0f; // Ki = Kp / Tn
static const float Kd = 0.0f;  //Kp * 5.0f; // Kd = Kp * Tv

static fPID pid;

static unsigned long t_start = 0;
static unsigned long t_width = 0;

const float OUTPUT_PID_REGULATION_RANGE = 2.0f;
const float POST_INFUSE_PRESSURE = 3.0f;
const float LOW_PRESSURE = 1.0f;
const float PLANT_OFFSET = 0.660f; // Static offset to reach steady-state, determined empirically

const unsigned long OUTPUT_MIN_PERIOD = 10;     // 1/2 cycle of 50Hz period
const unsigned long OUTPUT_PERIOD = 400;        // 
const unsigned long PID_UPDATE_PERIOD = 100;    // how often PID is calculated (ms)

//const float MAX_BOILER_TEMPERATURE = CONFIG_MAX_BOILER_TEMPERATURE_C;

namespace PressureControl {

static PressureProfile operating_profile = PressureProfile::Manual;

void initControlLoop()
{
    // Output between 0-100% duty cycle of pump
    pid.setOutputLimits(0.0f, 1.0f);

    pid.setParameters(Kp, Ki, Kd);

    pid.reset();
    pid.setSetpoint(pid_setpoint);

    // Static offset
    pid.setPlantOffset(PLANT_OFFSET);

    // Only apply integral when within this range of setpoint,
    // to avoid integral windup during initial ramp up of pressure
    pid.setRegulationRange(2.0f);

    Serial.printf("Pressure PID Parameters:\n\tKp: %.4f\n\tKi: %.4f\n\tKd: %.4f\n\tOf: %.4f\n", 
        pid.getKp(),
        pid.getKi(),
        pid.getKd(),
        PLANT_OFFSET
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
    pid_setpoint = sp;
    pid.setSetpoint(sp);
    Serial.printf("Target pressure: %.1f\n", sp);
}

void updateParameters(float Kp, float Ki, float Kd)
{
    pid.setParameters(Kp, Ki, Kd);

    pid.reset();
    pid.setSetpoint(pid_setpoint);

    Serial.printf("Pressure PID Parameters:\n\tKp: %.4f\n\tKi: %.4f\n\tKd: %.4f\n\tOf: %.4f\n", 
        pid.getKp(),
        pid.getKi(),
        pid.getKd(),
        PLANT_OFFSET
    );
}

void getParameters(float* p, float* i, float* d)
{
    *p = pid.getKp();
    *i = pid.getKi();
    *d = pid.getKd();
}

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

        // Not yet near the pid_setpoint, 100% duty until we get close
        //if (pid_input < (pid_setpoint - OUTPUT_PID_REGULATION_RANGE)) {

        // It takes several seconds for the pre-infusion chamber to fill.
        // Make sure we get past this point before we start regulating with PID,
        // otherwise the integral term will windup and cause instability through the shot.
        if (pid_input > POST_INFUSE_PRESSURE) {
            s_inrange = true;
        }
        else if (pid_input < LOW_PRESSURE) {
            IO::setPump(true);
            s_inrange = false;
        }

        if (s_inrange) 
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

                // Add static integral offset, determined empirically
                //pid_output += 0.82f;

                Serial.printf("PID: I=%.1f, S=%.1f, O=%.1f\n", pid_input, pid_setpoint, pid_output);

                IO::setPumpDuty(pid_output);


                // if (pid_output > 0.0) {
                //     IO::setPumpDuty(pid_output);
                //     // if (t_start == 0) {
                //     //     t_width = (unsigned long)(pid_output * (float)OUTPUT_PERIOD);
                //     //     if (t_width < OUTPUT_MIN_PERIOD) {
                //     //         t_width = 0;
                //     //         IO::setPump(false);
                //     //     }
                //     //     else {
                //     //         //IO::setHeatPower(pid_output);
                //     //     }
                        
                //     // }
                // }
                // else {
                //     IO::setPump(false);
                // }
            }

            // // Turn on pump every period, if non-zero
            // t_now = millis();
            // if ((t_now - t_last) >= OUTPUT_PERIOD) {
            //     t_last = t_now;
            //     if ((t_width >= OUTPUT_MIN_PERIOD) && (t_start == 0)) {
            //         t_start = t_now;
            //         Serial.printf("Pressure: %d/%d\n", t_width, OUTPUT_PERIOD);
            //         IO::setPump(true);

            //         // For very short pulses, we should delay here
            //         // since it may take too long before we get back here
            //         if (t_width < 60) {
            //             Serial.printf("Short delay: %d\n", t_width);
            //             delay(t_width);
            //             t_now += t_width;
            //             //IO::setPump(false);
            //         }
            //     }
            // }

            // // Turn off pump after defined period
            // if ((t_start > 0) && ((t_now - t_start) > t_width)) {
            //     // Keep pump on if at 100% duty (only turn off if width is less than max period)
            //     if (t_width < (OUTPUT_PERIOD - OUTPUT_MIN_PERIOD)) {
            //         IO::setPump(false);
            //     }

            //     t_width = 0;
            //     t_start = 0;
            // }

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
    s_inrange = false; // reset
    IO::setPump(true);
    Serial.println("Start pressure profile");
}

void stop() {
    s_run = false;
    s_inrange = false;
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