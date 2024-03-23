#include <Arduino.h>
#include "SensorSampler.h"
#include "HeatControl.h"
#include "HomeAssistant.h"
#include "PID.h"
#include "IO.h"
#include "hardware.h"
#include "config.h"

// static double pid_input = 0.0;
// static double pid_output = 0.0;
// static double pid_last_output = 0.0;
// static double pid_setpoint = CONFIG_BOILER_TEMPERATURE_C;
static float pid_setpoint = CONFIG_BOILER_TEMPERATURE_C;

static const float Kp = 10.0f;
static const float Ki = Kp / 13.0f; // Ki = Kp / Tn
static const float Kd = Kp * 5.0f; // Kd = Kp * Tv

// TODO: May need to incorporate another temperature probe for steam water,
// as currently the control loop doesn't respond to a drop in main boiler temperature
// until far too late. 
// Or, we could use a second pressure transducer which would respond very quickly, 
// and also let us detect whether steam is being used.

static fPID pid;

static unsigned long t_start = 0;
static unsigned long t_width = 0;

const float PID_OUTPUT_MAX = 100.0f;
const float PID_OUTPUT_MIN = 0.0f;
const unsigned long HEATER_MIN_PERIOD = 100;
const unsigned long HEATER_PERIOD = 5000;
const unsigned long PID_PERIOD = 1000;

const float MAX_BOILER_TEMPERATURE = CONFIG_MAX_BOILER_TEMPERATURE_C;

namespace HeatControl {

static BoilerProfile operating_profile = BoilerProfile::Off;

void initControlLoop()
{
    pid.setOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    pid.setParameters(Kp, Ki, Kd);
    pid.setSetpoint(pid_setpoint);
    pid.reset();

    Serial.printf("Heater PID Parameters:\n\tKp: %.2f\n\tKi: %.2f\n\tKd: %.2f\n", 
        pid.getKp(),
        pid.getKi(),
        pid.getKd()
    );

    // TODO: Prevent integral from going below 0, as it can
    // take a LOT longer to cool down than it will to warm up,
    // so the integral term will work against us.

    setProfile(BoilerProfile::Brew);
}

void publishTuningData(float pid_input, float pid_output) {
    float t_sec = millis() * 0.001f;
    char s[50];

    snprintf(s, sizeof(s), "%.1f,%.2f,%.1f", 
        t_sec,
        pid_input,
        pid_output
    );
    Serial.printf("Tuning: %s\n", s);
    HomeAssistant::publishData("lupa/tuning/boiler", s);
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

void processControlLoop()
{
    // TODO: Watchdog for safety.

    static unsigned long t_last = 0;
    static unsigned long t_last_pid = 0;
    static float pid_output = 0.0f;
    static int tuning_phase = 0;

    if (!SensorSampler::isTemperatureValid() || 
        IO::isWaterTankLow() || 
        IO::isBoilerTankLow() ||
        (operating_profile == BoilerProfile::Off))
    {
        IO::setHeat(false);
        IO::setHeatPower(0.0f);
    }
    else {
        float pid_input = SensorSampler::getTemperature();

        if (pid_input > MAX_BOILER_TEMPERATURE) {
            IO::setHeat(false);
            IO::setHeatPower(0.0f);
            return;
        }

        // Not yet near the pid_setpoint, 100% duty until we get close
        if (pid_input < (pid_setpoint - CONFIG_BOILER_PID_RANGE_C)) {
            if (IO::getHeatPower() < 1.0f) {
                Serial.printf("PREHEAT: %.1f\n", pid_input);
            }
            IO::setHeat(true);
            IO::setHeatPower(1.0f);
            return;
        }
        else {

            // Override PID when water is flowing
            float offset = 0.0f;
            if (SensorSampler::isFlowRateValid()) {
                auto flow = SensorSampler::getFlowRate();
                if (flow > 1.0f) {
                    // Perturb the PID controller error to predict 
                    // the fact that the temperature will start to decrease soon.
                    //offset = 20.0f;
                }
            }
            pid.setPerturbationOffset(offset);

            auto t_now = millis();
            if ((t_now - t_last_pid) >= PID_PERIOD) {
                t_last_pid = t_now;

                if (operating_profile == BoilerProfile::Tuning) {
                    pid_output = calculateTuningTick(pid_input);
                    publishTuningData(pid_input, pid_output);
                }
                else {
                    pid_output = pid.calculateTick(pid_input);
                }

                //Serial.printf("PID: I=%.1f, S=%.1f, O=%.1f\n", pid_input, pid_setpoint, pid_output);

                if (pid_output > 0.0) {
                    if (t_start == 0) {
                        t_width = (unsigned long)(pid_output * (float)HEATER_PERIOD * 0.01f);
                        if (t_width < HEATER_MIN_PERIOD) {
                            t_width = 0;
                            IO::setHeatPower(0.0);
                            IO::setHeat(false);
                        }
                        else {
                            IO::setHeatPower(pid_output * 0.01f);
                        }
                        
                    }
                }
                else {
                    IO::setHeatPower(0.0);
                    IO::setHeat(false);
                }
            }

            // Turn on heater every period, if non-zero
            t_now = millis();
            if ((t_now - t_last) >= HEATER_PERIOD) {
                t_last = t_now;
                if ((t_width >= HEATER_MIN_PERIOD) && (t_start == 0)) {
                    t_start = t_now;
                    Serial.printf("Heat: %d/%d\n", t_width, HEATER_PERIOD);
                    IO::setHeat(true);
                }
            }

            // Turn off heater after defined period
            if ((t_start > 0) && ((t_now - t_start) > t_width)) {
                // Keep heater on if at 100% duty (only turn off if width is less than max period)
                if (t_width < (HEATER_PERIOD - HEATER_MIN_PERIOD)) {
                    IO::setHeat(false);
                }

                t_width = 0;
                t_start = 0;
            }

        }

    }
}

void setProfile(BoilerProfile mode) {
    //mode = BoilerProfile::Off; // What the fuck? This causes the SensorSampler timer to crash???
    operating_profile = mode;

    Serial.print("Boiler heat profile: ");
    switch (mode) {
        case BoilerProfile::Off:
            //pid_setpoint = 0.0f;
            // TODO: Reset/Disable PID controller
            Serial.println("Off");
            break;
        case BoilerProfile::Brew:
            Serial.println("Brew");
            pid_setpoint = CONFIG_BOILER_TEMPERATURE_C;
            break;
        case BoilerProfile::Steam:
            Serial.println("Steam");
            pid_setpoint = CONFIG_BOILER_STEAM_TEMPERATURE_C;
            break;
        case BoilerProfile::Idle:
            Serial.println("Idle");
            pid_setpoint = CONFIG_BOILER_IDLE_TEMPERATURE_C;
            break;
        case BoilerProfile::Tuning:
            Serial.println("Tuning");
            pid_setpoint = CONFIG_BOILER_TUNING_TEMPERATURE_C;
            break;
    }

    pid.setSetpoint(pid_setpoint);
}

BoilerProfile getProfile() {
    return operating_profile;
}

}