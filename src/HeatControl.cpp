#include <Arduino.h>
#include <PID_v1.h>
#include "SensorSampler.h"
#include "HeatControl.h"
#include "Machine.h"
#include "hardware.h"
#include "config.h"

static double pid_input = 0.0;
static double pid_output = 0.0;
static double pid_setpoint = CONFIG_BOILER_TEMPERATURE_C;

// Defaults from smartcoffee project,
// seem to work okay with my machine.
// https://github.com/rancilio-pid/clevercoffee/blob/4c8ce515c5360aac4235b064c044bd739356de09/src/defaults.h
static const double Kp = 45.0;
static const double Ki = 45.0 / 130.0; // Ki = Kp / Tn
static const double Kd = 0;

// TODO: Internally this uses doubles, should really use floats
static PID pid(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);

static unsigned long t_start = 0;
static unsigned long t_width = 0;

const float PID_OUTPUT_MAX = 100.0;
const float PID_OUTPUT_MIN = 0.0;
const unsigned long HEATER_MIN_PERIOD = 100;
const unsigned long HEATER_MAX_PERIOD = 5000;
const unsigned long HEATER_PERIOD = 5000;

const float MAX_BOILER_TEMPERATURE = CONFIG_MAX_BOILER_TEMPERATURE_C;

namespace HeatControl {

void initControlLoop()
{
    pid.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    pid.SetMode(AUTOMATIC);

    Serial.printf("PID Parameters:\n\tKp: %.2f\n\tKi: %.2f\n\tKd: %.2f\n", 
        pid.GetKp(),
        pid.GetKi(),
        pid.GetKd()
    );

    // TODO: Prevent integral from going below 0, as it can
    // take a LOT longer to cool down than it will to warm up,
    // so the integral term will work against us.
}

void processControlLoop()
{
    // TODO: Watchdog for safety.

    static unsigned long t_last = 0;

    // TODO: Scheme should operate in 3 modes:
    // 1. Preheat : Heat 100% on until reach defined temperature
    // 2. Regulate : PID control loop to keep within target
    // 3. Steam : Heat 100% while pressure is low to maintain steam

    if (!SensorSampler::isTemperatureValid() || Machine::isWaterTankLow()) {
        // Turn off heat/pump/etc if we don't have a temperature reading
        Machine::failsafe();
    }
    else {
        pid_input = SensorSampler::getTemperature();

        if (pid_input > MAX_BOILER_TEMPERATURE) {
            Machine::setHeat(false);
            Machine::setHeatPower(0.0);
            return;
        }

        // Not yet near the pid_setpoint, 100% duty until we get close
        if (pid_input < (pid_setpoint - 10.0)) {
            Machine::setHeat(true);
            Machine::setHeatPower(1.0);
            return;
        }
        else {

            if (pid.Compute()) {
                Serial.printf("PID Target: %.1f, %.1f, %.1f\n", pid_input, pid_output, pid_setpoint);
                if (pid_output > 0.0) {
                    Machine::setHeatPower(pid_output * 0.01f);
                    t_width = (unsigned long)(pid_output * (double)HEATER_PERIOD * 0.01);

                    // if (t_width > (HEATER_PERIOD - HEATER_MIN_PERIOD)) {
                    //     t_width = (HEATER_PERIOD - HEATER_MIN_PERIOD;
                    // }
                }
                else {
                    Machine::setHeatPower(0.0);
                }
            }

            auto t_now = millis();

            // Turn on heater every period, if non-zero
            if ((t_now - t_last) >= HEATER_PERIOD) {
                t_last = t_now;
                if ((t_width > HEATER_MIN_PERIOD) && (t_start == 0)) {
                    t_start = t_now;
                    Serial.printf("Heat: %d/%d\n", t_width, HEATER_PERIOD);
                    Machine::setHeat(true);
                }
            }

            // Turn off heater after defined period
            if ((t_start > 0) && ((t_now - t_start) > t_width)) {
                // Keep heater on if at 100% duty
                if (t_width < (HEATER_PERIOD - HEATER_MIN_PERIOD)) {
                    Machine::setHeat(false);
                }

                t_width = 0;
                t_start = 0;
            }

        }

    }
}

}