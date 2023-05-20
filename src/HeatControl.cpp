#include <Arduino.h>
#include <PID_v1.h>
#include "SensorSampler.h"
#include "hardware.h"

static double input = 0.0;
static double output = 0.0;
static double setpoint = 115.0;

// Defaults from smartcoffee project,
// seem to work okay with my machine.
static const double Kp = 45.0;
static const double Ki = 45.0 / 130.0; // Ki = Kp / Tn
static const double Kd = 0;

static PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

static unsigned long t_start = 0;
static unsigned long t_width = 0;

const float PID_OUTPUT_MAX = 100.0;
const float PID_OUTPUT_MIN = 0.0;
const unsigned long HEATER_MIN_PERIOD = 100;
const unsigned long HEATER_MAX_PERIOD = 5000;
const unsigned long HEATER_PERIOD = 5000;
const float MAX_BOILER_TEMPERATURE = 123.0f;

bool g_isHeaterOn = false;
float g_heatPower = 0.0;

// main.cpp
bool isWaterTankLow();
void failsafe();

/// @brief Initialize control loop parameters
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

/// @brief Set heater power immediately
/// @param en Heater power on/off
void setHeater(bool en) {
    static bool prev_value = LOW;

    if (en) {
        if (en != prev_value) {
            Serial.println("HEAT: ON");
        }
        digitalWrite(PIN_OUT_HEAT, HIGH);
    }
    else {
        if (en != prev_value) {
            Serial.println("HEAT: OFF");
        }
        digitalWrite(PIN_OUT_HEAT, LOW);
    }

    prev_value = en;
    g_isHeaterOn = en;
}

/// @brief Calculate next tick of the control loop
void processControlLoop()
{
    // TODO: Watchdog for safety.

    static unsigned long t_last = 0;

    // TODO: Scheme should operate in 3 modes:
    // 1. Preheat : Heat 100% on until reach defined temperature
    // 2. Regulate : PID control loop to keep within target
    // 3. Steam : Heat 100% while pressure is low to maintain steam

    if (!SensorSampler::isTemperatureValid() || isWaterTankLow()) {
        // Turn off heat/pump/etc if we don't have a temperature reading
        failsafe();
        g_heatPower = 0.0f;
    }
    else {
        input = SensorSampler::getTemperature();

        if (input > MAX_BOILER_TEMPERATURE) {
            setHeater(false);
            g_heatPower = 0.0f;
            return;
        }

        // Not yet near the setpoint, 100% duty until we get close
        if (input < (setpoint - 10.0)) {
            setHeater(true);
            g_heatPower = 1.0f;
            return;
        }
        else {

            if (pid.Compute()) {
                Serial.printf("PID Target: %.1f, %.1f, %.1f\n", input, output, setpoint);
                if (output > 0.0) {
                    g_heatPower = output * 0.01f;
                    t_width = (unsigned long)(output * (double)HEATER_PERIOD * 0.01);

                    // if (t_width > (HEATER_PERIOD - HEATER_MIN_PERIOD)) {
                    //     t_width = (HEATER_PERIOD - HEATER_MIN_PERIOD;
                    // }
                }
                else {
                    g_heatPower = 0.0;
                }
            }

            auto t_now = millis();

            // Turn on heater every period, if non-zero
            if ((t_now - t_last) >= HEATER_PERIOD) {
                t_last = t_now;
                if ((t_width > HEATER_MIN_PERIOD) && (t_start == 0)) {
                    t_start = t_now;
                    Serial.printf("Heat: %d/%d\n", t_width, HEATER_PERIOD);
                    setHeater(true);
                }
            }

            // Turn off heater after defined period
            if ((t_start > 0) && ((t_now - t_start) > t_width)) {
                // Keep heater on if at 100% duty
                if (t_width < (HEATER_PERIOD - HEATER_MIN_PERIOD)) {
                    setHeater(false);
                }

                t_width = 0;
                t_start = 0;
            }

        }

    }
}
