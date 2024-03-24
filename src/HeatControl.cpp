#include <Arduino.h>
#include "SensorSampler.h"
#include "HeatControl.h"
#include "HomeAssistant.h"
#include "MqttParamManager.h"
#include "PID.h"
#include "IO.h"
#include "hardware.h"
#include "config.h"

namespace Defaults {
    // How often PID is calculated
    // NOTE: If this is changed, the coefficients will need to be updated too
    static const unsigned long UpdatePeriodMs = 1000;

    static float SetPoint = CONFIG_BOILER_TEMPERATURE_C;

    static const float Kp = 10.0f;
    static const float Ki = Kp / 13.0f; // Ki = Kp / Tn
    static const float Kd = Kp * 5.0f; // Kd = Kp * Tv

    static const float PlantOffset = 0.0f;

    static const float MaxBoilerTemperature = CONFIG_MAX_BOILER_TEMPERATURE_C;
    static const float RegulationRange = CONFIG_BOILER_PID_RANGE_C;
}



// TODO: May need to incorporate another temperature probe for steam water,
// as currently the control loop doesn't respond to a drop in main boiler temperature
// until far too late. 
// Or, we could use a second pressure transducer which would respond very quickly, 
// and also let us detect whether steam is being used.

static fPID pid;

const float MAX_BOILER_TEMPERATURE = CONFIG_MAX_BOILER_TEMPERATURE_C;

namespace HeatControl {

static BoilerProfile operating_profile = BoilerProfile::Off;

void updatePidCoefficients();

// Configuration parameters to expose to MQTT
MqttParam::Parameter<float> param_kp("pid/boiler/kp", Defaults::Kp, [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_ki("pid/boiler/ki", Defaults::Ki, [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_kd("pid/boiler/kd", Defaults::Kd, [] (float val) { updatePidCoefficients(); });
MqttParam::Parameter<float> param_po("pid/boiler/po", Defaults::PlantOffset, [] (float val) { updatePidCoefficients(); });

MqttParam::Parameter<float> param_boilerTemp("brew/boiler_temp", CONFIG_BOILER_TEMPERATURE_C,       [] (float val) { setProfile(operating_profile); });
MqttParam::Parameter<float> param_steamTemp("brew/steam_temp",   CONFIG_BOILER_STEAM_TEMPERATURE_C, [] (float val) { setProfile(operating_profile);  });

void initControlLoop()
{
    pid.reset();
    pid.setOutputLimits(0.0f, 100.0f);
    pid.setParameters(Defaults::Kp, Defaults::Ki, Defaults::Kd);
    pid.setSetpoint(Defaults::SetPoint);
    pid.setSampleTime(Defaults::UpdatePeriodMs);
    pid.setPlantOffset(Defaults::PlantOffset);
    pid.setRegulationRange(Defaults::RegulationRange);

    updatePidCoefficients();

    // TODO: Prevent integral from going below 0, as it can
    // take a LOT longer to cool down than it will to warm up,
    // so the integral term will work against us.

    setProfile(BoilerProfile::Brew);
}


void updatePidCoefficients() {
    float kp = param_kp.value();
    float ki = param_ki.value();
    float kd = param_kd.value();
    float po = param_po.value();

    pid.setParameters(kp, ki, kd);
    pid.setPlantOffset(po);
    pid.reset();

    Serial.printf("Boiler PID Parameters:\n\tKp: %.4f\n\tKi: %.4f\n\tKd: %.4f\n\tOf: %.4f\n", 
        pid.getKp(),
        pid.getKi(),
        pid.getKd(),
        po
    );
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
        //IO::setHeat(false);
        IO::setHeatPower(0.0f);
    }
    else {
        float pid_input = SensorSampler::getTemperature();

        if (pid_input > Defaults::MaxBoilerTemperature) {
            //IO::setHeat(false);
            IO::setHeatPower(0.0f);
            pid.reset();
            return;
        }

        // Not yet near the pid_setpoint, 100% duty until we get close
        if (pid_input < (pid.getSetpoint() - Defaults::RegulationRange)) {
            if (IO::getHeatPower() < 1.0f) {
                Serial.printf("PREHEAT: %.1f\n", pid_input);
            }
            //IO::setHeat(true);
            IO::setHeatPower(1.0f);
            pid.reset();
            return;
        }
        else {

#if false
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
#endif

            auto t_now = millis();
            if ((t_now - t_last_pid) >= Defaults::UpdatePeriodMs) {
                t_last_pid = t_now;

                if (operating_profile == BoilerProfile::Tuning) {
                    pid_output = calculateTuningTick(pid_input);
                    publishTuningData(pid_input, pid_output);
                }
                else {
                    pid_output = pid.calculateTick(pid_input);
                }

                //Serial.printf("PID: I=%.1f, S=%.1f, O=%.1f\n", pid_input, pid_setpoint, pid_output);

                IO::setHeatPower(pid_output * 0.01f);
            }
        }
    }
}

void setProfile(BoilerProfile mode) {
    operating_profile = mode;

    Serial.print("Boiler heat profile: ");
    switch (mode) {
        case BoilerProfile::Off:
            Serial.println("Off");
            pid.reset();
            break;
        case BoilerProfile::Brew:
            Serial.println("Brew");
            //pid.setSetpoint(CONFIG_BOILER_TEMPERATURE_C);
            pid.setSetpoint(param_boilerTemp.value());
            break;
        case BoilerProfile::Steam:
            Serial.println("Steam");
            //pid.setSetpoint(CONFIG_BOILER_STEAM_TEMPERATURE_C);
            pid.setSetpoint(param_steamTemp.value());
            break;
        case BoilerProfile::Idle:
            Serial.println("Idle");
            pid.setSetpoint(CONFIG_BOILER_IDLE_TEMPERATURE_C);
            break;
        case BoilerProfile::Tuning:
            Serial.println("Tuning");
            pid.setSetpoint(CONFIG_BOILER_TUNING_TEMPERATURE_C);
            break;
    }
}

BoilerProfile getProfile() {
    return operating_profile;
}

}