#include <Arduino.h>
#include "IO.h"
#include "StateMachine.h"
#include "hardware.h"
#include "button.h"
#include "driver/touch_sensor.h"
#include "config.h"

// Temporary: For lever pull detection
#include "UI.h"
#include "SensorSampler.h"

#define USE_WATERLEVEL

// Reading is typically 0 when water is filled,
// and ~500 when it needs filling
const touch_value_t water_threshold_high = 20;
const touch_value_t water_threshold_low = 10;

static bool s_isHeaterOn = false;
static float s_heaterPower = 0.0;
static bool s_waterLow = false;

static TimerHandle_t pwm_timer;
static TimerHandle_t pwm_timer_low;
static const unsigned long pwm_interval = 100;
static volatile unsigned long pwm_duty = 0;

extern "C" {
    uint8_t temprature_sens_read();
}

static Buttons<
    Btn<PIN_IN_POWER_BTN, HIGH>
> buttons;

namespace IO {

void onButtonPress(int pin) {
    switch (pin) {
        case 0: // POWER_BTN
            Serial.println("PWR BTN PRESSED\n");

            if (State::uiState == State::MachineState::Fault) {
                esp_restart();
            }
            else {
                bool pwr = (State::uiState == State::MachineState::Off  || State::uiState == State::MachineState::Sleep);
                State::setPowerControl(pwr);
            }
            break;
    }
}

/// @brief Reset device into a fail-safe mode
/// where any outputs are turned off.
void failsafe() {
    digitalWrite(PIN_OUT_HEAT, LOW);
    digitalWrite(PIN_OUT_PUMP, LOW);
    digitalWrite(PIN_OUT_FILL_SOLENOID, LOW);

    s_heaterPower = 0.0f;
}

static void onTimerTick(TimerHandle_t timer) {
    // atomic load from volatile
    auto duty = pwm_duty;
    auto interval = pwm_interval;

    if (duty < 5) {
        // Remain off (0%)
    }
    else {

        //Serial.printf("PWM ON: %d/%d\n", duty, interval);
        digitalWrite(PIN_OUT_PUMP, HIGH);

        //if (duty < interval) {
            if (xTimerStart(pwm_timer_low, pdMS_TO_TICKS(duty)) != pdPASS) {
                State::setFault(State::FaultState::SensorFailure, "PWM");
            }
        //}
        // else, remain on (100%)
    }
}

static void onTimerLowTick(TimerHandle_t timer) {
    //Serial.printf("PWM: OFF1\n");
    digitalWrite(PIN_OUT_PUMP, LOW);
}

void initGpio() {
    // BOOT button used for debugging
    pinMode(0, INPUT);

    pinMode(PIN_IN_POWER_BTN, INPUT_PULLDOWN);
    pinMode(PIN_IN_LEVER, INPUT_PULLDOWN);
    pinMode(PIN_IN_WATER_LOW, INPUT);
    pinMode(PIN_OUT_HEAT, OUTPUT);
    pinMode(PIN_OUT_PUMP, OUTPUT);
    pinMode(PIN_OUT_FILL_SOLENOID, OUTPUT);

    buttons.onButtonPress(onButtonPress);

#ifdef USE_WATERLEVEL
    // Initialize touch sensor input, used to detect boiler water level
    // https://github.com/ESP32DE/esp-iot-solution-1/blob/master/documents/touch_pad_solution/touch_sensor_design_en.md
    touchSetCycles(0xF000, 0xF000);
    touchRead(T0);
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
#endif

    failsafe();
}

void initPwm() {
    Serial.println("Initializing soft PWM");
    // Soft PWM
    // pwm_timer = xTimerCreate("PWM1H", pdMS_TO_TICKS(pwm_interval), pdTRUE, nullptr, onTimerTick);
    // pwm_timer_low = xTimerCreate("PWM1L", pdMS_TO_TICKS(0), pdFALSE, nullptr, onTimerLowTick);

    //xTimerStart(pwm_timer, 0);

    // uint8_t group=(chan/8), timer=((chan/2)%4);
    //ledcSetup(LEDC_CH_PUMP, 5, 8); // CH1 5Hz Min 0.78ms (Min duty for 20ms pulse = 25)
    //ledcSetup(LEDC_CH_PUMP, 10, 8); // CH1 10Hz
    ledcSetup(LEDC_CH_PUMP, 15, 8); // CH1 15Hz
    ledcAttachPin(PIN_OUT_PUMP, LEDC_CH_PUMP);
    ledcWrite(LEDC_CH_PUMP, 0);
}

#ifdef USE_WATERLEVEL
void readWaterLevel() {
    // Detect the boiler water level using the touch peripheral
    static unsigned long t_last = 0;
    static unsigned long fill_counter = 0;
    static int cycle = 0;
    if ((millis() - t_last) > 500) {
        t_last = millis();
        
        switch (cycle++) {
            case 0: // Begin sampling touch channel
                //touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER); 
                touch_pad_filter_start(10);
                touch_pad_sw_start();
                break;

            case 1: // Read touch channel and turn off sampling
            {
                auto water_level_raw = touchRead(T0);
                //Serial.printf("WaterLevel: %d\n", water_level_raw);

                if (water_level_raw > water_threshold_high) {
                    if (fill_counter >= 5) {
                        s_waterLow = true;
                    }
                    else {
                        fill_counter++;
                    }
                }
                else if (water_level_raw < water_threshold_low) {
                    fill_counter = 0;
                    s_waterLow = false;
                }

                //Serial.println("Stop touch sample");
                touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
                touch_pad_filter_stop();
                cycle = 0;
                break;
            }

            default:
                cycle = 0;
                break;
        }
    }
}

void disableWaterLevel() {
    //Serial.println("Disable touch sampling");
    //touch_pad_filter_stop();
    s_waterLow = false;
}
#endif

void process() {
    buttons.process();

//     static unsigned long t_last2 = 0;
//     if ((millis() - t_last2) > 10000) {
//         t_last2 = millis();

//         auto itemp = temprature_sens_read();
//         Serial.printf("iTemp: %d\n", itemp);
//     }

    auto state = State::getState();

    if (state == State::MachineState::Fault) {
        s_waterLow = false;
        return;
    }

#if !CONFIG_PRESSURE_PROFILING
    // Activate pump when lever pulled.
    // This is separate from the Brewing state logic to keep things simple.
    // In the future we could modulate the pump to give flow or pressure control.
    if (state == State::MachineState::Ready ||
        state == State::MachineState::Brewing ||
        state == State::MachineState::Preheat)
    {
        IO::setPump(IO::isLeverPulled());
    }
    else if (state != State::MachineState::FillTank) {
        IO::setPump(false);
    }
#endif

#ifdef USE_WATERLEVEL
    if (state == State::MachineState::FillTank ||
        state == State::MachineState::Ready ||
        state == State::MachineState::Brewing ||
        state == State::MachineState::Preheat)
    {
        readWaterLevel();
    }
    else
    {
        // Disable water level sensor to prevent
        // corrosion of metal probe
        disableWaterLevel();
    }
#endif
}

bool isWaterTankLow() {
    static bool last_reading = false;

    // For reasons I don't understand, this signal being high
    // causes spurious readings of the water low GPIO.
    // As a workaround, only sample when it is low (sensor ready).
    // This is not ideal since if this remains high for some reason,
    // we will never know if the tank is empty or not.
    if (digitalRead(MAX_RDY) == LOW) {
        last_reading = (digitalRead(PIN_IN_WATER_LOW) == LOW);
    }
    
    return last_reading;
}

bool isBoilerTankLow() {
    return s_waterLow;
}

bool isLeverPulled() {
    return (digitalRead(PIN_IN_LEVER) == HIGH);
}

bool isBrewing() {
    return (
        (State::uiState != State::MachineState::Preheat) && 
        (State::uiState != State::MachineState::FillTank) &&
        SensorSampler::isFlowing() // Water is flowing to grouphead (and not filling boiler)
       // isLeverPulled()
    );
}

void setHeatPower(float duty) {
    // TODO: Use this for setting heater PWM
    s_heaterPower = duty;
}

void setHeat(bool en) {
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
    s_isHeaterOn = en;
}

void setPump(bool en) {
    static bool prev_value = LOW;
    if (en != prev_value) {
        Serial.printf("PUMP: %s\n", en ? "ON" : "OFF");
        prev_value = en;
    }

    pwm_duty = 0; // Disable PWM output, manual override
    //ledcWrite(LEDC_CH_PUMP, en ? 0xFF : 0);
    setPumpDuty(en ? 0xFF : 0);
    //digitalWrite(PIN_OUT_PUMP, en);

    // TODO: Set a watchdog timer that turns this off after X seconds
}

void setPumpDuty(float duty) {
    auto iduty = (uint8_t)(256.0 * duty);
    Serial.printf("Set pump duty = %d\n", iduty);

    if (duty <= 0.0f) {
        ledcWrite(LEDC_CH_PUMP, 0);
    }
    else if (duty >= 1.0f) {
        ledcWrite(LEDC_CH_PUMP, 0xFF);
    }
    else {
        ledcWrite(LEDC_CH_PUMP, iduty);
    }
}

void setWaterFillSolenoid(bool en) {
    digitalWrite(PIN_OUT_FILL_SOLENOID, en);
}

float getHeatPower() {
    return s_heaterPower;
}

bool isHeaterOn() {
    return s_isHeaterOn;
}

}