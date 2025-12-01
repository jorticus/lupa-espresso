#include <Arduino.h>
#include <Wire.h>
#include "IO.h"
#include "Debug.h"
#include "StateMachine.h"
#include "hardware.h"
#include "button.h"
#include "driver/touch_sensor.h"
#include "config.h"

// Temporary: For lever pull detection
#include "UI.h"
#include "SensorSampler.h"

#define USE_WATERLEVEL

extern volatile bool g_isWaterTankLow;
extern volatile bool g_isTemperatureSensorIdle;

// Reading is typically 0 when water is filled,
// and ~500 when it needs filling
const touch_value_t water_threshold_high = 20;
const touch_value_t water_threshold_low = 10;

static bool s_isPwmInitialized = false;
static bool  s_isHeaterOn = false;
static float s_heaterPower = 0.0;
static bool  s_waterLow = false;
static unsigned long s_boilerInterval = 0;

const float PUMP_DUTY_MIN = 67.0f;  // Depends on configured ledc frequency
const float PUMP_DUTY_MAX = 255.0f;
const uint8_t PUMP_DUTY_OFF = 0;
const uint8_t PUMP_DUTY_ON = 0xFF;

const unsigned long HEATER_MIN_PERIOD = 100;
const unsigned long HEATER_PERIOD = 5000;

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
            Debug.println("PWR BTN PRESSED\n");

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

    if (s_isPwmInitialized) {
        ledcWrite(LEDC_CH_PUMP, PUMP_DUTY_OFF);
    }

    s_heaterPower = 0.0f;
}

void initGpio() {
    // Pressure sensors
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);  // I2C #1
    pinMode(I2C_SCL2, INPUT_PULLUP); // I2C #2
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();

    // The following devices share the same SPI bus. 
    // Ensure all CS pins are de-asserted.
    pinMode(TFT_CS_LEFT, OUTPUT);
    pinMode(TFT_CS_RIGHT, OUTPUT);
    pinMode(MAX1_CS, OUTPUT);
    pinMode(MAX2_CS, OUTPUT);
    digitalWrite(MAX1_CS, HIGH);
    digitalWrite(MAX2_CS, HIGH);
    digitalWrite(TFT_CS_LEFT, HIGH);
    digitalWrite(TFT_CS_RIGHT, HIGH);

    // Turn off display backlight
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);

    // Inputs & Outputs
    pinMode(PIN_IN_POWER_BTN, INPUT_PULLDOWN);
    pinMode(PIN_IN_LEVER, INPUT_PULLDOWN);
    pinMode(PIN_IN_WATER_LOW, INPUT);
    pinMode(PIN_OUT_HEAT, OUTPUT);
    pinMode(PIN_OUT_PUMP, OUTPUT);
    pinMode(PIN_OUT_FILL_SOLENOID, OUTPUT);

    // Set callback for power button (debouncd)
    buttons.onButtonPress(onButtonPress);

    // Just to be consistent, set IO into failsafe mode (outputs off)
    failsafe();
}

void initPwm() {
    // This is separate from initGpio so the failsafe recovery code can detect failures in init here
    Debug.println("Initializing PWM");

#ifdef USE_WATERLEVEL
    // Initialize touch sensor input, used to detect boiler water level
    // https://github.com/ESP32DE/esp-iot-solution-1/blob/master/documents/touch_pad_solution/touch_sensor_design_en.md
    touchSetCycles(0xF000, 0xF000);
    touchRead(T0);
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
#endif


#if CONFIG_ENABLE_PRESSURE_PROFILING
    // Configure LEDC peripheral for pump PWM output.
    // The pump is driven by an SSR with zero-crossing detection,
    // so there is a minimum pulse width of 10ms (one half-cycle of 50Hz AC).
    // Anything less will not activate the pump.

    // uint8_t group=(chan/8), timer=((chan/2)%4);
    //ledcSetup(LEDC_CH_PUMP, 5, 8);  // CH1 5Hz, Min duty 20
    //ledcSetup(LEDC_CH_PUMP, 10, 8); // CH1 10Hz, Min duty 10  -- this seems to be unstable with PID loop
    ledcSetup(LEDC_CH_PUMP, 15, 8);  // CH1 15Hz, Min duty 67
    ledcAttachPin(PIN_OUT_PUMP, LEDC_CH_PUMP);
    ledcWrite(LEDC_CH_PUMP, PUMP_DUTY_OFF);
    s_isPwmInitialized = true;
#endif
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
                //Debug.printf("WaterLevel: %d\n", water_level_raw);

                if (water_level_raw > water_threshold_high) {
                    if (fill_counter >= 5) {
                        if (!s_waterLow) {
                            Debug.println("Boiler tank low");
                        }
                        s_waterLow = true;
                    }
                    else {
                        fill_counter++;
                    }
                }
                else if (water_level_raw < water_threshold_low) {
                    fill_counter = 0;
                    if (s_waterLow) {
                        Debug.println("Boiler tank okay");
                    }
                    s_waterLow = false;
                }

                //Debug.println("Stop touch sample");
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
    //Debug.println("Disable touch sampling");
    //touch_pad_filter_stop();
    s_waterLow = false;
}
#endif

void updateBoilerPwm() {
    static unsigned long t_last = 0;
    static unsigned long s_boilerStartTs = 0;

    auto t_now = millis();
    if ((t_now - t_last) >= HEATER_PERIOD) {
        t_last = t_now;

        if (s_boilerInterval == 0) {
            // PWM not yet loaded, set the next interval
            s_boilerInterval = (unsigned long)(s_heaterPower * (float)HEATER_PERIOD);

            // Don't activate PWM if pulse is too short
            if (s_boilerInterval < HEATER_MIN_PERIOD) {
                s_boilerInterval = 0;
                setHeat(false);
                return;
            }
            // Begin next cycle
            else {
                s_boilerStartTs = t_now;
                Debug.printf("Heat: %d/%d\n", s_boilerInterval, HEATER_PERIOD);
                setHeat(true);
            }
        }
    }

    // Turn off heater after defined period
    if ((s_boilerInterval > 0) && ((t_now - s_boilerStartTs) > s_boilerInterval)) {
        // Keep heater on if at 100% duty (only turn off if width is less than max period)
        if (s_boilerInterval < (HEATER_PERIOD - HEATER_MIN_PERIOD)) {
            setHeat(false);
        }

        s_boilerInterval = 0;
    }
}

void process() {
    // Debounce buttons
    buttons.process();

    auto state = State::getState();
    if (state == State::MachineState::Fault) {
        s_waterLow = false;
        return;
    }

#if !CONFIG_ENABLE_PRESSURE_PROFILING
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

    updateBoilerPwm();
}

bool isWaterTankLow() {
    return g_isWaterTankLow;
    // static bool last_reading = false;

    // // // For reasons I don't understand, this signal being high
    // // // causes spurious readings of the water low GPIO.
    // // // As a workaround, only sample when it is low (sensor ready).
    // // // This is not ideal since if this remains high for some reason,
    // // // we will never know if the tank is empty or not.
    // // if (digitalRead(MAX_RDY) == LOW) {
    // //     last_reading = (digitalRead(PIN_IN_WATER_LOW) == LOW);
    // // }

    // if (g_isTemperatureSensorIdle) {
    //     last_reading = (digitalRead(PIN_IN_WATER_LOW) == LOW);
    //     g_isTemperatureSensorIdle = false;
    // }
    
    // return last_reading;
}

bool isBoilerTankLow() {
    return s_waterLow;
}

bool isLeverPulled() {
    return (digitalRead(PIN_IN_LEVER) == HIGH);
}

bool isBrewing() {
    return (
        (State::uiState != State::MachineState::FillTank) &&
        SensorSampler::isFlowing() && // Water is flowing to grouphead (and not filling boiler)
        isLeverPulled()
    );
}

void setHeatPower(float duty) {
    if (duty < 0.0f) {
        duty = 0.0f;
    }
    else if (duty >= 1.0f) {
        duty = 1.0f;
    }

    s_heaterPower = duty;

    // Immediately turn off boiler, don't wait for next PWM cycle
    if (duty <= 0.0f) {
        setHeat(false);
    }
}

void setHeat(bool en) {
    static bool prev_value = LOW;

    if (en) {
        if (en != prev_value) {
            Debug.println("HEAT: ON");
        }
        digitalWrite(PIN_OUT_HEAT, HIGH);
    }
    else {
        if (en != prev_value) {
            Debug.println("HEAT: OFF");
        }
        digitalWrite(PIN_OUT_HEAT, LOW);

        // Reset PWM cycle
        s_boilerInterval = 0;
    }

    prev_value = en;
    s_isHeaterOn = en;
}

void setPump(bool en) {
    static bool prev_value = LOW;
    if (en != prev_value) {
        Debug.printf("PUMP: %s\n", en ? "ON" : "OFF");
        prev_value = en;
    }

#if CONFIG_ENABLE_PRESSURE_PROFILING
    setPumpDuty(en ? PUMP_DUTY_ON : PUMP_DUTY_OFF);
#else
    digitalWrite(PIN_OUT_PUMP, en);
#endif
}

void setPumpDuty(float duty) {
#if CONFIG_ENABLE_PRESSURE_PROFILING
    auto iduty = (uint8_t)((float)PUMP_DUTY_MAX * duty);
    Debug.printf("Set pump duty = %d\n", iduty);

    if (duty <= 0.0f) {
        ledcWrite(LEDC_CH_PUMP, PUMP_DUTY_OFF);
    }
    else if (duty >= 1.0f) {
        ledcWrite(LEDC_CH_PUMP, PUMP_DUTY_ON);
    }
    else {
        ledcWrite(LEDC_CH_PUMP, iduty);
    }
#endif
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