
#include "esp32-hal-touch-legacy.h"

#include <Arduino.h>
#include <driver/i2c_master.h>
#include <Adafruit_SPIDevice.h>
#include "IO.h"
#include "Debug.h"
#include "StateMachine.h"
#include "hardware.h"
#include "button.h"

#include <driver/touch_sensor_legacy.h>
// #define CONFIG_TOUCH_SUPPRESS_DEPRECATE_WARN 1
// #include <driver/touch_sensor.h>
// #include <driver/touch_sens.h>

#include "config.h"

// Temporary: For lever pull detection
#include "UI.h"
#include "SensorSampler.h"

#define USE_WATERLEVEL

extern volatile bool g_isWaterTankLow;

#if defined(LUPA_V2)
extern SPIClass spi;  // HSPI (TFT)
extern SPIClass spi2; // VSPI
#endif

// Reading is typically 0 when water is filled,
// and ~500 when it needs filling
const touch_value_t water_threshold_high = 20;
const touch_value_t water_threshold_low = 10;

static bool s_isPwmInitialized = false;
static bool  s_isHeaterOn = false;
static float s_heaterPower = 0.0;
static bool  s_waterLow = false;
static unsigned long s_boilerInterval = 0;
static bool s_isFailsafeTriggered = false;

// const float PUMP_DUTY_MIN = 67.0f * 256.0f;  // Depends on configured ledc frequency
// const float PUMP_DUTY_MAX = 255.0f * 256.0f;
const uint32_t PUMP_DUTY_OFF = 0;
const uint32_t PUMP_DUTY_ON = ((1<<14)-1);

const unsigned long HEATER_MIN_PERIOD = 100;
const unsigned long HEATER_PERIOD = 5000;

extern "C" {
    uint8_t temprature_sens_read();
}

static Buttons<
    Btn<PIN_IN_POWER_BTN, HIGH>
> buttons;

namespace IO {

i2c_master_bus_handle_t i2c_bus;

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
    Debug.println("failsafe");

    pinMode(PIN_OUT_HEAT, OUTPUT);
    pinMode(PIN_OUT_FILL_SOLENOID, OUTPUT);

    digitalWrite(PIN_OUT_HEAT, LOW);
    digitalWrite(PIN_OUT_FILL_SOLENOID, LOW);

    if (s_isPwmInitialized) {
        ledcWriteChannel(LEDC_CH_PUMP, PUMP_DUTY_OFF);
    } else {
        pinMode(PIN_OUT_PUMP, OUTPUT);
        digitalWrite(PIN_OUT_PUMP, LOW);
    }

    s_heaterPower = 0.0f;
}

void initI2C() {
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)I2C_SDA,
        .scl_io_num = (gpio_num_t)I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true
        }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));
}

void initGpio() {
    // Pressure sensors
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);  // I2C #1
    pinMode(I2C_SCL2, INPUT_PULLUP); // I2C #2

    initI2C();

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

#if defined(LUPA_V2)
    // SPI busses
    // spi2.begin(40, 39, 38, 41); // TODO: GPIO defs
#endif

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
    touchRead(PIN_IN_WATER_FULL);
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
   
    // ledcSetup(LEDC_CH_PUMP, 15, 8);  // CH1 15Hz, Min duty 67
    // ledcAttachPin(PIN_OUT_PUMP, LEDC_CH_PUMP);
    ledcAttachChannel(PIN_OUT_PUMP, 15, 14, LEDC_CH_PUMP); // CH1 15Hz, Min duty 67

    ledcWrite(LEDC_CH_PUMP, PUMP_DUTY_OFF);
    s_isPwmInitialized = true;
#endif
}

#ifdef USE_WATERLEVEL
void readWaterLevel() {
    // Detect the boiler water level using the touch peripheral
    static unsigned long t_last = 0;
    static unsigned long fill_counter = 0;
    static touch_value_t last_level_raw = 0;
    static int cycle = 0;
    if ((millis() - t_last) > 500) {
        t_last = millis();
        
        switch (cycle++) {
            case 0: // Begin sampling touch channel
                //touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER); 
                    
                #if defined(CONFIG_IDF_TARGET_ESP32S3)
                touch_pad_filter_enable();
                #elif defined(CONFIG_IDF_TARGET_ESP32)
                // touch_pad_filter_start(10);
                #endif

                touch_pad_sw_start();
                break;

            case 1: // Read touch channel and turn off sampling
            {
                auto water_level_raw = touchRead(PIN_IN_WATER_FULL);
                if (water_level_raw != last_level_raw) {
                    last_level_raw = water_level_raw;
                    Debug.printf("Boiler Level Sensor: %d\n", water_level_raw);
                }

#if defined(CONFIG_IDF_TARGET_ESP32)
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
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
                // TODO...
                Debug.println("***BOILER TANK LEVEL NOT IMPLEMENTED***");
#endif

                //Debug.println("Stop touch sample");
                touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);

                #if defined(CONFIG_IDF_TARGET_ESP32S3)
                touch_pad_filter_disable();
                #elif defined(CONFIG_IDF_TARGET_ESP32)
                // touch_pad_filter_stop();
                #endif

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
#if defined(LUPA_V1)
    // Workaround for GPIO instability on V1 board
    return g_isWaterTankLow;
#elif defined(LUPA_V2)
    return digitalRead(PIN_IN_WATER_LOW);
#endif
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

    pinMode(PIN_OUT_HEAT, OUTPUT);

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
    setPumpDuty(en ? 1.0f : 0.0f);
#else
    digitalWrite(PIN_OUT_PUMP, en);
#endif
}

void setPumpDuty(float duty) {
#if CONFIG_ENABLE_PRESSURE_PROFILING
    static uint32_t iduty_last = 0;
    uint32_t iduty = (uint32_t)((float)PUMP_DUTY_ON * duty);

    if (iduty != iduty_last) {
        iduty_last = iduty;
        Debug.printf("Set pump = %.1f\n", duty);
    }

    if (duty <= 0.0f) {
        ledcWriteChannel(LEDC_CH_PUMP, PUMP_DUTY_OFF);
    }
    else if (duty >= 1.0f) {
        ledcWriteChannel(LEDC_CH_PUMP, PUMP_DUTY_ON);
    }
    else {
        ledcWriteChannel(LEDC_CH_PUMP, iduty);
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