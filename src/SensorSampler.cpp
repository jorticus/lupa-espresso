#include <Arduino.h>
#include <freertos/semphr.h>
#include "SensorSampler.h"

#include "value_array.h"

#include "hardware.h"
#include "Adafruit_MAX31865.h"
#include "pressure_transducer.h"
#include "PulseCounter.h"

extern Adafruit_MAX31865   rtd;
extern PressureTransducer  pressure;

const int numSamples = 300;
ValueArray<float, numSamples> temperatureSamples;
ValueArray<float, numSamples> pressureSamples;
ValueArray<float, numSamples> flowSamples;

// 1Hz filter, assuming sample rate of 5Hz (200ms)
// https://wirelesslibrary.labs.b-com.com/FIRfilterdesigner/#/#result-container
static float filter_taps[] = {
    0.000013852416493342338,-0.0005731238127498802,-0.0009721997930368535,-0.0003987652239076082,0.0015930558424197066,0.003593109253255786,0.0027499718691119065,-0.0023913231618520802,-0.008767551350291674,-0.009422988391483304,0.0004666650013992939,0.016363739680210786,0.023898422721949482,0.009361110013092595,-0.024837897952016432,-0.053559581273241195,-0.041269022948776955,0.03159907402929975,0.14754130006838234,0.2552663330169204,0.29914396401894533,0.2552663330169204,0.14754130006838234,0.03159907402929975,-0.041269022948776955,-0.053559581273241195,-0.024837897952016432,0.009361110013092595,0.023898422721949482,0.016363739680210786,0.0004666650013992939,-0.009422988391483304,-0.008767551350291674,-0.0023913231618520802,0.0027499718691119065,0.003593109253255786,0.0015930558424197066,-0.0003987652239076082,-0.0009721997930368535,-0.0005731238127498802,0.000013852416493342338
};
const int SAMPLEFILTER_TAP_NUM = sizeof(filter_taps) / sizeof(filter_taps[0]);

template <size_t N_TAPS>
class FirFilter {
public:
    FirFilter() : samples(), last_index(0)
    {
        for (int i = 0; i < N_TAPS; i++) {
            samples[i] = 0;
        }
    }

    void add(float sample) {
        samples[last_index++] = sample;
        if (last_index == N_TAPS)
            last_index = 0;
    }

    float get() const {
        float acc = 0;
        int index = last_index;
        for (int i = 0; i < N_TAPS; ++i) {
            index = index != 0 ? index-1 : N_TAPS-1;
            acc += samples[index] * filter_taps[i];
        };
        return acc;
    }

    std::array<float, N_TAPS> samples;
    unsigned int last_index;
    bool ready;
};

FirFilter<SAMPLEFILTER_TAP_NUM> filter1;
FirFilter<SAMPLEFILTER_TAP_NUM> filter2;
FirFilter<SAMPLEFILTER_TAP_NUM> filter3;

static float value_pressure = 0.0f;
static float value_temperature = 0.0f;
static float value_flow_rate = 0.0f;

static bool is_valid_pressure = false;
static bool is_valid_temperature = false;
static bool is_valid_flow_rate = false;

static const unsigned long sampleRateMs = 200; // 5Hz
const auto sampleTickDelay = 500 / portTICK_PERIOD_MS;

static TimerHandle_t timer;

/// @brief Sensor sampling timer, records sensor readings at a regular interval
/// @param timer FreeRTOS timer handle
static void onSensorTimer(TimerHandle_t timer) {
    auto t1 = millis();

    // As long a the timer interval is greater than ~50ms, we should have a valid sample by now
    auto sample = pressure.readSample();
    if (sample.is_valid) {
        filter1.add(sample.pressure);
    }
    else {
        filter1.add(0);
    }
    value_pressure = filter1.get();
    is_valid_pressure = sample.is_valid;
    pressure.startSample();

    // MAX chip is configured for an automatic 60Hz sample rate.
    // Since we're only sampling at ~10Hz, we should always have a sample available.
    auto raw = rtd.readSample();
    if (raw != 0) {
        //rtd_value = raw;
        filter2.add(raw);
        float raw_filtered = filter2.get();
        auto temperature = rtd.calculateTemperature(raw_filtered, RTD_NOMINAL_RESISTANCE, RTD_REFERENCE_RESISTANCE);
        if (temperature > RTD_MIN_TEMP && temperature < RTD_MAX_TEMP) {
            value_temperature = temperature;
            is_valid_temperature = true;
            //filter2.add(temperature);
            //rtd_value = temperature;
            //rtd_value = filter2.get();
        }
        else {
            is_valid_temperature = false;
        }
    } else {
        is_valid_temperature = false;
    }


    // The pulse counter should always have a valid sample, though it runs with its own timer
    // and may not be aligned to our sample rate.
    auto pulses_per_second = PulseCounter1.getFrequency();
    if (pulses_per_second > 0.1f && pulses_per_second < 300.0f) {
        const float correction = 0.155f;
        value_flow_rate = pulses_per_second * correction;
        is_valid_flow_rate = true;
    }
    else {
        // Invalid value, clamp to 0
        value_flow_rate = 0.0f;
        is_valid_flow_rate = false;
    }

    auto t2 = millis();

    if ((t2 - t1) > sampleRateMs) {
        // Above code took longer than the timer interval to execute
        // Typically takes 5-90ms, but sometimes exceeds 100ms
        Serial.println("TIMER OVERFLOW");
    }

    Serial.printf("P: %.1f  T: %.1f  F: %.1f  td:%d\n",
        value_pressure,
        value_temperature,
        value_flow_rate,
        (t2 - t1)
    );
}

void SensorSampler::Initialize() {
    Serial.println("Initialize Sensor Sampler");

    timer = xTimerCreate("SensorSampler", pdMS_TO_TICKS(sampleRateMs), pdTRUE, nullptr, onSensorTimer);
}

void SensorSampler::Start() {
    xTimerStart(timer, 0);

    rtd.autoConvert(true);
    pressure.startSample();
    PulseCounter1.begin(FLOW_PULSE_PIN, sampleRateMs);
}

void SensorSampler::Stop() {
    xTimerStop(timer, 0);

    rtd.autoConvert(false);
}

void SensorSampler::Process() {

}

float SensorSampler::getTemperature() {
    return value_temperature;
}

float SensorSampler::getPressure() {
    return value_pressure * 0.0001f;
}

float SensorSampler::getFlowRate() {
    return value_flow_rate;
}

float SensorSampler::getTotalFlowVolume() {
    return 0.0f; // TODO
}

void SensorSampler::resetFlowCounter() {
    // TODO
}

bool SensorSampler::isTemperatureValid() {
    return is_valid_temperature;
}

bool SensorSampler::isPressureValid() {
    return is_valid_pressure;
}

bool SensorSampler::isFlowRateValid() {
    return is_valid_flow_rate;
}
