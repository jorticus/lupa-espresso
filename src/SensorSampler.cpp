#include <Arduino.h>
#include <freertos/semphr.h>
#include "SensorSampler.h"

#include "value_array.h"

#include "hardware.h"
#include "Adafruit_MAX31865.h"
#include "PressureTransducer.h"
#include "PulseCounter.h"

extern Adafruit_MAX31865   rtd;
extern PressureTransducer  pressure;

const int numSamples = 300;
ValueArray<float, numSamples> temperatureSamples;
ValueArray<float, numSamples> pressureSamples;
ValueArray<float, numSamples> flowSamples;

bool isRtdAvailable = false;
bool isPressureAvailable = false;
bool isFlowAvailable = false;

// https://wirelesslibrary.labs.b-com.com/FIRfilterdesigner/#/#result-container
static float filter_taps_10hz[] = {
    // 0.5Hz @ 10Hz sample
    //0.000013852416493342338,-0.0005731238127498802,-0.0009721997930368535,-0.0003987652239076082,0.0015930558424197066,0.003593109253255786,0.0027499718691119065,-0.0023913231618520802,-0.008767551350291674,-0.009422988391483304,0.0004666650013992939,0.016363739680210786,0.023898422721949482,0.009361110013092595,-0.024837897952016432,-0.053559581273241195,-0.041269022948776955,0.03159907402929975,0.14754130006838234,0.2552663330169204,0.29914396401894533,0.2552663330169204,0.14754130006838234,0.03159907402929975,-0.041269022948776955,-0.053559581273241195,-0.024837897952016432,0.009361110013092595,0.023898422721949482,0.016363739680210786,0.0004666650013992939,-0.009422988391483304,-0.008767551350291674,-0.0023913231618520802,0.0027499718691119065,0.003593109253255786,0.0015930558424197066,-0.0003987652239076082,-0.0009721997930368535,-0.0005731238127498802,0.000013852416493342338

    // 20Hz @ 100Hz sample
    //0.000013852416495175956,-0.0005731238127494758,-0.0009721997930380986,-0.00039876522390940147,0.0015930558424190862,0.0035931092532572894,0.002749971869114803,-0.0023913231618499036,-0.00876755135029208,-0.009422988391486293,0.0004666650013959363,0.01636373968020993,0.023898422721952334,0.009361110013097404,-0.02483789795201317,-0.05355958127324209,-0.04126902294878148,0.03159907402929509,0.14754130006838131,0.25526633301692403,0.29914396401895105,0.25526633301692403,0.14754130006838131,0.03159907402929509,-0.04126902294878148,-0.05355958127324209,-0.02483789795201317,0.009361110013097404,0.023898422721952334,0.01636373968020993,0.0004666650013959363,-0.009422988391486293,-0.00876755135029208,-0.0023913231618499036,0.002749971869114803,0.0035931092532572894,0.0015930558424190862,-0.00039876522390940147,-0.0009721997930380986,-0.0005731238127494758,0.000013852416495175956

    // 1-10Hz @ 100Hz sample
    // 0.1-1Hz @ 10Hz sample
    //-0.0038816404946323798,0.005186306757490993,0.011187306915538823,0.020942863551074237,0.03385108254853396,0.0489843113646178,0.0649614184647876,0.08006975505821248,0.09251335964912975,0.10070514571819074,0.10356516269016608,0.10070514571819074,0.09251335964912975,0.08006975505821248,0.0649614184647876,0.0489843113646178,0.03385108254853396,0.020942863551074237,0.011187306915538823,0.005186306757490993,-0.0038816404946323798

    // 0.5-10Hz @ 100Hz sample
    0.00434242941534996,0.008949466084110017,0.015604865894648115,0.025553833195889674,0.03669107854100511,0.04997034706423948,0.06273586677719478,0.07524549489402194,0.08472427725410565,0.09148873683644758,0.09336684579062392,0.09148873683644758,0.08472427725410565,0.07524549489402194,0.06273586677719478,0.04997034706423948,0.03669107854100511,0.025553833195889674,0.015604865894648115,0.008949466084110017,0.00434242941534996
};
const int SAMPLEFILTER_TAP_NUM = sizeof(filter_taps_10hz) / sizeof(filter_taps_10hz[0]);

template <size_t N_TAPS>
class FirFilter {
public:
    FirFilter(const float taps[N_TAPS]) : taps(taps), samples(), last_index(0)
    {
        for (int i = 0; i < N_TAPS; i++) {
            samples[i] = 0;
        }
    }

    void add(float sample) {
        samples[last_index++] = sample;
        if (last_index == N_TAPS) {
            last_index = 0;
            ready = true;
        }
    }

    float get() const {
        float acc = 0;
        int index = last_index;
        for (int i = 0; i < N_TAPS; ++i) {
            index = index != 0 ? index-1 : N_TAPS-1;
            acc += samples[index] * taps[i];
        };
        return acc;
    }

    bool isReady() const {
        return ready;
    }

private:
    const float* taps;
    std::array<float, N_TAPS> samples;
    unsigned int last_index;
    bool ready;
};

FirFilter<SAMPLEFILTER_TAP_NUM> filter1 { filter_taps_10hz };
FirFilter<SAMPLEFILTER_TAP_NUM> filter2 { filter_taps_10hz };
FirFilter<SAMPLEFILTER_TAP_NUM> filter3 { filter_taps_10hz };

static float value_pressure = 0.0f;
static float value_temperature = 0.0f;
static float value_flow_rate = 0.0f;

static bool is_valid_pressure = false;
static bool is_valid_temperature = false;
static bool is_valid_flow_rate = false;

static const unsigned long sampleRateMs = 10;
static const unsigned long temperatureSampleRateMs = 100;
const auto sampleTickDelay = 500 / portTICK_PERIOD_MS;

static TimerHandle_t timer;
static TimerHandle_t timer2;

bool isMaxSampleReady() {
    return (digitalRead(MAX_RDY) == LOW);
}

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
    if (filter1.isReady()) {
        value_pressure = filter1.get() * 0.0001f;
        is_valid_pressure = sample.is_valid;
    }
    pressure.startSample();

    auto t2 = millis();

    if ((t2 - t1) > sampleRateMs) {
        // Above code took longer than the timer interval to execute
        Serial.println("TIMER1 OVERFLOW");
    }

    // Serial.printf("P: %.1f  T: %.1f  F: %.1f  td:%d\n",
    //     value_pressure,
    //     value_temperature,
    //     value_flow_rate,
    //     (t2 - t1)
    // );
}

static float calculateRtdTemperature(float rtd_raw) {
    float t = rtd.calculateTemperature(rtd_raw, RTD_NOMINAL_RESISTANCE, RTD_REFERENCE_RESISTANCE);
    
    // Correction factor for boiler RTD using real-life measurements
    return (t * 1.024f) - 1.38f;
}

static void onTemperatureTimer(TimerHandle_t timer) {
    static int divider10 = 0;
    static int divider2 = 0;

    auto t1 = millis();

    // MAX chip is configured for an automatic 60Hz sample rate.
    // Since we're only sampling at ~10Hz, we should always have a sample available.
    if (rtd.isSampleReady()) {
        auto raw = rtd.readSample();
        if (raw > 0 && raw < 0x7FFF) {
            filter2.add(raw);

            // auto unfiltered_temp = calculateRtdTemperature(raw);
            // if (unfiltered_temp > 200.0f) {
            //     Serial.printf("T GLITCH: %.1f %u\n", unfiltered_temp, raw);
            // }

            if (filter2.isReady()) {
                float raw_filtered = filter2.get();
                auto temperature = calculateRtdTemperature(raw_filtered);
                if (temperature > RTD_MIN_TEMP && temperature < RTD_MAX_TEMP) {

                    value_temperature = temperature;
                    is_valid_temperature = true;
                }
                else {
                    is_valid_temperature = false;
                }
            }
        }
    }

    
    // The pulse counter should always have a valid sample, though it runs with its own timer
    // and may not be aligned to our sample rate.
    if (PulseCounter1.isSampleReady()) {
        auto pulses_per_second = PulseCounter1.getFrequency();
        if (pulses_per_second < 300.0f) {
            const float correction = 0.155f;
            value_flow_rate = pulses_per_second * correction;
            is_valid_flow_rate = true;
        }
        else {
            // Ignore spurious reading (repeat sample)
        }
    }

    // 100ms per tick
    if (divider2++ == 2) {
        divider2 = 0;
        pressureSamples.add(value_pressure);
        flowSamples.add(value_flow_rate);
    }

    // 1 sec per tick
    if (divider10++ == 10) {
        divider10 = 0;
        if (filter2.isReady()) {
            temperatureSamples.add(value_temperature);
        }
    }

    auto t2 = millis();

    if ((t2 - t1) > temperatureSampleRateMs) {
        // Above code took longer than the timer interval to execute
        Serial.println("TIMER2 OVERFLOW");
    }

    //Serial.printf("T: %.1f  td:%d\n", value_temperature, (t2-t1));
}


bool initPressure() {
    Serial.println("Initialize Pressure");
    if (!pressure.begin()) {
        Serial.println("ERROR: No response from pressure transducer");
        return false;
    }
    else {
        return true;
    }
}

bool initTemperature() {
    Serial.println("Initialize MAX31865");

    pinMode(MAX_RDY, INPUT);

    rtd.begin(MAX31865_3WIRE);
    rtd.enableBias(true);
    rtd.enable50Hz(true);
    
    //Serial.println(rtd.readRegister8(MAX31865_CONFIG_REG), HEX);
    //Serial.println("ERROR: No response from MAX");

    rtd.readRTD();
    auto fault = rtd.readFault();
    if (fault) {
        Serial.print("Fault 0x"); Serial.println(fault, HEX);
        if (fault & MAX31865_FAULT_HIGHTHRESH) {
            Serial.println("RTD High Threshold"); 
        }
        if (fault & MAX31865_FAULT_LOWTHRESH) {
            Serial.println("RTD Low Threshold"); 
        }
        if (fault & MAX31865_FAULT_REFINLOW) {
            Serial.println("REFIN- > 0.85 x Bias"); 
        }
        if (fault & MAX31865_FAULT_REFINHIGH) {
            Serial.println("REFIN- < 0.85 x Bias (FORCE- open)"); 
        }
        if (fault & MAX31865_FAULT_RTDINLOW) {
            Serial.println("RTDIN- < 0.85 x Bias (FORCE- open)"); 
        }
        if (fault & MAX31865_FAULT_OVUV) {
            Serial.println("Under/Over voltage"); 
        }
    }
    else {
        rtd.autoConvert(true);

        // Wait for a sample to come in
        auto t1 = millis();
        while (((millis() - t1) < 1000) && (!rtd.isSampleReady()))
            continue;
        
        if (!rtd.isSampleReady()) {
            Serial.println("Error: No sample");
        }
        else if (rtd.readSample() == 0) {
            Serial.println("Error: Sample is zero");
        }

        return true;
    }

    return false;
}

bool initFlow() {
    Serial.println("Initialize Pulse Counter");

    pinMode(FLOW_PULSE_PIN, INPUT);
//    PulseCounter1.begin(FLOW_PULSE_PIN);
    return true;
}


bool SensorSampler::initialize() {
    Serial.println("Initialize Sensor Sampler");

    timer2 = xTimerCreate("SensorSamplerT", pdMS_TO_TICKS(temperatureSampleRateMs), pdTRUE, nullptr, onTemperatureTimer);
    if (timer2 == nullptr) {
        Serial.println("ERROR: Could not allocate SensorSamplerT timer");
    }
    timer = xTimerCreate("SensorSampler", pdMS_TO_TICKS(sampleRateMs), pdTRUE, nullptr, onSensorTimer);
    if (timer == nullptr) {
        Serial.println("ERROR: Could not allocate SensorSampler timer");
    }

    bool isTemperatureAvailable = initTemperature();
    bool isPressureAvailable = initPressure();
    bool isFlowAvailable = initFlow();

    return (isTemperatureAvailable && isPressureAvailable && isFlowAvailable);
}

void SensorSampler::start() {
    xTimerStart(timer2, 0);
    xTimerStart(timer, 0);

    rtd.autoConvert(true);
    pressure.startSample();
    PulseCounter1.begin(FLOW_PULSE_PIN, sampleRateMs);
}

void SensorSampler::stop() {
    xTimerStop(timer, 0);
    xTimerStop(timer2, 0);

    rtd.autoConvert(false);
}

void SensorSampler::process() {

}

float SensorSampler::getTemperature() {
    return value_temperature;
}

float SensorSampler::getPressure() {
    return value_pressure;
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
