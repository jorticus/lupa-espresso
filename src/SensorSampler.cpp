#include <Arduino.h>
#include <freertos/semphr.h>
#include "SensorSampler.h"
#include "StateMachine.h"
#include "Debug.h"

#include "value_array.h"
#include "fir.h"

#include "hardware.h"
#include "Adafruit_MAX31865.h"
#include "PressureTransducer.h"
#include "PulseCounter.h"
#include "config.h"


#define PREHEAT_TEMPERATURE_C (CONFIG_BOILER_TEMPERATURE_C - 5.0f)

extern Adafruit_MAX31865   rtd1;
extern Adafruit_MAX31865   rtd2;
extern PressureTransducer  pressure;

volatile bool g_isWaterTankLow = false;
volatile bool g_isTemperatureSensorIdle = true;

namespace SensorSampler {

// Circular buffers for storing historical sensor readings.
// These are used to render graphs in the UI
ValueArray<float, numSamples> temperatureSamples;
ValueArray<float, numSamples> temperatureSamples2;
ValueArray<float, numSamples> pressureSamples;
ValueArray<float, numSamples> flowSamples;

static int waterTankDebounce = 0;

static bool isRtdAvailable = false;
static bool isPressureAvailable = false;
static bool isFlowAvailable = false;

static bool s_isFlowing = false;

// https://wirelesslibrary.labs.b-com.com/FIRfilterdesigner/#/#result-container
static const float filter_21tap_10hz[] = {
    // 0.5-10Hz @ 100Hz sample
    0.00434242941534996,0.008949466084110017,0.015604865894648115,0.025553833195889674,0.03669107854100511,0.04997034706423948,0.06273586677719478,0.07524549489402194,0.08472427725410565,0.09148873683644758,0.09336684579062392,0.09148873683644758,0.08472427725410565,0.07524549489402194,0.06273586677719478,0.04997034706423948,0.03669107854100511,0.025553833195889674,0.015604865894648115,0.008949466084110017,0.00434242941534996
};

static const float filter_41tap_20hz[] = {
    0.000013852416495175956,-0.0005731238127494758,-0.0009721997930380986,-0.00039876522390940147,0.0015930558424190862,0.0035931092532572894,0.002749971869114803,-0.0023913231618499036,-0.00876755135029208,-0.009422988391486293,0.0004666650013959363,0.01636373968020993,0.023898422721952334,0.009361110013097404,-0.02483789795201317,-0.05355958127324209,-0.04126902294878148,0.03159907402929509,0.14754130006838131,0.25526633301692403,0.29914396401895105,0.25526633301692403,0.14754130006838131,0.03159907402929509,-0.04126902294878148,-0.05355958127324209,-0.02483789795201317,0.009361110013097404,0.023898422721952334,0.01636373968020993,0.0004666650013959363,-0.009422988391486293,-0.00876755135029208,-0.0023913231618499036,0.002749971869114803,0.0035931092532572894,0.0015930558424190862,-0.00039876522390940147,-0.0009721997930380986,-0.0005731238127494758,0.000013852416495175956
};

static const float filter_21tap_20hz[] = {
    // Sample rate: 10Hz
    // Symmetric, 21 taps
    // Pass Band: 0-2Hz, 0.1 ripple
    // Stop Band: 2.5-5Hz, -40dB
    0.03545137019649825,-0.0022262708339058795,-0.034990888681942306,-0.021822371807547182,0.03237086731810076,0.03566348010931214,-0.048490211874641614,-0.09471486749562563,0.045288533737803296,0.30932157253256914,0.4456425549907703,0.30932157253256914,0.045288533737803296,-0.09471486749562563,-0.048490211874641614,0.03566348010931214,0.03237086731810076,-0.021822371807547182,-0.034990888681942306,-0.0022262708339058795,0.03545137019649825
};

static const float filter_21tap_1Hz[] = {
    // Sample rate: 10Hz
    // Symmetric, 21 taps
    // Pass Band: 0-1Hz, 1Hz ripple
    // Stop Band: 2.5-5Hz, -60dB
    0.001465356455737356,0.005946307256892348,0.010949886826641072,0.007317673377397633,-0.012924269445836039,-0.04020894607521747,-0.04139075948899614,0.018310463615369015,0.13680053545393453,0.2593177467532354,0.3116728085909527,0.2593177467532354,0.13680053545393453,0.018310463615369015,-0.04139075948899614,-0.04020894607521747,-0.012924269445836039,0.007317673377397633,0.010949886826641072,0.005946307256892348,0.001465356455737356
};

static const float filter_21tap_1p2Hz[] = {
    // Sample rate: 10Hz
    // Symmetric, 21 taps
    // Pass Band: 0-0.5Hz, 1Hz ripple
    // Stop Band: 1.2-5Hz, -60dB
    -0.0001765242939152268,0.004401257856427506,0.011006069982754735,0.022423968669624268,0.03866768524989652,0.059048935435686956,0.0818098105491962,0.10429767844259977,0.12342079736957758,0.1362809596254636,0.14081655316434108,0.1362809596254636,0.12342079736957758,0.10429767844259977,0.0818098105491962,0.059048935435686956,0.03866768524989652,0.022423968669624268,0.011006069982754735,0.004401257856427506,-0.0001765242939152268
};

static const float filter_11tap_1hz[] = {
    // Sample rate: 10Hz
    // Symmetric, 11 taps
    // Pass Band: 0-1Hz, 0.1Hz ripple
    // Stop Band: 2.5-5Hz, -60dB
    // NOTE: Using 11 taps gives us a faster response time
    -0.017761211782565366,-0.023388527987987403,0.02182651994577536,0.13593660665383586,0.2678079051444623,0.32697267711600697,0.2678079051444623,0.13593660665383586,0.02182651994577536,-0.023388527987987403,-0.017761211782565366
};

static const float filter_11tap_0p5hz[] = {
    // Sample rate: 10Hz
    // Symmetric, 11 taps
    // Pass Band: 0-0.6Hz, 0.5Hz ripple
    // Stop Band: 1-5Hz, -40dB
    0.06750407977332626,0.06447378522564463,0.08602036182482176,0.10434577815450256,0.11664142455124397,0.12098301069687106,0.11664142455124397,0.10434577815450256,0.08602036182482176,0.06447378522564463,0.06750407977332626
};

static const float filter_11tap_2hz[] = {
    // Sample rate: 100Hz
    // Symmetric, 11 taps
    // Pass Band: 0-2Hz, 0.5Hz ripple
    // Stop Band: 20-50Hz, -40dB
    0.008781172584127157,0.03397045986906245,0.07521111897851179,0.1272020312087974,0.16957364237449396,0.18701713727754435,0.16957364237449396,0.1272020312087974,0.07521111897851179,0.03397045986906245,0.008781172584127157
};

static const float filter_21tap_5hz[] = {
    0.04970840507037454,0.02432910126303465,0.029393020463172576,0.034442051358810365,0.03929393287595301,0.04379242949355152,0.04772232341361974,0.05097417933381713,0.05339159902528805,0.05488367919912227,0.055399526352131044,0.05488367919912227,0.05339159902528805,0.05097417933381713,0.04772232341361974,0.04379242949355152,0.03929393287595301,0.034442051358810365,0.029393020463172576,0.02432910126303465,0.04970840507037454
};

// Automatic FIR filters of type FirFilter<N_TAPS> { taps }
static auto filter_pressure     = MAKE_FIR_FILTER(filter_21tap_5hz);  // 100Hz sample rate
static auto filter_temperature_1  = MAKE_FIR_FILTER(filter_21tap_10hz);   
static auto filter_temperature_2  = MAKE_FIR_FILTER(filter_21tap_10hz);   

static auto filter_flowrate     = MAKE_FIR_FILTER(filter_11tap_1hz);    // 10Hz sample
static auto filter2_flowrate     = MAKE_FIR_FILTER(filter_11tap_1hz);

static float value_pressure = 0.0f;
static float value_temperature_1 = 0.0f;
static float value_temperature_2 = 0.0f;
static float value_flow_rate = 0.0f;
static float value_flow_volume = 0.0f;

static bool is_valid_pressure = false;
static bool is_valid_temperature_1 = false;
static bool is_valid_temperature_2 = false;
static bool is_valid_flow_rate = false;

static const unsigned long sampleRateMs = 10;
static const unsigned long temperatureSampleRateMs = 100;
static const auto sampleTickDelay = 500 / portTICK_PERIOD_MS;

static TimerHandle_t timer1;
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
        filter_pressure.add(sample.pressure);
    }
    else {
        filter_pressure.add(0);
    }
    if (filter_pressure.isReady()) {
        value_pressure = filter_pressure.get() * 0.0001f;
        is_valid_pressure = sample.is_valid;
    }
    pressure.startSample();

    auto t2 = millis();
    auto td = (t2 - t1);

    if (td > sampleRateMs) {
        // Above code took longer than the timer interval to execute
        Debug.printf("TIMER1 OVERFLOW %d\n", td);
        td = sampleRateMs;
    }

    // Debug.printf("P: %.1f  T: %.1f  F: %.1f  td:%d\n",
    //     value_pressure,
    //     value_temperature,
    //     value_flow_rate,
    //     (t2 - t1)
    // );

    auto interval = sampleRateMs - td;
    if (xTimerStart(timer1, pdMS_TO_TICKS(sampleRateMs)) != pdPASS) {
        Debug.println("ERROR: Could not rearm timer1");
        State::setFault(State::FaultState::SensorFailure, "TIMER1");
    }
}

static float calculateRtdTemperature(float rtd_raw) {
    // Doesn't matter if we use RTD1 or RTD2 here, calculation is the same.
    float t = rtd1.calculateTemperature(rtd_raw, RTD_NOMINAL_RESISTANCE, RTD_REFERENCE_RESISTANCE);
    
    // Correction factor for boiler RTD using real-life measurements
    return (t * 1.024f) - 1.38f;
}

static void onTemperatureTimer(TimerHandle_t timer) {
    static int divider10 = 0;
    static int divider2 = 0;

    auto t1 = millis();

    // MAX chip is configured for an automatic 60Hz sample rate.
    // Since we're only sampling at ~10Hz, we should always have a sample available.
    if (rtd1.isSampleReady()) {
        auto raw = rtd1.readSample();
        if (raw > 0 && raw < 0x7000) {//0x7FFF) {
            filter_temperature_1.add(raw);

            // auto unfiltered_temp = calculateRtdTemperature(raw);
            // if (unfiltered_temp > 200.0f) {
            //     Debug.printf("T GLITCH: %.1f %u\n", unfiltered_temp, raw);
            // }

            if (filter_temperature_1.isReady()) {
                float raw_filtered = filter_temperature_1.get();
                auto temperature = calculateRtdTemperature(raw_filtered);
                if (temperature > RTD_MIN_TEMP && temperature < RTD_MAX_TEMP) {
                    value_temperature_1 = temperature;
                    is_valid_temperature_1 = true;
                }
                else {
                    Debug.printf("INVALID RTD1 T: %.2f (0x%x)\n", temperature, raw_filtered);
                    is_valid_temperature_1 = false;
                }
            }
        } else {
            Debug.printf("INVALID RTD1 RAW: 0x%x\n", raw);
        }
    }

    // This GPIO (34) is cursed - it MUST be read here, due to an inexplicable effect from the MAX RTD chip.
    // Before RTD1, it always reads open. After RTD2, it always reads closed.
    // Outside of this loop, it will be unstable.
    // Even here, it must be debounced as it will occasinally flip states.
    if (digitalRead(PIN_IN_WATER_LOW) == LOW) {
        if (waterTankDebounce < 10) waterTankDebounce++;
        else g_isWaterTankLow = true;
    }
    else {
        if (waterTankDebounce > 0) waterTankDebounce--;
        else g_isWaterTankLow = false;
    }

    if (rtd2.isSampleReady()) {
        auto raw = rtd2.readSample();
        if (raw > 0 && raw < 0x7000) {//0x7FFF) {
            filter_temperature_2.add(raw);

            // auto unfiltered_temp = calculateRtdTemperature(raw);
            // if (unfiltered_temp > 200.0f) {
            //     Debug.printf("T GLITCH: %.1f %u\n", unfiltered_temp, raw);
            // }

            if (filter_temperature_2.isReady()) {
                float raw_filtered = filter_temperature_2.get();
                auto temperature = calculateRtdTemperature(raw_filtered);
                if (temperature > RTD_MIN_TEMP && temperature < RTD_MAX_TEMP) {
                    value_temperature_2 = temperature;
                    is_valid_temperature_2 = true;
                }
                else {
                    Debug.printf("INVALID RTD2 T: %.2f (0x%x)\n", temperature, raw_filtered);
                    is_valid_temperature_2 = false;
                }
            }
        } else {
            Debug.printf("INVALID RTD2 RAW: 0x%x\n", raw);
        }
    }

    g_isTemperatureSensorIdle = true;
    
    // The pulse counter should always have a valid sample, though it runs with its own timer
    // and may not be aligned to our sample rate.
    if (PulseCounter1.isSampleReady()) {
        auto flow1_hz = PulseCounter1.getFrequency();
        auto flow2_hz = PulseCounter2.getFrequency();
        if (flow1_hz < 300.0f) {
            // Use raw unfiltered value for detecting boolean isFlowing,
            // as this avoids introducing delay
            if (s_isFlowing && (flow1_hz < 1.0f)) {
                s_isFlowing = false;
                //value_flow_volume = 0.0f; // Reset volume
            }
            else if (!s_isFlowing && (flow1_hz > 5.0f)) {
                s_isFlowing = true;
            }
            
            // Conversion of Hz to normalized reading [0.0,1.0]
            // using a typical reading of 40Hz for maximum flow
            const float hz_normalized_coeff = 0.025f;
            
            // Calibration to ensure f2 == f1 when water 
            // can only leave via return path and not the grouphead
            const float f1_to_f2_ratio = 1.111f;

            // Calibration to mL/s
            // Note: coeff comes from calibrating the total volume, 
            // so we must also take into account the sample rate of 10Hz 
            // Note2: Not really sure why 0.5 works tbh, but the reading checks out.
            const float ml_calib_coeff = 0.5f * 10.0f; 
            
            const float f1_coeff = hz_normalized_coeff * ml_calib_coeff;
            const float f2_coeff = hz_normalized_coeff * ml_calib_coeff * f1_to_f2_ratio;
            
            // Calibrate flow readings
            flow1_hz *= f1_coeff;
            flow2_hz *= f2_coeff;

            // The flow rate out of the grouphead is the difference between
            // flow into the system (flow1) minus the flow out of the system (flow2),
            // though this does not account for filling of the preinfusion chamber.
            auto diff = (flow1_hz - flow2_hz);
            //Debug.printf("Flow A:%.3f B:%.3f mL/s\n", flow1_hz, flow2_hz);

            // The combined flowrate is a value between 0.0 and 1.0
            filter_flowrate.add(diff);
            if (filter_flowrate.isReady()) {
                value_flow_rate = filter_flowrate.get();

                // V[mL] = R[mL/s] * dt[s]
                const float t_delta = (float)temperatureSampleRateMs * 0.001f;
                value_flow_volume += (value_flow_rate * (float)t_delta);

                // // Measure time delta since last reading
                // static uint64_t t_flow_last = 0;
                // uint64_t t_flow = millis();
                // uint64_t t_delta = (t_flow - t_flow_last);
                // t_flow_last = t_flow;
                // if (t_delta > 1000) {
                //     t_delta = t_delta; // Out of range
                // }
                //value_flow_volume += (value_flow_rate * (float)t_delta * 0.001f);

                is_valid_flow_rate = true;
            }
        }
        else {
            // Ignore spurious reading (repeat sample)
        }
    } else {
        is_valid_flow_rate = false;
        s_isFlowing = false;
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
        if (filter_temperature_1.isReady()) {
            temperatureSamples.add(value_temperature_1);
        }
        if (filter_temperature_2.isReady()) {
            temperatureSamples2.add(value_temperature_2);
        }
    }

    auto t2 = millis();
    auto td = (t2 - t1);

    if (td > temperatureSampleRateMs) {
        // Above code took longer than the timer interval to execute
        Debug.println("TIMER2 OVERFLOW");
        td = temperatureSampleRateMs;
    }

    auto interval = temperatureSampleRateMs - td;
    if (xTimerStart(timer, pdMS_TO_TICKS(interval)) != pdPASS) {
        Debug.println("ERROR: Could not rearm timer2");
        State::setFault(State::FaultState::SensorFailure, "TIMER2");
    }

    //Debug.printf("T: %.1f  td:%d\n", value_temperature, (t2-t1));
}


bool initPressure() {
    Debug.println("Initialize Pressure");
    if (!pressure.begin()) {
        Debug.println("ERROR: No response from pressure transducer");
        return false;
    }
    else {
        return true;
    }
}

bool initTemperature() {
    Debug.println("Initialize MAX31865");

    pinMode(MAX_RDY, INPUT);
    // pinMode(MAX1_CS, OUTPUT);
    // pinMode(MAX2_CS, OUTPUT);

    auto entries = { std::reference_wrapper<Adafruit_MAX31865>(rtd1), std::reference_wrapper<Adafruit_MAX31865>(rtd2) };
    for (Adafruit_MAX31865& rtd : entries) {
        rtd.begin(MAX31865_3WIRE);
        rtd.enableBias(true);
        rtd.enable50Hz(true);
        
        //Debug.println(rtd.readRegister8(MAX31865_CONFIG_REG), HEX);
        //Debug.println("ERROR: No response from MAX");

        rtd.readRTD();
        auto fault = rtd.readFault();
        if (fault) {
            Debug.print("Fault 0x"); Debug.println(fault, HEX);
            if (fault & MAX31865_FAULT_HIGHTHRESH) {
                Debug.println("RTD High Threshold"); 
            }
            if (fault & MAX31865_FAULT_LOWTHRESH) {
                Debug.println("RTD Low Threshold"); 
            }
            if (fault & MAX31865_FAULT_REFINLOW) {
                Debug.println("REFIN- > 0.85 x Bias"); 
            }
            if (fault & MAX31865_FAULT_REFINHIGH) {
                Debug.println("REFIN- < 0.85 x Bias (FORCE- open)"); 
            }
            if (fault & MAX31865_FAULT_RTDINLOW) {
                // Likely means there is a short to ground
                Debug.println("RTDIN- < 0.85 x Bias (FORCE- open)"); 
            }
            if (fault & MAX31865_FAULT_OVUV) {
                Debug.println("Under/Over voltage"); 
            }
            return false;
        }
        else {
            rtd.autoConvert(true);

            // Wait for a sample to come in
            auto t1 = millis();
            while (((millis() - t1) < 1000) && (!rtd.isSampleReady()))
                continue;
            
            if (!rtd.isSampleReady()) {
                Debug.println("Error: No sample");
            }
            else if (rtd.readSample() == 0) {
                Debug.println("Error: Sample is zero");
            }
        }

    }

    digitalWrite(MAX1_CS, HIGH);
    digitalWrite(MAX2_CS, HIGH);

    return true;
}

bool initFlow() {
    Debug.println("Initialize Pulse Counter");

    pinMode(FLOW1_PULSE_PIN, INPUT);
    pinMode(FLOW2_PULSE_PIN, INPUT);
//    PulseCounter1.begin(FLOW_PULSE_PIN);
    return true;
}


bool initialize() {
    Debug.println("Initialize Sensor Sampler");

    // NOTE: Do not use auto-reset for the timer,
    // as this may lead to an assert within the stack when it tries to reset the timer.
    // Manually resetting the timer is more reliable.

    timer2 = xTimerCreate("SensorSamplerT", pdMS_TO_TICKS(temperatureSampleRateMs), pdFALSE, nullptr, onTemperatureTimer);
    if (timer2 == nullptr) {
        Debug.println("ERROR: Could not allocate SensorSamplerT timer");
    }
    timer1 = xTimerCreate("SensorSampler", pdMS_TO_TICKS(sampleRateMs), pdFALSE, nullptr, onSensorTimer);
    if (timer1 == nullptr) {
        Debug.println("ERROR: Could not allocate SensorSampler timer");
    }

    bool isTemperatureAvailable = initTemperature();
    bool isPressureAvailable = initPressure();
    bool isFlowAvailable = initFlow();

    return (isTemperatureAvailable && isPressureAvailable && isFlowAvailable);
}

void start() {
    Debug.println("Start sensor sampler");
    xTimerStart(timer2, 0);
    xTimerStart(timer1, 0);

    rtd1.autoConvert(true);
    rtd2.autoConvert(true);
    pressure.startSample();
    PulseCounter1.begin(FLOW1_PULSE_PIN, sampleRateMs);
    PulseCounter2.begin(FLOW2_PULSE_PIN, sampleRateMs);
}

void stop() {
    Debug.printf("Stop sensor sampler");
    xTimerStop(timer1, 0);
    xTimerStop(timer2, 0);

    rtd1.autoConvert(false);
    rtd2.autoConvert(false);
}

void process() {

}

float getTemperature() {
    return value_temperature_1;
}

float getTemperature2() {
    return value_temperature_2;
}

bool isTemperatureStabilized() {
    return (value_temperature_1 >= PREHEAT_TEMPERATURE_C);
}

float getEstimatedGroupheadTemperature() {
    if (!is_valid_temperature_1) {
        return 0.0f;
    }
    
    float t = value_temperature_1;
    if (t < 90.0f) {
        // Correction doesn't work in this range, 
        // switch to approximate linear correction
        return t * 0.8008f;
    }
    else {
        // Polynomial correction based on real measurements between
        // boiler and grouphead temperature.
        // NOTE: Real temperatures fluctuate a lot, so this is 
        // only an approximation.
        //return (t*t*0.0921f) - (t*21.614f) + 1363.7f;
        // =(D40*D40*0.0163)-(D40*3.2351)+250
        return (t*t*0.0163f) - (t*3.2351f) + 250.0f;
    }
}

float getPressure() {
    return value_pressure;
}

float getFlowRate() {
    return value_flow_rate;
}

float getTotalFlowVolume() {
    return value_flow_volume;
}

void resetFlowCounter() {
    value_flow_volume = 0.0f;
}

bool isTemperatureValid() {
    return is_valid_temperature_1;
}

bool isPressureValid() {
    return is_valid_pressure;
}

bool isFlowRateValid() {
    return is_valid_flow_rate;
}

bool isFlowing() {
    return s_isFlowing;
}

} // namespace SensorSampler
