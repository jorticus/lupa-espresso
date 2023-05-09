#include <Arduino.h>
#include "PulseCounter.h"
#include <driver/timer.h>
#include <driver/pcnt.h>
#include <esp_intr_alloc.h>
//#include <FunctionalInterrupt.h>

/// @brief If ticks/window falls below this threshold, slow mode is engaged.
/// Slow mode measures the time between ticks.
const unsigned long SLOW_TICK_COUNT_THRESHOLD = 450;

/// @brief If ticks/window rises above this threshold, fast mode is engaged.
/// Fast mode measures the number of ticks in a fixed time window.
const unsigned long FAST_TICK_COUNT_THRESHOLD = 550;

/// @brief Maximum time between pulses that will be recognized.
/// If tick exceeds this length, value will be 0.
const unsigned long MAX_TICK_DELTA_USEC = 3000000;

PulseCounter PulseCounter1;

portMUX_TYPE timerIsrMux = portMUX_INITIALIZER_UNLOCKED;

typedef void (*voidFuncPtr)(void);
typedef void (*voidFuncPtrArg)(void*);

extern "C"
{
	extern void __attachInterruptFunctionalArg(uint8_t pin, voidFuncPtrArg userFunc, void * arg, int intr_type, bool functional);
}

void IRAM_ATTR PulseCounter::timerIsr() {
    portENTER_CRITICAL_ISR(&timerIsrMux);
    PulseCounter1.onTimer();
    portEXIT_CRITICAL_ISR(&timerIsrMux);
}

void IRAM_ATTR PulseCounter::pinChangeIsr(void* ctx) {
    reinterpret_cast<PulseCounter*>(ctx)->onPinChange();
}

void IRAM_ATTR PulseCounter::onTimer() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);

    this->_count = count;
    this->_isSampleReady = true;

    if (!_slowMode && (count < SLOW_TICK_COUNT_THRESHOLD)) {
        setSlowMode(true);
    }

    pcnt_counter_clear(PCNT_UNIT_0);
}

void IRAM_ATTR PulseCounter::onPinChange() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);

    if (_slowMode && (count > FAST_TICK_COUNT_THRESHOLD)) {
        setSlowMode(false);
        return;
    }

    unsigned long t = esp_timer_get_time();
    unsigned long delta = t - _time;
    _time = t;

    if (delta < MAX_TICK_DELTA_USEC) {
        _timeDelta += delta;
        _timeCount++;

        if (_timeDelta > _sampleWindowUs) {
            _isSampleReady = true;
        }
    }
}

bool PulseCounter::begin(int pin, int sampleWindowMs) {
    this->_pin = pin;
    this->_interruptPin = digitalPinToInterrupt(pin);
    this->_sampleWindowMs = sampleWindowMs;
    this->_sampleWindowUs = sampleWindowMs * 1000;

    pcnt_config_t config = {
        .pulse_gpio_num = pin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = INT16_MAX,
        .counter_l_lim = 0,
        .unit = PCNT_UNIT_0,
        .channel = PCNT_CHANNEL_0
    };

    auto err = pcnt_unit_config(&config);
    if (err != ESP_OK) {
        Serial.printf("ERROR: Could not configure PCNT peripheral: %x\n", err);
        return false;
    }

    setSlowMode(true);

    const int timerIdx = 0;
    const int timerDivier = 80;
    _timer = timerBegin(timerIdx, timerDivier, true);
    timerAttachInterrupt(_timer, &PulseCounter::timerIsr, true);
    timerAlarmWrite(_timer, sampleWindowMs*1000, true);

    pcnt_counter_clear(PCNT_UNIT_0);
    timerAlarmEnable(_timer);
    return true;
}

void PulseCounter::setSlowMode(bool slow) {
    if (slow) {
        Serial.println("Slow mode");
        __attachInterruptFunctionalArg(_interruptPin, &PulseCounter::pinChangeIsr, this, CHANGE, false);
    }
    else {
        Serial.println("Fast mode");
        detachInterrupt(_interruptPin);
    }

    this->_slowMode = slow;
}

bool PulseCounter::isSampleReady() {
    bool ready = this->_isSampleReady;
    _isSampleReady = false;
    return ready;
}

float PulseCounter::getFrequency() {
    float value = 0.0f;
    if (_slowMode) {
        if (_timeCount > 0) {
            // usecs/tick -> ticks/second
            value = 1000000.0f / (float)(_timeDelta / _timeCount);

            _timeDelta = 0;
            _timeCount = 0;
        }
    }
    else {
        // ticks/timeInterval -> ticks/second
        value = (int)_count * 1000 / _sampleWindowMs;
    }

    return value;
}