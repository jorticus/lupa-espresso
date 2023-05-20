#pragma once

/// @brief Interface for counting pulses using ESP32 peripherals
class PulseCounter
{
public:
    PulseCounter()
    { }

    /// @brief Set up pulse counter
    /// @param pin The pin to count pulses on
    /// @param sampleWindowMs Window to measure pulses in
    /// @return false if there was an error setting up the pulse counter
    bool begin(int pin, int sampleWindowMs = 200);

    /// @brief A sample is ready for collection
    bool isSampleReady();

    /// @brief Latest pulse counter reading
    float getFrequency();

protected:
    static void IRAM_ATTR timerIsr();
    static void IRAM_ATTR pinChangeIsr(void* ctx);

    void IRAM_ATTR onTimer();
    void IRAM_ATTR onPinChange();

    void setSlowMode(bool slow);

    int _pin;
    int _interruptPin;
    int _sampleWindowMs;
    unsigned long _sampleWindowUs;
    volatile bool _slowMode;
    volatile bool _isSampleReady;
    volatile int16_t _count;
    volatile unsigned long _time;
    volatile unsigned long _timeDelta;
    volatile int _timeCount;
    volatile int16_t _t1;
    hw_timer_t* _timer;
};

extern PulseCounter PulseCounter1;

