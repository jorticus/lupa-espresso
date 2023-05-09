#pragma once

class PulseCounter
{
public:
    PulseCounter()
    { }

    bool begin(int pin, int sampleWindowMs = 200);

    bool isSampleReady();

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

