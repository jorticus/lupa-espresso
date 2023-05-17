#pragma once

#include <stdint.h>

typedef struct {
    bool is_valid;
    int pressure;
    int temperature;
} pressure_sample_t;

class PressureTransducer
{
public:
    PressureTransducer(int full_scale_pressure, uint8_t i2c_addr = 0x7F)
        : full_scale_pressure(full_scale_pressure), i2c_addr(i2c_addr)
    { }

    bool begin();

    bool startSample();
    bool isSampleReady();
    pressure_sample_t readSample();

private:
    bool readRegister(uint8_t reg, uint8_t* buf, size_t len);

    const int full_scale_pressure;
    const uint8_t i2c_addr;
};
