#pragma once

#include <stdint.h>

typedef struct {
    bool is_valid;
    int pressure;
    int temperature;
} pressure_sample_t;

/// @brief Interface for Xidibei I2C pressure transducer
class PressureTransducer
{
public:
    PressureTransducer(int full_scale_pressure, uint8_t i2c_addr = 0x7F)
        : full_scale_pressure(full_scale_pressure), i2c_addr(i2c_addr)
    { }

    /// @brief Detect sensor on bus
    /// @return false if no sensor was found
    bool begin();

    /// @brief Instruct sensor to begin sampling
    /// @return false if I2C transaction error
    bool startSample();

    /// @brief A sample is ready for collection
    bool isSampleReady();

    /// @brief Read the latest sample from the sensor
    /// @return A structure containing temperature, pressure, and whether the sample is valid
    pressure_sample_t readSample();

private:
    bool readRegister(uint8_t reg, uint8_t* buf, size_t len);

    const int full_scale_pressure;
    const uint8_t i2c_addr;
};
