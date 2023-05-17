#include <Arduino.h>
#include <Wire.h>
#include "PressureTransducer.h"

#define REGISTER_SAMPLE_CONTROL     0x30
#define REGISTER_PRESSURE_VALUE     0x06
#define REGISTER_TEMPERATURE_VALUE  0x09

// Sample control register
#define START_SAMPLE    0x0A
#define IS_SAMPLING     0x08

bool PressureTransducer::begin() {
    // Write 0x0A to register 0x30 to begin acquisition
    // Read 0x30 until bit3 is 0, or wait 50ms
    // Read register 0x06, 5 bytes
    if (!startSample()) {
        return false;
    }

    while (!isSampleReady())
        continue;

    auto sample = readSample();
    return sample.is_valid;
}

bool PressureTransducer::startSample() {
    Wire.beginTransmission(i2c_addr);
    Wire.write(REGISTER_SAMPLE_CONTROL);
    Wire.write(START_SAMPLE);
    return (Wire.endTransmission() == 0);
}

bool PressureTransducer::isSampleReady() {
    Wire.beginTransmission(i2c_addr);
    Wire.write(REGISTER_SAMPLE_CONTROL);
    if (Wire.endTransmission() != 0)
        return false;

    Wire.requestFrom(i2c_addr, (uint8_t)1);
    int b = Wire.read();
    return (b & IS_SAMPLING) == 0;
}

#define PRESSURE_I2C_ADDR 0x7F

pressure_sample_t PressureTransducer::readSample() {
    pressure_sample_t sample = {0};
    uint8_t buf[5];

    if (readRegister(REGISTER_PRESSURE_VALUE, buf, sizeof(buf)))
    {

        int raw_pressure = (buf[0] << 16) | (buf[1] << 8) | buf[2];
        if (raw_pressure > 0x800000)
        raw_pressure -= 0x1000000;

        // 120 * 1000 = 20 bits
        // 24 bits + 20 bits + 1 sign bit = 44 bits total required for integer math to work.
        // Return value in 1000'ths of a unit.
        sample.pressure = ((int64_t)raw_pressure * this->full_scale_pressure * 1000) >> 23;

        int16_t raw_temperature = (buf[3] << 8) | buf[4];
        sample.temperature = (((int32_t)raw_temperature * 1000) >> 8);

        sample.is_valid = true;
    }

    return sample;
}

bool PressureTransducer::readRegister(uint8_t reg, uint8_t* buf, size_t len) {
    Wire.beginTransmission(i2c_addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
        Serial.println("NACK from pressure");
        return false;
    }

    Wire.requestFrom(i2c_addr, len);
    size_t n_bytes = Wire.readBytes(buf, len);
    if (n_bytes < sizeof(buf)) {
        Serial.printf("ERROR: Received %d bytes, expected %d\n", n_bytes, len);
        return false;
    }

    return true;
}
