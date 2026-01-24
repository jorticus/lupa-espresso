#include <driver/i2c_master.h>
#include "PressureTransducer.h"
#include "Debug.h"
#include "IO.h"

#define REGISTER_SAMPLE_CONTROL     0x30
#define REGISTER_PRESSURE_VALUE     0x06
#define REGISTER_TEMPERATURE_VALUE  0x09

// Sample control register
#define START_SAMPLE    0x0A
#define IS_SAMPLING     0x08

static const int I2C_TIMEOUT = 40; //pdMS_TO_TICKS(10);

static i2c_master_dev_handle_t i2c_device;

bool PressureTransducer::begin() {
    i2c_device_config_t i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0,
        // .scl_speed_hz = 10000
        .scl_speed_hz = 400000
    };

    // i2c_master_bus_handle_t handle;
    // ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &handle));

    ESP_ERROR_CHECK(i2c_master_bus_add_device(IO::i2c_bus, &i2c_device_config, &i2c_device));

    // Sample sequence:
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
    uint8_t txbuf[] = { REGISTER_SAMPLE_CONTROL, START_SAMPLE };
    esp_err_t err = i2c_master_transmit(i2c_device, txbuf, sizeof(txbuf), I2C_TIMEOUT);
    return (err == ESP_OK);
}

bool PressureTransducer::readRegister(uint8_t reg, uint8_t* buf, size_t len) {
    esp_err_t err = i2c_master_transmit_receive(i2c_device, 
        &reg, sizeof(reg),
        buf, len,
        I2C_TIMEOUT
    );
    return (err == ESP_OK);
}

bool PressureTransducer::isSampleReady() {
    uint8_t b;
    if (readRegister(REGISTER_SAMPLE_CONTROL, &b, sizeof(b))) {
        return (b & IS_SAMPLING) == 0;
    } else {
        return false;
    }
}

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
