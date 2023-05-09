#pragma once

#define LCD_DC      15
#define LCD_CS      22

#define I2C_SCL     19
#define I2C_SDA     21

#define MAX_MOSI    13
#define MAX_MISO    12
#define MAX_CLK     14
#define MAX_CS      27
#define MAX_RDY     26

// https://www.eptsensor.com/flow-sensor/hall-flow-sensor/fm-hl3012-flow-meter-hall-sensor.html
// 5V-24V
// Flow rate 0.75-1L/min
#define FLOW_PULSE_PIN 25

#define RTD_NOMINAL_RESISTANCE (100)
#define RTD_REFERENCE_RESISTANCE (430)
#define RTD_MIN_TEMP (-200)
#define RTD_MAX_TEMP (400)

#define PRESSURE_I2C_ADDR (0x7F)

// Note: even though full scale pressure is 12 bar, 
// the actual full scale is 120 units.
// Pressure should read 1.0 bar when open to atmosphere.
#define PRESSURE_FULL_SCALE (120) // Bar

// White: SDA
// Black: SCL
// Brown: 3.3V
// Blue:  GND
