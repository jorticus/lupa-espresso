#pragma once

//
// Input GPIOs
//

// Lever actuator input switch
// High = Lever open
// Low  = Lever closed or partially open
#define PIN_IN_LEVER 8

// Water tank low switch
// High = Water tank is low
//#define PIN_IN_WATER_LOW 22
#define PIN_IN_WATER_LOW 9

// Power button
#define PIN_IN_POWER_BTN 10

// Water flow pulse sensor input pin
// 5V-24V
// Pull-up
// Flow rate 0.75-1L/min
// https://www.eptsensor.com/flow-sensor/hall-flow-sensor/fm-hl3012-flow-meter-hall-sensor.html
#define FLOW1_PULSE_PIN 11
#define FLOW2_PULSE_PIN 12
// Note: D39 has stability issues, do not use for this.

// Boiler tank sensor (capsense/touch)
// Used for auto-fill logic
#define PIN_IN_WATER_FULL 4 // Touch0

//
// Output GPIOs
//

// Boiler heater element output (PWM)
#define PIN_OUT_HEAT 13

// Pump output (PWM)
#define PIN_OUT_PUMP 14

// Fill boiler solenoid output (On/Off)
//#define PIN_OUT_FILL_SOLENOID 5
#define PIN_OUT_FILL_SOLENOID 45


//
// I2C Bus
//
// [0x7F] : Pressure Transducer
//
#define I2C_SCL     46
#define I2C_SDA     47
//#define I2C_SCL1     46
#define I2C_SCL2     48 // 2nd transducer, shares SDA

// Pressure Transducer
// White: SDA
// Black: SCL
// Brown: 3.3V
// Blue:  GND
#define PRESSURE_I2C_ADDR (0x7F)

// Full-scale pressure reading
// Note: even though full scale pressure is 12 bar, 
// the actual full scale is 120 units.
// Pressure should read 1.0 bar when open to atmosphere.
#define PRESSURE_FULL_SCALE (120) // Bar x10


//
// SPI Bus 1 (HSPI)
// - LCD Left & Right
// SPI Bus 2 (VSPI)
// - MAX31865 RTD #1 & #2
//

#define MAX_RDY     40

#define MAX1_CS     38
#define MAX2_CS     39

#define RTD_NOMINAL_RESISTANCE (100)
#define RTD_REFERENCE_RESISTANCE (430)
#define RTD_MIN_TEMP (-200)
#define RTD_MAX_TEMP (400)


#define LEDC_CH_DISPLAY (0)
#define LEDC_CH_PUMP    (2)

#define UART_DEBUG_BAUD (115200)
