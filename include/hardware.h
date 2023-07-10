#pragma once

//
// Input GPIOs
//

// Lever actuator input switch
// High = Lever open
// Low  = Lever closed or partially open
#define PIN_IN_LEVER -1
//#define PIN_IN_LEVER 21 // Rev1

// Water tank low switch
// High = Water tank is low
#define PIN_IN_WATER_LOW 22
//#define PIN_IN_WATER_LOW 34 // Rev1

// Power button
#define PIN_IN_POWER_BTN 23

// Water flow pulse sensor input pin
// 5V-24V
// Pull-up
// Flow rate 0.75-1L/min
// https://www.eptsensor.com/flow-sensor/hall-flow-sensor/fm-hl3012-flow-meter-hall-sensor.html
// #define FLOW1_PULSE_PIN 15 // Rev1
// #define FLOW2_PULSE_PIN 16 // Rev1
#define FLOW1_PULSE_PIN 15
#define FLOW2_PULSE_PIN 21
// Note: D39 has stability issues, do not use for this.

// Boiler tank sensor (capsense/touch)
// Used for auto-fill logic
#define PIN_IN_WATER_FULL 4 // Touch0 // Rev1

//
// Output GPIOs
//

// Boiler heater element output (PWM)
#define PIN_OUT_HEAT 5
//#define PIN_OUT_HEAT 17 // Rev1

// Pump output (PWM)
#define PIN_OUT_PUMP 17
//#define PIN_OUT_PUMP 2 // Rev1

// Fill boiler solenoid output (On/Off)
#define PIN_OUT_FILL_SOLENOID 5
//#define PIN_OUT_FILL_SOLENOID 22 // Rev1


//
// I2C Bus
//
// [0x7F] : Pressure Transducer
//
#define I2C_SCL     18
#define I2C_SDA     19
// #define I2C_SCL1     19 // Rev1
// #define I2C_SCL2     5 // Rev1, 2nd transducer, shares SDA
// #define I2C_SDA     18 // Rev1

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
#define PRESSURE_MAX_BAR (12.0f)

// Maximum flowrate for water flow sensor (mL/s)
#define FLOW_MAX_VALUE (15.0f)

//
// SPI Bus:
// - LCD Left
// - LCD Right
// - MAX31865 RTD
//

#define MAX_CS      25
#define MAX_RDY     35

#define RTD_NOMINAL_RESISTANCE (100)
#define RTD_REFERENCE_RESISTANCE (430)
#define RTD_MIN_TEMP (-200)
#define RTD_MAX_TEMP (400)
