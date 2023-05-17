#pragma once

//
// Input GPIOs
//

// Lever actuator input switch
// High = Lever open
// Low  = Lever closed or partially open
#define PIN_IN_LEVER 23

// Water tank low switch
// High = Water tank is low
#define PIN_IN_WATER_LOW 22

// Water flow pulse sensor input pin
// 5V-24V
// Pull-up
// Flow rate 0.75-1L/min
// https://www.eptsensor.com/flow-sensor/hall-flow-sensor/fm-hl3012-flow-meter-hall-sensor.html
#define FLOW_PULSE_PIN 15
// Note: D39 has stability issues, do not use for this.

// Boiler tank full sense
// Used for auto-fill logic
// High = Boiler is full
#define PIN_IN_WATER_FULL 21

//
// Output GPIOs
//

// Boiler heater element output (PWM)
#define PIN_OUT_HEAT 16

// Pump output (PWM)
#define PIN_OUT_PUMP 17

// Fill boiler solenoid output (On/Off)
#define PIN_OUT_FILL_SOLENOID 5


//
// I2C Bus
//
// [0x7F] : Pressure Transducer
//
#define I2C_SCL     18
#define I2C_SDA     19

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
#define PRESSURE_FULL_SCALE (120) // Bar

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
