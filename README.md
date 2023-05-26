# LUPA ESPRESSO

ESP32 firmware for controlling my modified Vibiemme Domobar Junior, which is an E61-grouphead Heat Exchanger (HX) espresso machine.

The main goal is to add some smarts, but retain the manual control and analog feel.

NOTE: This project is not intended to be re-used on other machines as it is highly specific to the particular modifications I have made.

## Features

- PID control of heater boiler
- Automatic brew timer
- Digital pressure / temperature / flow readouts
- Graphing and gauges displayed on two round TFT displays
- Grouphead temperature approximation
- Preheat/sleep states
- Remote control
- HomeAssistant integration

## Modifications

- Analog pressure dials replaced with round TFT displays
- Digital pressure transducer
- Water flow sensor
- Platinum PT100 thermocouple probe
- Boiler SSR controlled by ESP32
