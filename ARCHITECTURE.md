# Task Overview

## Core 0 : RF/Radio

## Core 1 : Arduino loop()

### Tasks:

- CoreIO : Priority 3, 1ms tick
  - GPIO handling
  - State machine
  - PID control loops
        
- Network : Priority 2, 10ms tick
  - WiFi reconnection logic
  - MQTT reconnection logic
  - HomeAssistant publishing
  - OTA

- UI : Priority 1
  - Idle loop (exeucte when no other tasks running)
  - Render
  - Display Update

- loop() : Priority 1
  - Failsafe code execution only.

### Timers:

- Timer 1 : 10ms
  - Pressure

- Timer 2 : 100ms
  - Temperature RTD 1 & 2
  - Water Low GPIO
  - Pulse Counter 1 & 2
  - Update graph samples
