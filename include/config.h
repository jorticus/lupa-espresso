#pragma once

// Target boiler temperature for brewing (BoilerProfile::Brew)
//#define CONFIG_BOILER_TEMPERATURE_C (115.0f) // ~95C on grouphead
//#define CONFIG_BOILER_TEMPERATURE_C (117.0f) // ~96C on grouphead
#define CONFIG_BOILER_TEMPERATURE_C (120.0f) // ~97C on grouphead
//#define CONFIG_BOILER_TEMPERATURE_C (125.0f) // ~94C on grouphead according to manual

// Target boiler temperature for steaming (BoilerProfile::Steam)
#define CONFIG_BOILER_STEAM_TEMPERATURE_C (125.0f)

// Target boiler temperature in idle mode (BoilerProfile::Idle)
#define CONFIG_BOILER_IDLE_TEMPERATURE_C (60.0f)

// Target boiler temperature for PID tuning (BoilerProfile::Tuning)
#define CONFIG_BOILER_TUNING_TEMPERATURE_C (110.f)

// Range at which the PID controller is active
// relative to the current PID setpoint
#define CONFIG_BOILER_PID_RANGE_C (20.0f)

// Maximum boiler temperature
// Heat control loop will be disabled above this temperature.
// Important: This does not replace the need for a hardware safety cutoff!
// The VBM Domobar HX has three safety cutoffs - 
// A. the pressurestat will regulate the upper temperature
// B. the over pressure valve on the boiler will release steam if pressure exceeds safe limits
// C. the thermal fuse will blow if the boiler exterior exceeds safe limits
#define CONFIG_MAX_BOILER_TEMPERATURE_C (130.0f)

// Target preheat temperature
// Defines when to exit the preheat phase and become ready.
// Does not control actual target temperature, only controls for how
// long the preheat UI is shown.
#define CONFIG_BOILER_PREHEAT_TARGET_C (CONFIG_BOILER_TEMPERATURE_C - 5.0f)

// Minimum system pressure
// Machine will attempt to keep pressure above this level when idle
#define CONFIG_MIN_PRESSURE (0.5f)
#define CONFIG_MAX_PRESSURE (12.0f)

// Target brew pressure
#define CONFIG_TARGET_BREW_PRESSURE (9.0f)

// Machine idle timeout
//#define CONFIG_IDLE_TIMEOUT_MS (0) // Disabled
//#define CONFIG_IDLE_TIMEOUT_MS (30*60*1000) // 30m
#define CONFIG_IDLE_TIMEOUT_MS (2*60*60*1000) // 2h

// Fault clear timeout
#define CONFIG_FAULT_CLEAR_TIMEOUT_MS (30*60*1000) // 30sec

// If true, put device into power-saving mode as soon as it is
// up to temperature. 
#define CONFIG_IDLE_AFTER_PREHEAT (false)

// Brew finish timeout
// Time before post-brew screen disappears and is replaced with the Ready screen
#define CONFIG_BREW_FINISH_TIMEOUT_MS (60*1000)

#define CONFIG_FULL_BRIGHTNESS (1.0f)
#define CONFIG_IDLE_BRIGHTNESS (0.2f)

// Whether to wait for WiFi connection on startup
#define CONFIG_WAIT_FOR_WIFI (false)

// Perform PID tuning process instead of usual logic
#define CONFIG_DO_PID_TUNE (false)

// Measured power of boiler in watts,
// used for estimating energy consumption
#define CONFIG_BOILER_FULL_POWER_WATTS (1333.0f)

// Whether to enable pressure profiling (PID control of pressure during brew)
#define CONFIG_ENABLE_PRESSURE_PROFILING (true)
