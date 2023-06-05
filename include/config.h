#pragma once

// If true, put device into power-saving mode as soon as it is
// up to temperature. 
#define CONFIG_SLEEP_AFTER_PREHEAT (false)

// Preheat end temperature
// Defines when to exit the preheat phase and become ready.
// Does not control actual target temperature.
//#define CONFIG_BOILER_PREHEAT_TEMPERATURE_C (90.0f)

// Target boiler temperature for brewing
//#define CONFIG_BOILER_TEMPERATURE_C (115.0f) // ~95C on grouphead
#define CONFIG_BOILER_TEMPERATURE_C (117.0f)
//#define CONFIG_BOILER_TEMPERATURE_C (120.0f) // ~97C on grouphead

// Target boiler temperature for steaming
#define CONFIG_BOILER_STEAM_TEMPERATURE_C (123.0f)

// Target boiler temperature in sleep mode
#define CONFIG_BOILER_SLEEP_TEMPERATURE_C (95.0f)

// Maximum boiler temperature
#define CONFIG_MAX_BOILER_TEMPERATURE_C (125.0f)

// Machine idle timeout
#define CONFIG_IDLE_TIMEOUT_MS (30*60*1000)

// Brew finish timeout
#define CONFIG_BREW_FINISH_TIMEOUT_MS (60*1000)

#define CONFIG_FULL_BRIGHTNESS (1.0f)
#define CONFIG_IDLE_BRIGHTNESS (0.2f)

#define CONFIG_WAIT_FOR_WIFI (false)