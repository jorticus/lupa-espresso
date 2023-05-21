#pragma once

// Target boiler temperature
// 115C corresponds to a grouphead temperature of ~95C
#define CONFIG_BOILER_TEMPERATURE_C (115.0f)

// Preheat end temperature
// Defines when to exit the preheat phase and become ready.
// Does not control actual target temperature.
#define CONFIG_BOILER_PREHEAT_TEMPERATURE_C (90.0f)

// Maximum boiler temperature
#define CONFIG_MAX_BOILER_TEMPERATURE_C (123.0f)
