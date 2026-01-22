#include <array>
#include <span>
#include "BrewControl.h"
#include "BrewProfiles.h"
#include "MqttParamManager.h"

using namespace ProfileDefs;

namespace BrewControl {
    // User controlled pressure/flowrate targets
    extern MqttParam::Parameter<float> param_brewPressure;
    extern MqttParam::Parameter<float> param_brewFlowRate;
}

#if 0

// Full Manual Control
// Pump @ 100% allowing maximum allowable pressure
static const std::array<Stage, 0> s_profileFullManual = {
    // Do nothing. 
};

// Flat Pressure (Equivalent to BrewMode::ManualPressure)
static const std::array<Stage, 1> s_profileFlatPressure = {
    SetPressure(BrewControl::param_brewPressure)
};

// Flat Flow Rate (Equivalent to BrewMode::ManualFlow)
static const std::array<Stage, 1> s_profileFlatFlowRate = {
    SetFlowRate(BrewControl::param_brewFlowRate)
};


// Londinium (Light & Dark roast, 1:2 ratio)
// 3 bar, until drops appear (assuming typically 5 seconds)
// ramp to 9 bar until last 3rd
// 6 bar finish
static const std::array<Stage, 5> s_profileLondinium = {
    SetFlowRate(3.0f),
    WaitUntil(5.0f),
    RampPressure(9.0f, 0.1f),
    WaitUntil(20.0f),
    SetPressure(6.0f)
};


// Blooming espresso (grind finer, 1:2, 1:2.5 ratio)
// 6 bar preinfusion, cut flow, wait 30 sec
// 8-9 bar 
static const std::array<Stage, 5> s_profileBloom = {
    SetPressure(6.0f),
    WaitUntil(5.0f),
    SetPressure(0.0f),
    // SetFlowRate(0.0f), // Cut flow
    WaitUntil(30.0f),
    SetPressure(BrewControl::param_brewPressure)
};

// Declining profile
// (Dark roast, 1:2 ratio or less, reduces astringency)
// 8-9 bar @ 7mL/s
// slowly taper off throughout the shot
static const std::array<Stage, 3> s_profileDeclining = {
    SetPressure(BrewControl::param_brewPressure),  // Start at 9 Bar
    WaitUntil(3.0f),            // Wait for built-in preinfusion
    RampPressure(1.0f, -0.027f) // Ramp down to 1Bar (Aiming for total 30 sec)
};

// Slayer shot (1:2 - 1:3 ratio)
// preinfuse 1-2mL/s until pressure raises (not when drips appear)
// ramp up to 8-9 bars
// optionally taper to 6 bar
static const std::array<Stage, 6> s_profileSlayerShot = {
    Conditional([](auto& state) { return state.currPressure < 2.0f; }), // Wait for lever to depressurize system
    SetFlowRate(2.0f), // TODO: Flow regulation not yet working correctly
    Conditional([](auto& state) { return state.currPressure >= 3.0f; }),
    RampPressure(9.0f, +0.1f), // Ramp up to 9 Bar @ 1 Bar/sec (0.1Bar / 100ms)
    WaitUntil(10.0f), // 25 sec?
    RampPressure(6.0f, -0.1f), // Ramp down to 6 Bar
};

// Allonge (1:5 - 1:7 ratio) - Filter style (light roasts, grind coarser)
// Preinfuse for 3-6 sec
// Peak 8 bar @ 4.5mL/s
// Continue 4.5mL/s for 35-40s
static const std::array<Stage, 3> s_profileAllonge = {
    SetPressure(3.0f),
    WaitUntil(6.0f),
    // SetPressure(8.0f),
    SetFlowRate(4.5f) // TODO: Should we do a hybrid where it does either up to 8 bar or limit flow?
};

#endif

/// @brief Strings to show in UI selector
const ProfilesList s_profiles = {
    // { "Manual",     s_profileFullManual },
    // { "Fixed Pressure",  s_profileFlatPressure },
    // { "Fixed Flow", s_profileFlatFlowRate },
    // { "Londinium",  s_profileLondinium },
    // { "Bloom",      s_profileBloom },
    // { "Declining",  s_profileDeclining },
    // { "Slayer",     s_profileSlayerShot },
    // { "Allonge",    s_profileAllonge },
};

const std::vector<std::string> s_profileNames = {
    "Manual", "Fixed Pressure", "Fixed Flow",
    // "Londinium", "Bloom", "Declining", "Slayer", "Allonge"
};

// static constexpr std::vector<std::string> s_profileNames = 