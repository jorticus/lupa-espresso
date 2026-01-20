#pragma once

#include <array>
#include <span>
#include "BrewControl.h"
#include "ProfileDefs.h"

using namespace ProfileDefs;

// typedef struct {
//     /// @brief Time at which this point shall trigger
//     uint32_t time_sec;
//     /// @brief Pressure target for this time point
//     float    pressure;
//     /// @brief Flow target for this time point
//     // float    flow;
// } PressurePoint;

// // Londinium (Light & Dark roast, 1:2 ratio)
// // 3 bar, until drops appear (assuming typically 5 seconds)
// // ramp to 9 bar to end 3rd
// // 6 bar finish
// static const std::array<PressurePoint, 3> s_dynProfile1 = {
//     PressurePoint { 0,  2.5f },  // 5 sec @ 2.5 bar preinfusion
//     PressurePoint { 5,  9.0f }, // 15 sec @ 9.0 bar extraction
//     PressurePoint { 20, 6.0f }, // ramp down to 6.0 bar post-extraction
// };

// // Lever-style brew profile (light/medium roast)
// static const std::array<PressurePoint, 4> s_dynProfile2 = {
//     PressurePoint { 0,  1.5f },  // 10 sec @ 1.5 bar preinfusion
//     PressurePoint { 10, 4.0f },  // 10 sec @ 4 bar
//     PressurePoint { 20, 9.0f },  // 20 sec @ 9 bar
//     PressurePoint { 40, 6.0f }   // ramp down to 6 bar
// };

// static const std::array<std::span<const PressurePoint>, 2> s_dynProfiles = {
//     s_dynProfile1, s_dynProfile2
// };

// static const std::array<PressurePoint, 3> s_dynProfile3 = {

// };


// Londinium (Light & Dark roast, 1:2 ratio)
// 3 bar, until drops appear (assuming typically 5 seconds)
// ramp to 9 bar to end 3rd
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
    SetFlowRate(0.0f), // Cut flow
    WaitUntil(30.0f),
    SetPressure(9.0f)
};

// Declining profile
// (Dark roast, 1:2 ratio or less, reduces astringency)
// 8-9 bar @ 7mL/s
// slowly taper off throughout the shot
static const std::array<Stage, 3> s_profileDeclining = {
    SetPressure(9.0f),          // Start at 9 Bar
    WaitUntil(3.0f),            // Wait for built-in preinfusion
    RampPressure(1.0f, -0.027f) // Ramp down to 1Bar (Aiming for total 30 sec)
};

// Slayer shot (1:2 - 1:3 ratio)
// preinfuse 1-2mL/s until pressure raises (not when drips appear)
// ramp up to 8-9 bars
// optionally taper to 6 bar
static const std::array<Stage, 5> s_profileSlayerShot = {
    SetFlowRate(2.0f),
    Conditional([](auto& state) { return state.currPressure >= 3.0f; }),
    RampPressure(9.0f, +0.1f), // Ramp up to 9 Bar @ 1 Bar/sec (0.1Bar / 100ms)
    WaitUntil(25.0f),
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

static const ProfilesList s_profiles = {
    { "Londinium", s_profileLondinium }
};