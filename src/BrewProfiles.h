#pragma once

#include <stdint.h>
#include "BrewControl.h"
#include "IO.h"
#include "MqttParamManager.h"
#include "Debug.h"

#include <functional>
#include <variant>
#include <string>
#include <span>
#include <tuple>


namespace ProfileDefs {

class State {
public:
    uint32_t timeElapsed;
    float currPressure;
    float setPressure;
    float currFlowRate;
    float setFlowRate;
};

/// @brief Stage base class for defining a brew profile
class StageBase {
public:
    /// @brief Exeucted when stage first triggered
    virtual void activate() const {
    }

    /// @brief Executed every PID step (~100ms)
    /// @param state Current state of the PID control loop
    /// @return Whether to advance to the next stage
    virtual bool step(const State& state) const {
        return true; // advance to next action by default
    }

    virtual void print() const { }
};

/// @brief Action to set the pressure.
/// Advances to next stage automatically.
class SetPressure : public StageBase {
public:
    SetPressure(float target) : 
        get_target{},
        target(target)
    { }

    SetPressure(MqttParam::Parameter<float>& param) : 
        get_target([&param]() -> float { return param.value(); }),
        target(NAN)
    { }

    void activate() const override {
        if (get_target) {
            BrewControl::setPressure(get_target());
        } else {
            BrewControl::setPressure(target);
        }
    }

    void print() const override { 
        Debug.printf("Set Pressure: %.1f\n", (get_target) ? get_target() : target);
    }

protected:
    std::function<float()> get_target;
    const float target;
};

/// @brief Action to set the flow rate
/// Advances to next stage automatically.
class SetFlowRate : public StageBase {
public:
    SetFlowRate(float target) : 
        get_target{},
        target(target)
    { }

    SetFlowRate(MqttParam::Parameter<float>& param) : 
        get_target([&param]() -> float { return param.value(); }),
        target(NAN)
    { }

    void activate() const override {
        if (get_target) {
            BrewControl::setFlowRate(get_target());
        } else {
            BrewControl::setFlowRate(target);
        }
    }

    void print() const override { 
        Debug.printf("Set Flow Rate: %.1f\n", (get_target) ? get_target() : target);
    }

protected:
    std::function<float()> get_target;
    const float target;
};

/// @brief Action to immediately turn the pump off
class PumpOff : public StageBase {
public:
    PumpOff() { }

    void activate() const override {
        BrewControl::disableOutput();

    }

    void print() const override { 
        Debug.println("Pump Off");
    }
};

/// @brief Action to ramp pressure up or down to the specified target value.
/// Advances to next stage automatically.
class RampPressure : public StageBase {
public:
    RampPressure(float target, float rate) :
        target(target),
        rate(rate)
        { }

    bool step(const State& state) const override {
        if (
            ((rate > 0) && (state.setPressure >= target)) ||
            ((rate < 0) && (state.setPressure <= target))
        ) {
            BrewControl::setPressure(target);
            return true; // Advance stage
        }
        else {
            BrewControl::setPressure(state.setPressure + rate);
            return false; // Do not advance
        }
    }

    void print() const override { Debug.printf("Ramp Pressure to %.1f @ %.1f/s\n", target, rate*10.0f); }

protected:
    const float target;
    const float rate;
};

/// @brief Conditional waiting until time has elapsed.
/// Blocks until condition is met.
class WaitUntil : public StageBase {
public:
    WaitUntil(uint32_t timestamp_sec) :
        timestamp_sec(timestamp_sec)
    { }

    bool step(const State& state) const override {
        return (state.timeElapsed >= this->timestamp_sec);
    }

    void print() const override { Debug.printf("Wait Until: %d s\n", timestamp_sec); }

protected:
    const uint32_t timestamp_sec;
};

/// @brief Conditional waiting until preinfusion has completed (approximately).
/// Blocks until condition is met.
class WaitForPreinfuse : public StageBase {
public:
    WaitForPreinfuse()
    { }

    bool step(const State& state) const override {
        return true; // TODO...
    }

    void print() const override { Debug.printf("Wait Until: Preinfusion\n"); }
};

/// @brief Custom conditional using lambda function
/// Blocks until condition is met.
class Conditional : public StageBase {
public:
    Conditional(std::function<bool(const State& state)> conditional)
        : _conditional(conditional) { }

    bool step(const State& state) const override {
        bool c = _conditional(state);
        Debug.printf("CONDITION: %s\n", c ? "TRUE" : "FALSE");
        return c;
    }

    void print() const override { Debug.printf("Condition\n"); }

protected:
    const std::function<bool(const State& state)> _conditional;
};

/// @brief Indicate that the brew has finished, regardless of whether the lever is still pulled
class EndBrew : public StageBase {
public:
    EndBrew() { }

    void activate() const override {
        BrewControl::endBrew();
    }

    void print() const override { 
        Debug.println("End Brew");
    }
};

using Stage = std::variant<
    // Actions
    SetPressure,
    RampPressure,
    SetFlowRate,
    PumpOff,
    EndBrew,
    // Conditions
    WaitUntil,
    WaitForPreinfuse,
    Conditional
>;

using Profile = std::span<const Stage>;
using ProfilesList = std::vector<std::tuple<std::string, Profile>>;

}

extern const ProfileDefs::ProfilesList s_profiles;
extern const std::vector<std::string> s_profileNames;

namespace BrewProfiles {

    const std::vector<std::string> getProfileNames();

}
