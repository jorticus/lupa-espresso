#pragma once

#include <stdint.h>
#include "BrewControl.h"
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
    SetPressure(float target) : target(target)
    { }

    void activate() const override {
        BrewControl::setPressure(target);
    }

    void print() const override { Debug.printf("SetPressure: %.1f\n", target); }

protected:
    const float target;
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
        if (state.setPressure >= target) {
            BrewControl::setPressure(target);
            return true; // Advance stage
        } else {
            BrewControl::setPressure(state.setPressure + rate);
            return false; // Do not advance
        }
    }

    void print() const override { Debug.printf("RampPressure: to %.1f @ %.1f/s\n", target, rate*10.0f); }

protected:
    const float target;
    const float rate;
};

/// @brief Action to set the flow rate
/// Advances to next stage automatically.
class SetFlowRate : public StageBase {
public:
    SetFlowRate(float target) : target(target) { }

    void activate() const override {
        BrewControl::setFlowRate(target);
    }

protected:
    const float target;
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

    void print() const override { Debug.printf("WaitUntil: %d s\n", timestamp_sec); }

protected:
    const uint32_t timestamp_sec;
};

/// @brief Custom conditional using lambda function
/// Blocks until condition is met.
class Conditional : public StageBase {
public:
    Conditional(std::function<bool(const State& state)> conditional)
        : _conditional(conditional) { }

    bool step(const State& state) const override {
        return _conditional(state);
    }

    void print() const override { Debug.printf("Condition\n"); }

protected:
    const std::function<bool(const State& state)> _conditional;
};

/// @brief Indicate that the brew has finished, regardless of whether the lever is still pulled
// class EndBrew : public StageBase {

// };

using Stage = std::variant<
    SetPressure,
    RampPressure,
    SetFlowRate,
    WaitUntil,
    Conditional
>;

using Profile = std::span<const Stage>;
using ProfilesList = std::vector<std::tuple<std::string, Profile>>;

}
