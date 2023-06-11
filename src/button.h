#pragma once
#include <Arduino.h>
#include <functional>

template <int _pin, uint8_t _level = HIGH>
struct Btn {
public:
    static constexpr int pin = _pin;
    static constexpr uint8_t level = _level;
};


// Variadic buttons template
// Usage:
// Buttons<Btn<BTN1_PIN>, Btn<BTN1_PIN>> buttons;
// if (buttons[0]) { ... }
template <typename ... buttons>
class Buttons {
    typedef std::function<void(int)> onButton_t;

private:
    static constexpr int N = sizeof...(buttons);
    int      pins[N]        = { buttons::pin... };
    uint8_t  levels[N]      = { buttons::level... };
    bool     states[N]      = { 0 };
    uint32_t ts[N]          = { 0 };

    onButton_t cb_onButtonPress;
    onButton_t cb_onButtonRelease;

public:
    Buttons() :
        cb_onButtonPress(),
        cb_onButtonRelease()
    { }

    // Initialize button GPIOs
    void init() {
        for (int i = 0; i < N; i++) {
            pinMode(pins[i], INPUT);
        }
    }

    // De-bounce buttons
    // https://pubweb.eng.utah.edu/~cs5780/debouncing.pdf
    void process(int press_interval_ms = 10, int release_interval_ms = 10) {
        for (int i = 0; i < N; i++) {
            auto& state = states[i];
            auto& lastTime = ts[i];
            const bool currState = (digitalRead(pins[i]) == levels[i]) ? true : false;

            if (currState == state) {
                lastTime = millis() + ((state) ? 
                    press_interval_ms :   // Press counter
                    release_interval_ms); // Release counter
            }
            else {
                // State has changed - wait for signal to become stable (rollover-safe)
                if ((millis() - lastTime) >= 0) {
                    state = currState;
                    if (state) {
                        if (cb_onButtonPress) cb_onButtonPress(i);
                    } else {
                        if (cb_onButtonRelease) cb_onButtonRelease(i);
                    }

                    // Reset the timer
                    lastTime = millis() + ((state) ? 
                        press_interval_ms :   // Press counter
                        release_interval_ms); // Release counter
                }
            }
        }
    }

    void onButtonPress(onButton_t cb) {
        cb_onButtonPress = cb;
    }
    void onButtonRelease(onButton_t cb) {
        cb_onButtonRelease = cb;
    }

    // The current button state
    bool operator[] (int i) {
        if (i >= 0 && i < N) {
            return states[i];
        } else {
            return false;
        }
    }
};

// Convenience wrapper - assumes all buttons are active HIGH
// Usage:
// Buttons<BTN1_PIN, BTN2_PIN>
// template <int ... pin>
// class Buttons : public Buttons<Btn<pin..., HIGH>> { };