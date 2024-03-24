#pragma once
#include <Arduino.h>

extern Stream& Debug;

namespace DebugLogger {

    void init();
    void process();
}
