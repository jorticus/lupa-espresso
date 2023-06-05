#pragma once

namespace HomeAssistant {
    void init();
    void process();

    void reportPowerControlState(bool pwr);
}