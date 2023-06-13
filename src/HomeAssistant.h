#pragma once

namespace HomeAssistant {
    void init();
    void process();

    void publishData(const char* topic, const char* payload);
}