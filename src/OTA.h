#pragma once

/// @brief Handles Over-The-Air firmware updates.
namespace OTA {
    
    /// @brief Set up Over-The-Air updates.
    /// Requires WiFi to be connected.
    void initOTA();

    /// @brief Handle OTA updates. Must be called repeatedly.
    void handle();

}
