#pragma once

/// @brief Handles WiFi network connection
namespace Network {

    /// @brief Set up and connect to a WiFi network
    void initWiFi();

    /// @brief Keep network connection alive
    void handle();
    
    /// @brief true if currently attempting to connect to a network
    bool isConnecting();

    /// @brief true if connected to a network
    static bool isConnected() {
        return ((WiFi.getMode() & WIFI_MODE_STA) && (WiFi.status() == WL_CONNECTED));
    }
}