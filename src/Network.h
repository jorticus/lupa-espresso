#pragma once

/// @brief Handles WiFi network connection
namespace Network {

    /// @brief Set up and connect to a WiFi network
    void initWiFi();

    void handle();
    
    bool isConnecting();
    bool isConnected();
}