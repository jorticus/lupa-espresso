#pragma once

/// @brief Provides an HTTP web server for displaying sensor data
namespace WebSrv {
    /// @brief Initialize the HTTP web server
    void setup();

    /// @brief Handle incoming HTTP traffic
    void process();
}
