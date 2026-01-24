#pragma once

/// @brief Provides an HTTP web server for displaying sensor data
namespace WebSrv {
    /// @brief Initialize the HTTP web server
    void setup();

    /// @brief Shut down the web server
    void stop();

    /// @brief Handle incoming HTTP traffic
    void process();
}
