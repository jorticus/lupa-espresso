#include <Arduino.h>
#include "WiFi.h"
#include "Display.h"
#include "UI.h"
#include "Debug.h"
#include "secrets.h"

using namespace Display;

namespace Network
{

static int s_reconnectAttempts = 0;

enum class WifiConnectionStatus {
    Connecting,
    Failure,
    Connected
};

/// @brief Render and update the display with current WiFi connection status.
/// @param state WiFi status
/// @param err Error code if failed to connect
static void uiRenderWiFiConnect(WifiConnectionStatus state, wl_status_t err = (wl_status_t)0) {
    tftClearCanvas();

    UI::Widgets::uiRenderLabelCentered(gfx_right, "WIFI CONNECT", -60, TFT_WHITE);

    switch (state) {
        case WifiConnectionStatus::Connecting:
        {
            UI::Widgets::uiRenderLabelCentered(gfx_right, "CONNECTING TO", 0, TFT_WHITE);
            UI::Widgets::uiRenderLabelCentered(gfx_right, secrets::wifi_ssid, 30, TFT_LIGHTGREY);
            break;
        }
            break;

        case WifiConnectionStatus::Connected:
        {
            auto ip_str = WiFi.localIP().toString();
            UI::Widgets::uiRenderLabelCentered(gfx_right, "CONNECTED", 0, TFT_WHITE);
            UI::Widgets::uiRenderLabelCentered(gfx_right, ip_str.c_str(), 30, TFT_LIGHTGREY);
            break;
        }

        case WifiConnectionStatus::Failure:
        {
            UI::Widgets::uiRenderLabelCentered(gfx_right, "ERROR", 0, TFT_WHITE);

            const char* err_msg = nullptr;
            switch (err) {
                case WL_NO_SSID_AVAIL:
                    // Bad SSID or AP not present
                    err_msg = "No SSID";
                    break;
                case WL_CONNECT_FAILED:
                    err_msg = "Connect Failed";
                    break;
                case WL_IDLE_STATUS:
                    err_msg = "Station Idle";
                    break;
                case WL_CONNECTION_LOST:
                    err_msg = "Connection Lost";
                    break;
            }

            if (err_msg != nullptr) {
                UI::Widgets::uiRenderLabelCentered(gfx_right, err_msg, 30, TFT_LIGHTGREY);
            }
            break;
        }
    }

    tftUpdateDisplay();
}

#include "esp_wifi.h"

static void startWiFi()
{
    // // Erase prior configuration
    WiFi.disconnect(true, true);
    // WiFi.mode(WIFI_OFF);

    // // Store config in RAM instead of committing to flash
    // esp_wifi_set_storage(WIFI_STORAGE_RAM);

    // // Workaround to disable persistent config
    //wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // cfg.nvs_enable = 0;

    //WiFi.persistent(true);
    WiFi.persistent(false); // Store config in RAM
    
    WiFi.setHostname(secrets::device_name);
    WiFi.setAutoReconnect(true);

    // Enable WiFi Station Mode
    if (!WiFi.mode(WIFI_STA)) {
        Debug.println("ERROR: Could not enable WiFi STA");
    }

    // Disable power saving to keep connection stable
    WiFi.setSleep(WIFI_PS_NONE);

    //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    
    auto status = WiFi.begin(secrets::wifi_ssid, secrets::wifi_pw);
    if (status == WL_CONNECT_FAILED) {
        Debug.println("ERROR: Could not start WiFi");
        //WiFi.reconnect();
    }
}

void initWiFi()
{
    wl_status_t result;

    //WiFi.persistent(false);

#if CONFIG_WAIT_FOR_WIFI
    while (WiFi.status() != WL_CONNECTED) {
        uiRenderWiFiConnect(WifiConnectionStatus::Connecting);
        Debug.println();

        startWiFi();
        Debug.printf("Connecting to SSID: %s\n", secrets::wifi_ssid); 

        result = (wl_status_t)WiFi.waitForConnectResult();
        if (result != WL_CONNECTED) {
            uiRenderWiFiConnect(WifiConnectionStatus::Failure, result);

            Debug.println("WiFi Diagnostics:");
            WiFi.printDiag(Serial);
            Debug.println();

            delay(1000);
        }
    }

    uiRenderWiFiConnect(WifiConnectionStatus::Connected);

    Debug.print("IP address: ");
    Debug.println(WiFi.localIP());

    delay(1500);
#else
    Debug.println("Starting WiFi...");
    startWiFi();
    
    //Debug.printf("Connecting to SSID: %s\n", secrets::wifi_ssid); 
    //WiFi.waitForConnectResult();
#endif
}

static const char* wifi_status_str[] = {
    "Idle",
    "No SSID",
    "Scan Completed",
    "Connected",
    "Connect Failed",
    "Connection Lost",
    "Disconnected"
};

void handle() {
    static unsigned long t_last = 0;
    static wl_status_t last_status = WL_DISCONNECTED;
    const unsigned long reconnect_interval_ms = 20000;

    auto status = WiFi.status();
    if (last_status != status) {
        last_status = status; 
        Debug.write("WiFi: ");
        Debug.println(wifi_status_str[(int)status]);
    }

#if true
    if (!(WiFi.getMode() & WIFI_MODE_STA) || 
        (status == WL_NO_SSID_AVAIL) || 
        (status == WL_CONNECT_FAILED) || 
        (status == WL_CONNECTION_LOST) ||
        (status == WL_DISCONNECTED) || 
        (status == WL_IDLE_STATUS)) // Connecting
    {
        // Throttle reconnection attempts
        if ((millis() - t_last) > reconnect_interval_ms) {
            t_last = millis();

            if (status != WL_DISCONNECTED) {
                Debug.println("WiFi connection lost, reconnecting...");
            }

            WiFi.reconnect();

            s_reconnectAttempts++;
            if (s_reconnectAttempts > 6) {
                // After 1 minute, fall back to AP mode
                WiFi.disconnect();
                WiFi.softAP("LUPA", "espresso");
            }
        }
    }
    else
    {
        s_reconnectAttempts = 0;
    }
#endif
}

bool isConnecting() {
    if (WiFi.getMode() & WIFI_MODE_STA) {
        auto wifiStatus = WiFi.status();
        //if (wifiStatus == WL_IDLE_STATUS || wifiStatus >= WL_DISCONNECTED) {
        if (wifiStatus == WL_IDLE_STATUS) {
            return true;
        }
    }
    return false;
}

bool isConnected() {
    if (WiFi.getMode() & WIFI_MODE_STA) {
        auto wifiStatus = WiFi.status();
        return (wifiStatus == WL_CONNECTED);
    }
    return false;
}

}