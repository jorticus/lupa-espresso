#include <Arduino.h>
#include "WiFi.h"
#include "Display.h"
#include "UI.h"
#include "secrets.h"

using namespace Display;

namespace Network
{

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

static void startWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(WIFI_PS_NONE);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname(secrets::device_name);
    WiFi.begin(secrets::wifi_ssid, secrets::wifi_pw);
}

void initWiFi()
{
    wl_status_t result;

    WiFi.persistent(false);

#if CONFIG_WAIT_FOR_WIFI
    while (WiFi.status() != WL_CONNECTED) {
        uiRenderWiFiConnect(WifiConnectionStatus::Connecting);
        Serial.println();

        startWiFi();
        Serial.printf("Connecting to SSID: %s\n", secrets::wifi_ssid); 

        result = (wl_status_t)WiFi.waitForConnectResult();
        if (result != WL_CONNECTED) {
            uiRenderWiFiConnect(WifiConnectionStatus::Failure, result);

            Serial.println("WiFi Diagnostics:");
            WiFi.printDiag(Serial);
            Serial.println();

            delay(1000);
        }
    }

    uiRenderWiFiConnect(WifiConnectionStatus::Connected);

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    delay(1500);
#else
    Serial.println("Connecting WiFi...");
    startWiFi();
    
    //Serial.printf("Connecting to SSID: %s\n", secrets::wifi_ssid); 
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
    const unsigned long reconnect_interval_ms = 10000;

    auto status = WiFi.status();
    if (last_status != status) {
        last_status = status; 
        Serial.write("WiFi: ");
        Serial.println(wifi_status_str[(int)status]);
    }

    if (!(WiFi.getMode() & WIFI_MODE_STA) || 
        (status == WL_NO_SSID_AVAIL) || 
        (status == WL_CONNECT_FAILED) || 
        (status == WL_CONNECTION_LOST) ||
        (status == WL_DISCONNECTED))
    {
        // Throttle reconnection attempts
        if ((millis() - t_last) > reconnect_interval_ms) {
            t_last = millis();
            //WiFi.mode(WIFI_OFF);
            WiFi.disconnect(true, false);
            if (status != WL_DISCONNECTED) {
                Serial.println("WiFi connection lost, reconnecting...");
            }
            startWiFi();
        }
    }
}

bool isConnecting() {
    if (WiFi.getMode() & WIFI_MODE_STA) {
        auto wifiStatus = WiFi.status();
        if (wifiStatus == WL_IDLE_STATUS || wifiStatus >= WL_DISCONNECTED) {
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