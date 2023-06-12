#include <Arduino.h>
#include <WiFi.h>
#include "UI.h"
#include "StateMachine.h"
#include "UIWidgets.h"
#include "Display.h"
#include "SensorSampler.h"
#include "HeatControl.h"
#include "IO.h"
#include "Network.h"
#include "value_array.h"
#include "secrets.h"
#include "config.h"
#include "hardware.h"

using namespace Display;

constexpr float deg2rad      = 3.14159265359/180.0;

const int numSamples = 300;
extern ValueArray<float, numSamples> temperatureSamples;
extern ValueArray<float, numSamples> pressureSamples;
extern ValueArray<float, numSamples> flowSamples;

// Shadow copy of above sample arrays so we can freeze the sensor data
ValueArray<float, numSamples> pressureSamplesFrozen;
ValueArray<float, numSamples> flowSamplesFrozen;

const uint16_t COLOUR_BOILER_T = TFT_RED;
const uint16_t COLOUR_PRESSURE = TFT_DARKGREEN;
const uint16_t COLOUR_BOILER_POWER = TFT_YELLOW;
const uint16_t COLOUR_FLOW_RATE = TFT_RGB656(100,100,235); // Blue

namespace UI {

using namespace State;
using namespace UI::Widgets;


void uiFreezeGraphs() {
    for (int i = 0; i < pressureSamples.size(); i++) {
        pressureSamplesFrozen.add(pressureSamples[i]);
    }
    for (int i = 0; i < flowSamples.size(); i++) {
        flowSamplesFrozen.add(flowSamples[i]);
    }
}


void uiGetRadialCoords(int32_t r, float angle, int32_t *x, int32_t *y) {
    angle = 180 + angle;
    *x = (TFT_WIDTH/2)  + (r * -sinf(angle * deg2rad));
    *y = (TFT_HEIGHT/2) + (r * +cosf(angle * deg2rad));
}

void uiRenderWiFiStatus(GfxCanvas& gfx) {
    const int32_t r = (TFT_WIDTH/2) - 40;
    int32_t x, y;

    // WiFi connection indicator
    // TODO: Replace with WiFi icons
    int wifiColor = TFT_YELLOW;
    if (Network::isConnected()) {
        wifiColor = TFT_DARKGREEN;
    } else if (Network::isConnecting()) {
        float f = sinf((millis() * 0.5f) * deg2rad + PI) * 0.5f + 0.5f;
            int16_t c = f*127.0f + 127.0f;
            wifiColor = TFT_RGB656(0, 0, c);
            //wifiColor = TFT_SKYBLUE;
    }

    // auto ip_str = WiFi.localIP().toString();
    // uiRenderLabelCentered(gfx_left, ip_str.c_str(), 30, TFT_LIGHTGREY);
    const char* err_msg = nullptr;
    switch (WiFi.status()) {
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
        uiRenderLabelCentered(gfx_left, err_msg, 0);
    }

    uiGetRadialCoords(r, 22, &x, &y);
    gfx.fillCircle(x, y, 5, wifiColor);
}

void uiRenderStatusIcons(GfxCanvas& gfx) {
    const int32_t r = (TFT_WIDTH/2) - 40;
    int32_t x, y;

    // Boiler heater indicator
    if (IO::isHeaterOn()) {
        // TODO: Replace with heat icon
        uiGetRadialCoords(r, -45, &x, &y);
        gfx.fillCircle(x, y, 5, TFT_ORANGERED);
    }

    // WiFi connection status
    uiRenderWiFiStatus(gfx);

    // Boiler heat mode
    uint32_t color = TFT_BLACK;
    auto mode = HeatControl::getMode();
    switch (mode) {
        case HeatControl::Mode::Brew:
            color = TFT_RED;
            break;
        case HeatControl::Mode::Steam:
            color = TFT_SKYBLUE;
            break;
        case HeatControl::Mode::Sleep:
            color = TFT_DARKCYAN;
            break;
    }
    uiGetRadialCoords(r, +45, &x, &y);
    gfx.fillCircle(x, y, 5, color);
}

void uiRenderPressureGauge(GfxCanvas& gfx) {
    float min_pressure = 1.0f; // Ambient air pressure
    float max_pressure = PRESSURE_MAX_BAR;
    float value_norm = (SensorSampler::getPressure() - min_pressure) / (max_pressure - min_pressure);
    uiRenderGauge(gfx, value_norm, COLOUR_PRESSURE);

    uiRenderLabelFormattedCentered(gfx,
        60,
        TFT_WHITE,
        (SensorSampler::isPressureValid() ? "%.2f Bar" : "- Bar"),
        SensorSampler::getPressure());
}

float getTemperatureMaxRange() {
    return 140.0f;
}
float getTemperatureMinRange() {
    if ((uiState != MachineState::Preheat) && (SensorSampler::getTemperature() >= 80.0f)) {
        // Use smaller range when up to temperature
        return 80.0f;
    }
    else {
        return 20.0f;
    }
}

void uiRenderTemperatureGauge(GfxCanvas& gfx) {
    float temperature = SensorSampler::getTemperature();
    float min_temp = getTemperatureMinRange();
    float max_temp = getTemperatureMaxRange();
    float value_norm = (temperature - min_temp) / (max_temp - min_temp);
    uiRenderGauge(gfx, value_norm, COLOUR_BOILER_T);
}

void uiRenderHeaterPowerGauge(GfxCanvas& gfx) {
    uiRenderGauge(gfx, IO::getHeatPower(), COLOUR_BOILER_POWER, 15);
}

void uiRenderFlowGauge(GfxCanvas& gfx) {
    float flow = SensorSampler::getFlowRate();
    float min_flow = 0.0f;
    float max_flow = FLOW_MAX_VALUE;
    float value_norm = (flow - min_flow) / (max_flow - min_flow);
    uiRenderGauge(gfx, value_norm, COLOUR_FLOW_RATE, 15);
}

void uiRenderTemperatureGraph(GfxCanvas& gfx, uint16_t color = TFT_WHITE) {
    float min_value = getTemperatureMinRange();
    float max_value = getTemperatureMaxRange();
    uiRenderGraph(gfx, temperatureSamples, min_value, max_value, color);
}

void uiRenderBrewGraph(GfxCanvas& gfx, bool freeze = false) {
    {
        float min_value = 0.0f;
        float max_value = PRESSURE_MAX_BAR;
        auto& samples = (freeze) ? pressureSamplesFrozen : pressureSamples;

        uiRenderGraph(gfx, samples, min_value, max_value, COLOUR_PRESSURE);
    }

    {
        float min_value = 0.0f;
        float max_value = FLOW_MAX_VALUE;
        auto& samples = (freeze) ? flowSamplesFrozen : flowSamples;

        uiRenderGraph(gfx, samples, min_value, max_value, COLOUR_FLOW_RATE);
    }
}

void uiRenderPostBrewScreen(GfxCanvas& gfx)
{
    // Display snapshot of last brew graph
    uiRenderBrewGraph(gfx, true);

    float brewTimeSec = (brewStats.end_brew_time - brewStats.start_brew_time) / 1000.0f;

    int16_t y = (-(TFT_HEIGHT/2) + 60);
    uiRenderLabelCentered(gfx, y, TFT_WHITE, "BREW TIME:");

    y += 30;
    uiRenderLabelFormattedCentered(gfx, y, TFT_WHITE, "%.1f", brewTimeSec);
}

void uiRenderReadyScreen(GfxCanvas& gfx)
{
    uiRenderLabelCentered(gfx, "READY", 0);
    //uiRenderBrewGraph(gfx_right);
}

void uiRenderBrewingScreen(GfxCanvas& gfx) {

    uiRenderLabelCentered(gfx, 0, TFT_WHITE, "BREWING");

    float brewTimeSec = (millis() - brewStats.start_brew_time) / 1000.0f;
    uiRenderLabelFormattedCentered(gfx, 
        20, 
        TFT_WHITE,
        "%.1f", 
        brewTimeSec);

    uiRenderLabelFormattedCentered(gfx, 
        60, 
        TFT_WHITE,
        (SensorSampler::isPressureValid() ? "%.1f Bar" : "- Bar"), 
        SensorSampler::getPressure());

    // {
    //     const char* status_str = "BREWING";
    //     int16_t tw = gfx.textWidth(status_str);
    //     int16_t th = 14;
    //     gfx.setCursor(TFT_WIDTH/2 - tw/2, TFT_HEIGHT/2 - th/2);
    //     gfx.print(status_str);
    // }
}

/// @brief Render sensors for debugging
void uiRenderSensorTest()
{
    uiRenderTemperatureGraph(gfx_left, TFT_RED);
    uiRenderBrewGraph(gfx_right);

    uiRenderLabelFormattedCentered(gfx_right, 
        -60,
        TFT_DARKGREEN,
        (SensorSampler::isPressureValid() ? "%.2f Bar" : "- Bar"),
        SensorSampler::getPressure());

    uiRenderLabelFormattedCentered(gfx_right, 
        -30,
        TFT_SKYBLUE,
        (SensorSampler::isFlowRateValid() ? "%.1f /s" : "- /s"),
        SensorSampler::getFlowRate());

    uiRenderLabelFormattedCentered(gfx_right, 
        0,
        TFT_DARKCYAN,
        (SensorSampler::isFlowRateValid() ? "%.1f mL" : "- mL"),
        SensorSampler::getTotalFlowVolume());

    uiRenderPressureGauge(gfx_right);
    uiRenderFlowGauge(gfx_right);

    uiRenderLabelFormattedCentered(gfx_left, 
        0,
        TFT_RED,
        (SensorSampler::isFlowRateValid() ? "%.2f C" : "- C"),
        SensorSampler::getTotalFlowVolume());

    uiRenderTemperatureGauge(gfx_left);
}

/// @brief Render the left UI
void renderLeft() {
    auto& gfx = gfx_left;

    uiRenderTemperatureGraph(gfx);

    float t_boiler = SensorSampler::getTemperature();

    // Render current temperature
    bool is_t_valid = SensorSampler::isTemperatureValid() && (t_boiler > 20.0f);
    uiRenderLabelFormattedCentered(gfx,
        (TFT_HEIGHT/2 - 60),
        TFT_WHITE,
        (is_t_valid ? "%.2f C" : "- C"), 
        t_boiler);

    // Estimated grouphead temperature
    bool is_est_valid = SensorSampler::isTemperatureValid() && (t_boiler > 100.0f);
    if (is_est_valid) {
        float t_estimated = SensorSampler::getEstimatedGroupheadTemperature();
        uiRenderLabelFormattedCentered(gfx,
            (TFT_HEIGHT/2 - 30),
            TFT_WHITE,
            "%.0f C",
            t_estimated);
    }

    uiRenderTemperatureGauge(gfx);
    uiRenderHeaterPowerGauge(gfx);
    uiRenderStatusIcons(gfx);
}

void uiRenderStatusRing(GfxCanvas& gfx, const char* message, uint16_t color, float ring_w) {
    if (color != TFT_BLACK) {
        gfx.drawSmoothArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-ring_w, 0, 360, color, TFT_BLACK);
    }

    if (message != nullptr) {
        int16_t tw = gfx.textWidth(message);
        int16_t th = 14;
        gfx.setCursor(TFT_WIDTH/2 - tw/2, TFT_HEIGHT/2 - th/2);
        gfx.print(message);
    }
}

/// @brief Render the right UI
void renderRight() {
    auto& gfx = gfx_right;

    uint32_t ring_w = 10;
    unsigned long t = millis();
    const char* status_str = nullptr;
    uint32_t color = TFT_BLACK;

    // Render UI elements for the current state
    switch (uiState) {
        case MachineState::Init:
            uiRenderStatusRing(gfx, "INIT", TFT_BLUE, ring_w);
            break;

        case MachineState::Preheat:
            // Pulse animation
            ring_w += 5 + (sinf((t * 0.1f) * deg2rad + PI) * 5.0f);
            uiRenderStatusRing(gfx, "WARMING UP", TFT_ORANGERED, ring_w);
            break;

        case MachineState::Fault:
        {
            switch (uiFault) {
                case FaultState::LowWater:
                    status_str = "FILL WATER";
                    break;
                case FaultState::NotHeating:
                    status_str = "NOT HEATING";
                    break;
                case FaultState::OverTemp:
                    status_str = "OVER TEMP";
                    break;
                case FaultState::SensorFailure:
                    status_str = "SENSOR FAILURE";
                    break;
                case FaultState::FirmwareUpdateFailure:
                    status_str = "UPDATE FAILED";
                    break;
                default:
                    status_str = "FAULT";
                    break;
            }

            // Flash animation
            float f = sinf((t * 0.5f) * deg2rad + PI) * 0.5f + 0.5f;
            int16_t c = f*f*255.0f;
            color = TFT_RGB656(c, 0, 0);

            uiRenderStatusRing(gfx, status_str, color, ring_w);
            break;
        }

        case MachineState::Ready:
            if (brewStats.start_brew_time > 0) {
                // Show post-brew snapshot graph + brew time
                uiRenderPostBrewScreen(gfx);
            }
            else {
                uiRenderBrewGraph(gfx);
                uiRenderReadyScreen(gfx);
            }

            uiRenderPressureGauge(gfx);
            uiRenderFlowGauge(gfx);
            break;

        case MachineState::Brewing:
            uiRenderBrewGraph(gfx);
            uiRenderBrewingScreen(gfx);
            
            uiRenderPressureGauge(gfx);
            uiRenderFlowGauge(gfx);
            break;

        case MachineState::Sleep:

            // Pulse animation
            ring_w += 5 + (sinf((t * 0.1f) * deg2rad + PI) * 5.0f);

            uiRenderStatusRing(gfx, "ZzZz", TFT_DARKCYAN, ring_w);
            break;
    }
}

void render() {
    auto t1 = millis();

    gfx_left.fillSprite(TFT_BLACK);
    gfx_right.fillSprite(TFT_BLACK);

    if (uiState == MachineState::SensorTest) {
        uiRenderSensorTest();
        return;
    }

    renderLeft();
    renderRight();

    auto t2 = millis();

    tftUpdateDisplay();

    auto t3 = millis();

    //Serial.printf("Render %dms Update %dms\n", (t2-t1), (t3-t2));
}


}