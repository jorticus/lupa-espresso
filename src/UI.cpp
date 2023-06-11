#include <Arduino.h>
#include <WiFi.h>
#include "UI.h"
#include "StateMachine.h"
#include "UIWidgets.h"
#include "Display.h"
#include "SensorSampler.h"
#include "HeatControl.h"
#include "IO.h"
#include "value_array.h"
#include "secrets.h"
#include "config.h"

using namespace Display;

constexpr float deg2rad      = 3.14159265359/180.0;

const int numSamples = 300;
extern ValueArray<float, numSamples> temperatureSamples;
extern ValueArray<float, numSamples> pressureSamples;
extern ValueArray<float, numSamples> flowSamples;

ValueArray<float, numSamples> pressureSamplesFrozen;
ValueArray<float, numSamples> flowSamplesFrozen;



namespace UI {

using namespace State;
using namespace UI::Widgets;

void uiDrawStatusCircle(GfxCanvas& gfx);


void uiFreezeGraphs() {
    for (int i = 0; i < pressureSamples.size(); i++) {
        pressureSamplesFrozen.add(pressureSamples[i]);
    }
    for (int i = 0; i < flowSamples.size(); i++) {
        flowSamplesFrozen.add(flowSamples[i]);
    }
}


void uiDrawStatusCircle(GfxCanvas& gfx) {
    uint32_t ring_w = 10;
    unsigned long t = millis();


    const char* status_str = nullptr;
    uint32_t color = TFT_BLACK;

    switch (State::getState()) {
        case MachineState::Init:
            status_str = "INIT";
            color = TFT_BLUE;
            break;

        case MachineState::Preheat:
        {
            status_str = "WARMING UP";
            color = TFT_ORANGERED;

            // Pulse animation
            ring_w += 5 + (sinf((t * 0.1f) * deg2rad + PI) * 5.0f);

            break;
        }

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
            break;
        }

        case MachineState::Ready:
            // TODO: Connect the above pulse animation and relax back into an idle state

            //status_str = "READY";
            //color = TFT_DARKGREEN;

            break;

        case MachineState::Sleep:
            status_str = "ZzZz";
            color = TFT_DARKCYAN;

            // Pulse animation
            ring_w += 5 + (sinf((t * 0.1f) * deg2rad + PI) * 5.0f);

            break;

        case MachineState::FirmwareUpdate:
            status_str = "UPDATING";
            color = TFT_SKYBLUE;
            break;

        default:
            return;
    }

    if (color != TFT_BLACK) {
        gfx.drawSmoothArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-ring_w, 0, 360, color, TFT_BLACK);
    }

    if (status_str != nullptr) {
        int16_t tw = gfx.textWidth(status_str);
        int16_t th = 14;
        gfx.setCursor(TFT_WIDTH/2 - tw/2, TFT_HEIGHT/2 - th/2);
        gfx.print(status_str);
    }
}

void uiGetRadialCoords(int32_t r, float angle, int32_t *x, int32_t *y) {
    angle = 180 + angle;
    *x = (TFT_WIDTH/2)  + (r * -sinf(angle * deg2rad));
    *y = (TFT_HEIGHT/2) + (r * +cosf(angle * deg2rad));
}

void uiRenderStatusIcons(GfxCanvas& gfx) {
    const int32_t r = (TFT_WIDTH/2) - 40;
    int32_t x, y;

    if (IO::isHeaterOn()) {
        // TODO: Replace with heat icon
        uiGetRadialCoords(r, -45, &x, &y);
        gfx.fillCircle(x, y, 5, TFT_ORANGERED);
    }

    // WiFi connection indicator
    // TODO: Replace with WiFi icons
    int wifiColor = TFT_YELLOW;
    if (WiFi.getMode() & WIFI_MODE_STA) {
        auto wifiStatus = WiFi.status();
        if (wifiStatus == WL_IDLE_STATUS || wifiStatus >= WL_DISCONNECTED) {
            float f = sinf((millis() * 0.5f) * deg2rad + PI) * 0.5f + 0.5f;
            int16_t c = f*127.0f + 127.0f;
            wifiColor = TFT_RGB656(0, 0, c);
            //wifiColor = TFT_SKYBLUE;

        }
        else if (wifiStatus == WL_CONNECTED) {
            wifiColor = TFT_DARKGREEN;
        }

        // auto ip_str = WiFi.localIP().toString();
        // uiRenderLabelCentered(gfx_left, ip_str.c_str(), 30, TFT_LIGHTGREY);

        const char* err_msg = nullptr;
        switch (wifiStatus) {
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

    }

    uiGetRadialCoords(r, 22, &x, &y);
    gfx.fillCircle(x, y, 5, wifiColor);



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

void uiRenderMargin() {
    // uint32_t margin = 5;
    // uint32_t ring_w = 10 + margin;
    // gfx_right.drawArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-ring_w, 0, 360, TFT_BLACK, TFT_BLACK, false);
}

void uiRenderPressureGauge(GfxCanvas& gfx) {
     //float value_norm = (values.t - 15.0f) / (25.0f - 15.0f);
    float min_pressure = 1.0f; // Ambient air pressure
    float max_pressure = 12.0f;
    float value_norm = (SensorSampler::getPressure() - min_pressure) / (max_pressure - min_pressure);
    uiRenderGauge(gfx, value_norm, TFT_DARKGREEN);
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
    uiRenderGauge(gfx, value_norm, TFT_RED);
}

void uiRenderHeaterPowerGauge(GfxCanvas& gfx) {
    uiRenderGauge(gfx, IO::getHeatPower(), TFT_YELLOW, 15);
}

void uiRenderFlowGauge(GfxCanvas& gfx) {
    float flow = SensorSampler::getFlowRate();
    float min_flow = 0.0f;
    float max_flow = 10.0f;
    float value_norm = (flow - min_flow) / (max_flow - min_flow);
    uiRenderGauge(gfx, value_norm, TFT_SKYBLUE, 15);
}

unsigned long t_last = 0;





void uiRenderTemperatureGraph(GfxCanvas& gfx, uint16_t color = TFT_WHITE) {
    float min_value = getTemperatureMinRange();
    float max_value = getTemperatureMaxRange();
    uiRenderGraph(gfx, temperatureSamples, min_value, max_value, color);
}

void uiRenderBrewGraph(GfxCanvas& gfx, bool freeze = false) {
    {
        float min_value = 0.0f;
        float max_value = 12.0f;
        auto& samples = (freeze) ? pressureSamplesFrozen : pressureSamples;

        uiRenderGraph(gfx, samples, min_value, max_value, TFT_DARKGREEN);
    }

    {
        float min_value = 0.0f;
        float max_value = 10.0f;
        auto& samples = (freeze) ? flowSamplesFrozen : flowSamples;

        uiRenderGraph(gfx, samples, min_value, max_value, TFT_SKYBLUE);
    }
}

float calculateEstimatedGroupheadTemp(float t) {
    if (t < 90.0f) {
        // Correction doesn't work in this range, 
        // switch to approximate linear correction
        return t * 0.8008f;
    }
    else {
        // Polynomial correction based on real measurements between
        // boiler and grouphead temperature.
        // NOTE: Real temperatures fluctuate a lot, so this is 
        // only an approximation.
        //return (t*t*0.0921f) - (t*21.614f) + 1363.7f;
        // =(D40*D40*0.0163)-(D40*3.2351)+250
        return (t*t*0.0163f) - (t*3.2351f) + 250.0f;
    }
}

void uiRenderTemperatureLeft()
{
    uiRenderTemperatureGraph(gfx_left);

    float t_boiler = SensorSampler::getTemperature();

    // Render current temperature
    {
        char buffer[20];
        if (SensorSampler::isTemperatureValid() && (t_boiler > 20.0f)) {
            snprintf(buffer, sizeof(buffer), "%.2f C", t_boiler);
        } else {
            snprintf(buffer, sizeof(buffer), "- C");
        }
        buffer[sizeof(buffer)-1] = '\0';

        uiRenderLabelCentered(gfx_left, buffer, (TFT_HEIGHT/2 - 60));
    }

    // Estimated grouphead temperature
    {
        char buffer[20];
        if (SensorSampler::isTemperatureValid() && (t_boiler > 100.0f)) {
            float t = calculateEstimatedGroupheadTemp(t_boiler);
            // NOTE: No decimal places as the estimation is only accurate to
            // within a few degrees C anyway, don't want to give the illusion
            // of accuracy.
            snprintf(buffer, sizeof(buffer), "%.0f C", t);
        } else {
            snprintf(buffer, sizeof(buffer), "- C");
        }
        buffer[sizeof(buffer)-1] = '\0';

        uiRenderLabelCentered(gfx_left, buffer, (TFT_HEIGHT/2 - 30));
    }

    uiRenderTemperatureGauge(gfx_left);
    uiRenderHeaterPowerGauge(gfx_left);
    uiRenderStatusIcons(gfx_left);
}

void uiRenderPreheat()
{

}

void uiRenderReady()
{
    // Left Display:
    //     - Temperature Gauge & Graph ??
    // Right Display:
    //     - Green ring indicating "ready"

    if (brewStats.start_brew_time > 0) {
        // Left Display:
        //     - Pressure & flow chart
        //     - Average pressure & flow
        //     - Average temperature
        // Right Display:
        //     - Final time
        //     - mL delivered

        // Display snapshot of last brew graph
        uiRenderBrewGraph(gfx_right, true);

        float brewTimeSec = (brewStats.end_brew_time - brewStats.start_brew_time) / 1000.0f;
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%.1f", brewTimeSec);
        buffer[sizeof(buffer)-1] = '\0';

        uiRenderLabelCentered(gfx_right, "BREW TIME:", (-(TFT_HEIGHT/2) + 60));
        uiRenderLabelCentered(gfx_right, buffer,  (-(TFT_HEIGHT/2) + 90));
    }
    else {
        //uiRenderLabelCentered(gfx_right, "READY", 0);
        
        uiRenderBrewGraph(gfx_right);
    }

    uiRenderPressureGauge(gfx_right);
    uiRenderFlowGauge(gfx_right);

    {
        // Render pressure label
        char buffer[20];
        if (SensorSampler::isPressureValid()) {
            snprintf(buffer, sizeof(buffer), "%.2f Bar", SensorSampler::getPressure());
        } else {
            snprintf(buffer, sizeof(buffer), "- Bar");
        }
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, 60);
    }
}

void uiRenderBrewing() {
    // Left Display:
    //     - Pressure Gauge
    //     - Pressure & flow chart
    //     - Warnings: 
    //       - Channeling detected (flow vs pressure)
    //       - Pressure too low
    //       - Flow too low
    //       - Flow too high
    // Right Display:
    //     - Timer, in center
    //     - Phase (Soak, Pre-Infusion, Extract)

    uiRenderBrewGraph(gfx_right);
    uiRenderMargin();

    {
        const char* status_str = "BREWING";
        int16_t tw = gfx_right.textWidth(status_str);
        int16_t th = 14;
        gfx_right.setCursor(TFT_WIDTH/2 - tw/2, TFT_HEIGHT/2 - th/2);
        gfx_right.print(status_str);
    }

    {
        // Render timer
        float brewTimeSec = (millis() - brewStats.start_brew_time) / 1000.0f;
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%.1f", brewTimeSec);
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, 20);
    }

    {
        // Render pressure
        char buffer[20];
        if (SensorSampler::isPressureValid()) {
            snprintf(buffer, sizeof(buffer), "%.2f Bar", SensorSampler::getPressure());
        } else {
            snprintf(buffer, sizeof(buffer), "- Bar");
        }
        buffer[sizeof(buffer)-1] = '\0';

        uiRenderLabelCentered(gfx_right, buffer, 60);
    }

    uiRenderPressureGauge(gfx_right);
    uiRenderFlowGauge(gfx_right);
    uiRenderTemperatureGauge(gfx_left);
}

void uiRenderSensorTest()
{
    uiRenderTemperatureGraph(gfx_left, TFT_RED);
    uiRenderBrewGraph(gfx_right);

    {
        char buffer[20];
        if (SensorSampler::isPressureValid()) {
            snprintf(buffer, sizeof(buffer), "%.2f Bar", SensorSampler::getPressure());
        } else {
            snprintf(buffer, sizeof(buffer), "- Bar");
        }
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, -60, TFT_DARKGREEN);
    }

    {
        char buffer[20];
        if (SensorSampler::isFlowRateValid()) {
            snprintf(buffer, sizeof(buffer), "%.1f /s", SensorSampler::getFlowRate());
        } else {
            snprintf(buffer, sizeof(buffer), "- /s");
        }
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, -30, TFT_SKYBLUE);
    }


    {
        char buffer[20];
        if (SensorSampler::isFlowRateValid()) {
            snprintf(buffer, sizeof(buffer), "%.1f mL", SensorSampler::getTotalFlowVolume());
        } else {
            snprintf(buffer, sizeof(buffer), "- mL");
        }
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, 0, TFT_DARKCYAN);
    }

    {
        char buffer[20];
        if (SensorSampler::isTemperatureValid()) {
            snprintf(buffer, sizeof(buffer), "%.1f C", SensorSampler::getTemperature());
        } else {
            snprintf(buffer, sizeof(buffer), "- C");
        }
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_left, buffer, 0, TFT_RED);
    }

    uiRenderPressureGauge(gfx_right);
    uiRenderFlowGauge(gfx_right);
    uiRenderTemperatureGauge(gfx_left);
}


void render() {
    auto t1 = millis();

    gfx_left.fillSprite(TFT_BLACK);
    gfx_right.fillSprite(TFT_BLACK);


    switch (uiState) {
        case MachineState::Preheat:
            uiRenderPreheat();
            uiRenderTemperatureLeft();
            break;
        case MachineState::Ready:
            uiRenderReady();
            uiRenderTemperatureLeft();
            break;
        case MachineState::Brewing:
            uiRenderBrewing();
            uiRenderTemperatureLeft();
            break;
        case MachineState::Sleep:
            uiRenderTemperatureLeft();
            break;
        case MachineState::SensorTest:
            uiRenderSensorTest();
            break;
        // case MachineState::PostBrew:
        //     uiRenderPostBrew();
        //     break;
        default:
            break;
    }

    uiDrawStatusCircle(gfx_right);

    auto t2 = millis();

    tftUpdateDisplay();

    auto t3 = millis();

    //Serial.printf("Render %dms Update %dms\n", (t2-t1), (t3-t2));
}


}