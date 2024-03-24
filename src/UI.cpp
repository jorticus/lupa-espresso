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
using namespace SensorSampler;

constexpr float deg2rad      = 3.14159265359/180.0;

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
            err_msg = "Bad SSID";
            break;
        case WL_CONNECT_FAILED:
            err_msg = "Connect Failed";
            break;
        case WL_IDLE_STATUS:
            err_msg = "Connecting";
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
    auto mode = HeatControl::getProfile();
    switch (mode) {
        // case HeatControl::BoilerProfile::Brew:
        //     color = TFT_RED;
        //     break;
        case HeatControl::BoilerProfile::Steam:
            color = TFT_SKYBLUE;
            break;
        case HeatControl::BoilerProfile::Idle:
            color = TFT_DARKCYAN;
            break;
    }
    uiGetRadialCoords(r, +45, &x, &y);
    gfx.fillCircle(x, y, 5, color);
}

void uiRenderPressureGauge(GfxCanvas& gfx, bool postbrew = false) {
    float min_pressure = 1.0f; // Ambient air pressure
    float max_pressure = PRESSURE_MAX_BAR;
    float value_norm = (SensorSampler::getPressure() - min_pressure) / (max_pressure - min_pressure);
    uiRenderGauge(gfx, value_norm, COLOUR_PRESSURE);

    if (postbrew) {
        float avg = 0.0f;
        if (brewStats.brew_pressure_avg_count > 0)
            avg = brewStats.avg_brew_pressure / brewStats.brew_pressure_avg_count;
        uiRenderLabelFormattedCentered(gfx,
            60,
            TFT_WHITE,
            "%.1f Bar",
            avg);
    }
    else {
        uiRenderLabelFormattedCentered(gfx,
            60,
            TFT_WHITE,
            (SensorSampler::isPressureValid() ? "%.1f Bar" : "- Bar"),
            SensorSampler::getPressure());
    }
}

float getTemperatureMaxRange() {
    return CONFIG_MAX_BOILER_TEMPERATURE_C;
}
float getTemperatureMinRange() {
    const float range1 = 20.0f;
    const float range2 = 100.0f;
    if ((uiState != MachineState::Preheat) && (SensorSampler::getTemperature() >= range2)) {
        // Use smaller range when up to temperature
        return range2;
    }
    else {
        return range1;
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

void uiRenderFlowGauge(GfxCanvas& gfx, bool postbrew = false) {
    float flow = SensorSampler::getFlowRate();
    float min_flow = 0.0f;
    float max_flow = FLOW_MAX_VALUE;
    float value_norm = (flow - min_flow) / (max_flow - min_flow);
    uiRenderGauge(gfx, value_norm, COLOUR_FLOW_RATE, 15);

    //uiRenderLabelFormattedCentered(gfx, -60, TFT_RED, "%.3f", flow);

    if (postbrew) {
        uiRenderLabelFormattedCentered(gfx, 90, TFT_WHITE, "%.0f mL", brewStats.total_volume);
    }
    else {
        // NOTE: Only accurate to +/- 10mL assuming system is primed
        // Cannot fully account for volume lost to internal plumbing
        float flow_accum = SensorSampler::getTotalFlowVolume();
        uiRenderLabelFormattedCentered(gfx, 90, TFT_WHITE, "%.0f mL", flow_accum);
    }
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

    // Rate the brew
    const char* text = "DONE!";

    // A proper brew should take between 10 to 60 seconds
    if (brewTimeSec < 10.0f) {
        text = "TOO SHORT";
    }
    else if (brewTimeSec > 60.0f) {
        text = "TOO LONG";
    }

    // A proper brew should reach 8 bar ideally
    if (brewStats.brew_pressure_avg_count > 0) {
        const float min_required_brew_pressure = 7.0f;
        float avg_pressure = brewStats.avg_brew_pressure / brewStats.brew_pressure_avg_count;
        if (avg_pressure < min_required_brew_pressure) {
            text = "NO PRESSURE";
        }
    } 

    if (text != nullptr) {
        uiRenderLabelCentered(gfx, 0, TFT_WHITE, text);
    }

    uiRenderLabelFormattedCentered(gfx, 30, TFT_WHITE, "%.1f s", brewTimeSec);
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
        30, 
        TFT_WHITE,
        "%.1f", 
        brewTimeSec);

    // uiRenderLabelFormattedCentered(gfx, 
    //     60, 
    //     TFT_WHITE,
    //     (SensorSampler::isPressureValid() ? "%.1f Bar" : "- Bar"), 
    //     SensorSampler::getPressure());

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
        (SensorSampler::isFlowRateValid() ? "%.2f mL" : "- mL"),
        SensorSampler::getTotalFlowVolume());

    uiRenderTemperatureGauge(gfx_left);
}

/// @brief Render the left UI
void renderLeft() {
    auto& gfx = gfx_left;

    uiRenderTemperatureGraph(gfx);

    float t_boiler = SensorSampler::getTemperature();

    // Render current temperature
    bool is_t_valid = SensorSampler::isTemperatureValid() && (t_boiler > 1.0f);
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

void uiRenderStatusRing(GfxCanvas& gfx, const char* message, uint16_t color, uint32_t ring_w) {
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

    const uint32_t ring_w_min = 10;

    static uint32_t ring_w = ring_w_min;

    unsigned long t = millis();
    const char* status_str = nullptr;

    // Render UI elements for the current state
    switch (uiState) {
        case MachineState::Init:
            uiRenderStatusRing(gfx, "INIT", TFT_BLUE, ring_w);
            break;

        case MachineState::Tuning:
            ring_w = ring_w_min + 5 + (sinf((t * 0.1f) * deg2rad + PI) * 5.0f);
            uiRenderStatusRing(gfx, "TUNING", TFT_GREENYELLOW, ring_w);
            break;

        case MachineState::Preheat:
            // Pulse animation
            ring_w = ring_w_min + 5 + (sinf((t * 0.1f) * deg2rad + PI) * 5.0f);
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
                case FaultState::SoftwarePanic:
                    status_str = "FIRMWARE CRASH";
                    break;
                case FaultState::FailsafeRecovery:
                    status_str = "RECOVERY MODE";
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
            uint16_t color = TFT_RGB656(c, 0, 0);

            uiRenderStatusRing(gfx, status_str, color, ring_w);
            if (uiFaultMessage != nullptr) {
                uiRenderLabelCentered(gfx, 24, TFT_WHITE, uiFaultMessage);
            }
            break;
        }

        case MachineState::Ready:
            // Connect animation from Brewing phase
            if (ring_w > ring_w_min) {
                ring_w = ring_w_min + 5 + (sinf((t * 0.1f) * deg2rad + PI) * 5.0f);
            }

            if (brewStats.start_brew_time > 0) {
                // Show post-brew snapshot graph + brew time
                uiRenderPostBrewScreen(gfx);

                // Post-brew stats will be rendered for each gauge type
                uiRenderPressureGauge(gfx, true);
                uiRenderFlowGauge(gfx, true);
            }
            else {
                //uiRenderBrewGraph(gfx);
                uiRenderReadyScreen(gfx);
                    
                uiRenderPressureGauge(gfx);
                uiRenderFlowGauge(gfx);
            }
            break;

        case MachineState::Brewing:
            uiRenderBrewGraph(gfx);
            uiRenderBrewingScreen(gfx);
            
            uiRenderPressureGauge(gfx);
            uiRenderFlowGauge(gfx);
            break;

        case MachineState::FillTank:
            ring_w += 5 + (sinf((t * 0.1f) * deg2rad + PI) * 5.0f);
            uiRenderStatusRing(gfx, "FILLING", TFT_YELLOW, ring_w);
            break;

        case MachineState::StabilizePressure:
            uiRenderPressureGauge(gfx);
            uiRenderFlowGauge(gfx);
            uiRenderStatusRing(gfx, "STABILIZING", TFT_YELLOW, 10);
            break;

        case MachineState::Sleep:

            // Pulse animation
            ring_w += 5 + (sinf((t * 0.1f) * deg2rad + PI) * 5.0f);

            uiRenderStatusRing(gfx, "ZzZz", TFT_DARKCYAN, ring_w);
            break;
    }
}

const uint32_t anim_steps = 10;
static uint32_t startup_anim = anim_steps;
static uint32_t power_off_anim = 0;

void triggerAnimation(Anim anim) {
    switch (anim) {
        case Anim::PowerOff:
            Serial.println("Anim: Power Off");
            startup_anim = 0;
            power_off_anim = anim_steps;
            break;

        case Anim::PowerOn:
            Serial.println("Anim: Power On");
            power_off_anim = 0;
            startup_anim = anim_steps;
            break;
    }
}

void uiRenderGlobalAnimations() {
    
    if (startup_anim > 0) {
        startup_anim--;

        if (startup_anim == 0) {
            Display::setBrightness(CONFIG_FULL_BRIGHTNESS);
            return;
        }

        float b = 1.0f - ((float)(startup_anim) / (float)anim_steps);
        Serial.printf("%.1f (%d)\n", b, startup_anim);
        Display::setBrightness(b * CONFIG_FULL_BRIGHTNESS);

//Experimental growing ring animation
#if false
        const uint32_t steps = anim_steps / 2;
        const uint32_t w = (TFT_WIDTH/2);

        if (startup_anim > steps) {
            uint32_t r = w - (((startup_anim-steps) * w) / steps);
            gfx_left.fillSmoothCircle(
                (TFT_WIDTH/2),
                (TFT_HEIGHT/2),
                r,
                TFT_SKYBLUE,
                TFT_BLACK
            );
            gfx_right.fillSmoothCircle(
                (TFT_WIDTH/2),
                (TFT_HEIGHT/2),
                r,
                TFT_SKYBLUE,
                TFT_BLACK
            );

            // float b = 1.0f - ((float)(startup_anim-steps) / (float)steps);
            // Display::setBrightness(b * 0.2f);
        }
        else {
            uint32_t r = w - (((startup_anim) * w) / steps);
            gfx_left.drawSmoothArc(
                (TFT_WIDTH/2),
                (TFT_HEIGHT/2),
                w, r,
                0, 360,
                TFT_SKYBLUE,
                TFT_BLACK
            );
            gfx_right.drawSmoothArc(
                (TFT_WIDTH/2),
                (TFT_HEIGHT/2),
                w, r,
                0, 360,
                TFT_SKYBLUE,
                TFT_BLACK
            );
        }
#endif

        return;
    }

    if (power_off_anim > 0) {
        power_off_anim--;

        if (power_off_anim == 0) {
            Display::setBrightness(0.0f);
            Display::turnOff();
            return;
        }

        float b = ((float)(power_off_anim) / (float)anim_steps);
        Display::setBrightness(b * CONFIG_FULL_BRIGHTNESS);

#if false
        const uint32_t w = (TFT_WIDTH/2);
        uint32_t r = w - (((power_off_anim) * w) / anim_steps);
        gfx_left.fillSmoothCircle(
            (TFT_WIDTH/2),
            (TFT_HEIGHT/2),
            r,
            TFT_BLACK,
            TFT_BLACK
        );
        gfx_right.fillSmoothCircle(
            (TFT_WIDTH/2),
            (TFT_HEIGHT/2),
            r,
            TFT_BLACK,
            TFT_BLACK
        );
#endif
        return;
    }
}

void render() {
    auto t1 = millis();

    if (uiState == MachineState::Off && power_off_anim == 0) {
        return;
    }

    gfx_left.fillSprite(TFT_BLACK);
    gfx_right.fillSprite(TFT_BLACK);

    if (uiState == MachineState::SensorTest) {
        uiRenderSensorTest();
    }

    renderLeft();
    renderRight();

    uiRenderGlobalAnimations();

    auto t2 = millis();

    tftUpdateDisplay();

    auto t3 = millis();

    //Serial.printf("Render %dms Update %dms\n", (t2-t1), (t3-t2));
}


}