#include <Arduino.h>
#include <WiFi.h>
#include "UI.h"
#include "StateMachine.h"
#include "UIWidgets.h"
#include "Display.h"
#include "SensorSampler.h"
#include "HeatControl.h"
#include "IO.h"
#include "Debug.h"
#include "Net.h"
#include "Images.h"
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

void uiRenderWiFiStatus(GfxCanvas& gfx, int32_t x, int32_t y) {

    if (Net::isConnected()) {
        uiRenderImageCentered(gfx, x, y, ico_wifi_connected_16px, TFT_WHITE);
    }
    else if (Net::isConnecting()) {
        // Pulsing color animation
        float f = sinf((millis() * 0.5f) * deg2rad + PI) * 0.5f + 0.5f;
        int16_t c = f*127.0f + 127.0f;
        int color = TFT_RGB656(0, 0, c);
        uiRenderImageCentered(gfx, x, y, ico_wifi_unknown_16px, color);
    }
    else {
        // auto ip_str = WiFi.localIP().toString();
        // uiRenderLabelCentered(gfx_left, ip_str.c_str(), 30, TFT_LIGHTGREY);
        const char* err_msg = nullptr;
        switch (WiFi.status()) {
            case WL_NO_SSID_AVAIL:
                // Bad SSID or AP not present
                err_msg = "No WiFi";
                break;
            case WL_CONNECT_FAILED:
                err_msg = "WiFi Failed";
                break;
            // case WL_IDLE_STATUS:
            //     err_msg = "Connecting";
            //     break;
            case WL_CONNECTION_LOST:
                err_msg = "Connection Lost";
                break;
        }
        if (err_msg != nullptr) {
            uiRenderImageCentered(gfx, x, y, ico_wifi_err_16px, TFT_ORANGE);
            uiRenderLabelCentered(gfx, err_msg, 0); // TODO: May need repositioning
        }
        else {
            uiRenderImageCentered(gfx, x, y, ico_wifi_err_16px, TFT_DARKGREY);
        }
    }
}

void uiRenderStatusIcons(GfxCanvas& gfx) {
    const int32_t r = (TFT_WIDTH/2) - 20;
    int32_t x, y;

    // Boiler heater indicator
    uiGetRadialCoords(r, -135, &x, &y);
    uiRenderImageCentered(gfx, x, y, ico_element_16px, 
        (IO::isHeaterOn() ? TFT_ORANGERED : TFT_DARKGREY));

    // WiFi connection status
    uiGetRadialCoords(r, +135, &x, &y);
    uiRenderWiFiStatus(gfx, x, y);

#if 0
    // Boiler heat mode
    uiGetRadialCoords(r, +45, &x, &y);
    uint32_t color = TFT_BLACK;
    auto mode = HeatControl::getProfile();
    switch (mode) {
        case HeatControl::BoilerProfile::Brew:
            uiRenderImageCentered(gfx, x, y, ico_cup_16px, TFT_SKYBLUE);
            break;
        case HeatControl::BoilerProfile::Steam:
            uiRenderImageCentered(gfx, x, y, ico_steam_16px, TFT_SKYBLUE);
            break;
        case HeatControl::BoilerProfile::Idle:
            break;
    }
#endif
    
    //gfx.fillCircle(x, y, 5, color);
}

void uiRenderPressureGauge(GfxCanvas& gfx, bool postbrew = false) {
    int32_t y = 80;
    float min_pressure = 1.0f; // Ambient air pressure
    float max_pressure = PRESSURE_UI_MAX_BAR;
    float value_norm = (SensorSampler::getPressure() - min_pressure) / (max_pressure - min_pressure);
    uiRenderGauge(gfx, value_norm, COLOUR_PRESSURE);

    if (postbrew) {
        float avg = 0.0f;
        if (brewStats.brew_pressure_avg_count > 0)
            avg = brewStats.avg_brew_pressure / brewStats.brew_pressure_avg_count;

        uiRenderLabelFormattedCentered(gfx,
            y,
            TFT_WHITE,
            "%.1f Bar",
            avg);
    }
    else if (uiState == MachineState::Brewing) {
        uiRenderLabelFormattedCentered(gfx,
            y,
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
    const int32_t y = 30;
    float flow = SensorSampler::getFlowRate();
    float min_flow = 0.0f;
    float max_flow = FLOW_UI_MAX_VALUE;
    float value_norm = (flow - min_flow) / (max_flow - min_flow);
    uiRenderGauge(gfx, value_norm, COLOUR_FLOW_RATE, 15);

    //uiRenderLabelFormattedCentered(gfx, -60, TFT_RED, "%.3f", flow);

    if (postbrew) {
        uiRenderLabelFormattedCentered(gfx, y, TFT_WHITE, "%.0f mL", brewStats.total_volume);
    }
    else if (uiState == MachineState::Brewing) {
        // NOTE: Only accurate to +/- 10mL assuming system is primed
        // Cannot fully account for volume lost to internal plumbing
        float flow_accum = SensorSampler::getTotalFlowVolume();
        uiRenderLabelFormattedCentered(gfx, y, TFT_WHITE, "%.0f mL", flow_accum);
    }
}

void uiRenderTemperatureGraph(GfxCanvas& gfx, uint16_t color = TFT_WHITE) {
    float min_value = getTemperatureMinRange();
    float max_value = getTemperatureMaxRange();

#if 0
    // DEBUG: Print PID outputs
    const float pid_max = 50.0f;
    const float pid_min = -50.0f;
    uiRenderGraph(gfx, HeatControl::pid_i,  pid_min, pid_max, TFT_PURPLE);
    uiRenderGraph(gfx, HeatControl::pid_d,  pid_min, pid_max, TFT_YELLOW);
    uiRenderGraph(gfx, HeatControl::pid_d2, pid_min, pid_max, TFT_RED);

    uiRenderGraph(gfx, temperatureSamples2, min_value, max_value, TFT_SILVER);
#endif

    uiRenderGraph(gfx, temperatureSamples,  min_value, max_value, color, HeatControl::getSetpoint());
}

void uiRenderBrewGraph(GfxCanvas& gfx, bool freeze = false) {
    {
        float min_value = 0.0f;
        float max_value = PRESSURE_UI_MAX_BAR;
        auto& samples = (freeze) ? pressureSamplesFrozen : pressureSamples;

        uiRenderGraph(gfx, samples, min_value, max_value, COLOUR_PRESSURE);
    }

    {
        float min_value = 0.0f;
        float max_value = FLOW_UI_MAX_VALUE;
        auto& samples = (freeze) ? flowSamplesFrozen : flowSamples;

        uiRenderGraph(gfx, samples, min_value, max_value, COLOUR_FLOW_RATE);
    }
}

void uiRenderPostBrewScreen(GfxCanvas& gfx)
{
    // Display snapshot of last brew graph
    uiRenderBrewGraph(gfx, true);

    float brewTimeSec = (brewStats.end_brew_time - brewStats.start_brew_time) / 1000.0f;

#if 0
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
#else
    uiRenderLabelFormattedCentered(gfx, 0, TFT_WHITE, "%.1f s", brewTimeSec);
#endif
}

void uiRenderReadyScreen(GfxCanvas& gfx)
{
    //uiRenderLabelCentered(gfx, "READY", 0);
    //uiRenderBrewGraph(gfx_right);
}

void uiRenderBrewingScreen(GfxCanvas& gfx) {

    //uiRenderLabelCentered(gfx, 0, TFT_WHITE, "BREWING");

    float brewTimeSec = (millis() - brewStats.start_brew_time) / 1000.0f;
    uiRenderLabelFormattedCentered(gfx, 
        0, 
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

#ifdef DUAL_BUFFERS
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
#endif

/// @brief Render the left UI
void renderLeft(GfxCanvas& gfx) {

    uiRenderTemperatureGraph(gfx);

    float t_boiler = SensorSampler::getTemperature();

    // Render current temperature
    bool is_t_valid = SensorSampler::isTemperatureValid() && (t_boiler > 1.0f);
    uiRenderLabelFormattedCentered(gfx,
        80,
        TFT_WHITE,
        (is_t_valid ? "%.2f C" : "- C"), 
        t_boiler);

    // // Estimated grouphead temperature
    // bool is_est_valid = SensorSampler::isTemperatureValid() && (t_boiler > 100.0f);
    // if (is_est_valid) {
    //     float t_estimated = SensorSampler::getEstimatedGroupheadTemperature();
    //     uiRenderLabelFormattedCentered(gfx,
    //         (TFT_HEIGHT/2 - 60),
    //         TFT_WHITE,
    //         "%.0f C",
    //         t_estimated);
    // }

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

void uiRenderFillingAnim(GfxCanvas& gfx) {
    // Ocean colours
    const uint16_t ocean = TFT_RGB656(0, 100, 190);
    const uint16_t ocean_high = TFT_RGB656(90, 170, 235);

    // Base rectangle fills the lower half
    const int32_t baseY = TFT_HEIGHT / 2;
    gfx.fillRect(0, baseY, TFT_WIDTH, TFT_HEIGHT - baseY, ocean);

    // Wave parameters
    unsigned long t = millis();
    const float amplitude = 16.0f;
    const float two_pi = 2.0f * PI;

    for (int x = 0; x <= TFT_WIDTH; x++) {
        float ang = (x * 0.01f) + (t * 0.0005f) * two_pi;
        int32_t h = sinf(ang) * amplitude - amplitude;
        gfx.drawLine(x, baseY, x, baseY + h, ocean);
    }

    const char* message = "FILLING";
    int16_t tw = gfx.textWidth(message);
    int16_t th = 14;
    gfx.setCursor(TFT_WIDTH/2 - tw/2, 40);//TFT_HEIGHT/2 - th/2 - 2*amplitude - 10);
    gfx.print(message);
}

void uiRenderFaultRing(GfxCanvas& gfx) {
    const uint32_t ring_w = 10;
    const char* status_str = nullptr;
    unsigned long t = millis();

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
}

/// @brief Render the right UI
void renderRight(GfxCanvas& gfx) {

    const uint32_t ring_w_min = 10;

    static uint32_t ring_w = ring_w_min;

    unsigned long t = millis();
    const char* status_str = nullptr;

    // Pulse animation
    ring_w = ring_w_min + 10 + (sinf((t * 0.1f) * deg2rad + PI) * 5.0f);

    // Render UI elements for the current state
    switch (uiState) {
        case MachineState::Init:
            uiRenderStatusRing(gfx, "INIT", TFT_BLUE, ring_w);
            break;

        case MachineState::Tuning:
            uiRenderStatusRing(gfx, "TUNING", TFT_GREENYELLOW, ring_w);
            break;

        case MachineState::Preheat:
            // Pulse animation
            uiRenderStatusRing(gfx, "WARMING UP", TFT_ORANGERED, ring_w);
            break;

        case MachineState::Fault:
            uiRenderFaultRing(gfx);
            break;

        case MachineState::Ready:
        case MachineState::PostBrew:
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
            uiRenderFillingAnim(gfx);
            break;

        case MachineState::StabilizePressure:
            // uiRenderPressureGauge(gfx);
            // uiRenderFlowGauge(gfx);
            uiRenderStatusRing(gfx, "STABILIZING", TFT_YELLOW, ring_w);
            break;

        case MachineState::Sleep:
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
            Debug.println("Anim: Power Off");
            startup_anim = 0;
            power_off_anim = anim_steps;
            break;

        case Anim::PowerOn:
            Debug.println("Anim: Power On");
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
        // Debug.printf("%.1f (%d)\n", b, startup_anim);
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

void uiRenderBackground(GfxCanvas& gfx) {
    if (uiState == MachineState::Ready) {
        uiRenderImage(gfx, 0, 0, bg_coffee_eye);
    }
    else {
        // If background image is drawn, we don't need to clear the buffer.
        gfx.fillSprite(TFT_BLACK);
    }
}

void render() {
    auto t1 = millis();

    // if (uiState == MachineState::Off && power_off_anim == 0) {
    //     return;
    // }

#ifdef DUAL_BUFFERS

    // With two buffers available, we can render both sides at the same time

    uiRenderBackground(gfx_left);
    uiRenderBackground(gfx_right);

    if (uiState == MachineState::SensorTest) {
        uiRenderSensorTest();
    }

    renderLeft(gfx_left);
    renderRight(gfx_right);

    uiRenderGlobalAnimations();

    auto t2 = millis();

    tftUpdateDisplay();

    auto t3 = millis();

#else

    // With only one buffer, we save memory, but have to draw/update alternating buffers

    uiRenderBackground(gfx);
    renderLeft(gfx);
    tftUpdateDisplay(Display::ActiveBuffer::Left);

    uiRenderBackground(gfx);
    renderRight(gfx);
    tftUpdateDisplay(Display::ActiveBuffer::Right);

    uiRenderGlobalAnimations();

#endif

    // Debug.printf("Render %dms Update %dms\n", (t2-t1), (t3-t2));
}

void renderFailsafe() {
    // Minimal UI render
    tftClearCanvas();
#ifdef DUAL_BUFFERS
    uiRenderFaultRing(gfx_right);
#else
    uiRenderFaultRing(gfx);
#endif
    tftUpdateDisplay();
    Display::setBrightness(CONFIG_FULL_BRIGHTNESS);
}


}