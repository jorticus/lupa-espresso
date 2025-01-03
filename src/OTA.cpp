#include <Arduino.h>
#include <ArduinoOTA.h>
#include "Display.h"
#include "IO.h"
#include "UI.h"
#include "Debug.h"
#include "StateMachine.h"
#include "secrets.h"
#include <esp_task_wdt.h>

using namespace Display;

const uint32_t WDT_OTA_TIMEOUT_SEC = 120;

namespace OTA
{

enum class OtaState {
    Begin,
    Progress,
    Success,
    Failure
};

/// @brief Render and update display with OTA progress
/// @param state Current state of the OTA operation
/// @param param Progress or error code
static void uiRenderFirmwareUpdate(OtaState state, int param) {
    int32_t ringw = 10;

    tftClearCanvas();

    if (state != OtaState::Failure) {
        int progress = param;
        int color = TFT_SKYBLUE;

        uint32_t min_angle = 0;
        uint32_t max_angle = 360;
        uint32_t angle = (progress * (max_angle - min_angle) / 100) + min_angle;
        gfx_right.drawSmoothArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-ringw, min_angle, max_angle, TFT_DARKESTGREY, TFT_BLACK, false);
        if (angle > min_angle) {
            gfx_right.drawSmoothArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-ringw, min_angle, angle, TFT_SKYBLUE, TFT_BLACK, false);
        }
    }
    else {
        gfx_right.drawSmoothArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-ringw, 0, 360, TFT_RED, TFT_BLACK, false);
    }

    const char* status_str = nullptr;
    switch (state) {
        case OtaState::Begin:
        case OtaState::Progress:
            status_str = "UPDATING";
            break;
        case OtaState::Success:
            status_str = "DONE!";
            break;
        case OtaState::Failure:
            status_str = "UPDATE FAILURE";
            break;
    }

    if (status_str != nullptr) {
        int16_t tw = gfx_right.textWidth(status_str);
        int16_t th = 14;
        gfx_right.setTextColor(TFT_WHITE);
        gfx_right.setCursor(TFT_WIDTH/2 - tw/2, TFT_HEIGHT/2 - th/2);
        gfx_right.print(status_str);
    }

    tftUpdateDisplay();
}



void initOTA() {
    Debug.println("Starting OTA Server...");

    ArduinoOTA.setHostname(secrets::device_name);

    // No authentication by default
    // ArduinoOTA.setPassword((const char *)"123");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
        State::setState(State::MachineState::FirmwareUpdate);

        //Display::setBrightness(1.0f);
        Debug.println("OTA Initiated");

        esp_task_wdt_reset();
        esp_task_wdt_init(WDT_OTA_TIMEOUT_SEC, true);

        // TODO: Communicate state to UI task. Cannot render here without causing issues...
        //uiRenderFirmwareUpdate(OtaState::Begin, 0);
    });

    ArduinoOTA.onEnd([]() {
        Debug.println("OTA Done!");

        esp_task_wdt_reset();

        //uiRenderFirmwareUpdate(OtaState::Success, 100);

        //ESP.restart();
        esp_restart();
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        // Throttle display updates to avoid slowing down the update
        static int last_x = 0;
        int x = (progress / (total / 20));
        if (x != last_x) {
             last_x = x;
            int p = (progress * 100) / total;
            //uiRenderFirmwareUpdate(OtaState::Progress, p);
        }

        esp_task_wdt_reset();
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Debug.println();
        Debug.printf("OTA Error[%u]: ", error);
        
        State::setFault(State::FaultState::FirmwareUpdateFailure);

        esp_task_wdt_reset();

        if (error == OTA_AUTH_ERROR)            Debug.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)      Debug.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)    Debug.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)    Debug.println("Receive Failed");
        else if (error == OTA_END_ERROR)        Debug.println("End Failed");

        uiRenderFirmwareUpdate(OtaState::Failure, (int)error);

        delay(1000);
        esp_restart();
    });

    // Enable OTA
    ArduinoOTA.begin();
}

void handle() {
    ArduinoOTA.handle();
}

}