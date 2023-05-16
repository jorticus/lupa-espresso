#include <array>
#include <Arduino.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_GC9A01A.h>
//#include <Arduino_GFX.h>
#include <Wire.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_MAX31865.h>
#include <ArduinoOTA.h>
#include "secrets.h"
#include "pressure_transducer.h"
#include "PulseCounter.h"
#include <TFT_eSPI.h>
#include "config.h"
#include "hardware.h"
#include "value_array.h"

#define TFT_RGB656(r,g,b)      ((((r & 0xFF) >> 3) << 11) | (((g & 0xFF) >> 2) << 6) | ((b & 0xFF) >> 2))
#define TFT_DARK_GOLDENROD  (TFT_RGB656(0xB8,0x86,0x0B))
#define TFT_ORANGERED       (TFT_RGB656(0xFF,0x45,0x00))
#define TFT_DARKESTGREY     (TFT_RGB656(30,30,30)) // TODO: This is green?

const char* DEVICE_NAME = "LUPA";

TFT_eSPI    tft;
TFT_eSprite gfx_left  { &tft };
TFT_eSprite gfx_right { &tft };

constexpr float deg2rad      = 3.14159265359/180.0;

Adafruit_MAX31865   rtd(MAX_CS, &tft.getSPIinstance());
PressureTransducer  pressure(PRESSURE_FULL_SCALE);

enum class UiState {
    Init,
    Preheat,
    Ready,
    Fault,
    Brewing,
    PostBrew,
    SensorTest,
    FirmwareUpdate,
};
UiState uiState = UiState::Init;

const char* UiState_Str[] = {
    "Init",
    "Pre-Heat",
    "Ready",
    "Fault",
    "Brewing",
    "Post-Brew",
    "Sensor Test",
    "Firmware Update"
};

enum class FaultState {
    NoFault,
    LowWater,
    OverTemp,
    SensorFailure,
    NotHeating,
    FirmwareUpdateFailure
};
FaultState uiFault = FaultState::NoFault;

enum class OtaState {
    Begin,
    Progress,
    Success,
    Failure
};

enum class WifiConnectionStatus {
    Connecting,
    Failure,
    Connected
};

struct {
    float t;
    bool t_valid;

    int p;
    bool p_valid;
    bool p_reading;

    float f;
    bool f_valid;
    float f_accum;
} values = {0};

struct {
    unsigned long start_brew_time;
    unsigned long end_brew_time;

    float total_flow;
} brew = {0};

//unsigned long startBrewTime = 0;

ValueArray<float, TFT_WIDTH> temperatureSamples;
ValueArray<float, TFT_WIDTH> pressureSamples;
ValueArray<float, TFT_WIDTH> flowSamples;

bool isRtdAvailable = false;
bool isPressureAvailable = false;
bool isFlowAvailable = false;

int counter = 0;

void uiDrawStatusCircle(TFT_eSprite& gfx);
void uiRenderLabelCentered(TFT_eSprite& gfx, const char* s, int16_t y, uint16_t color);

/// @brief Reset device into a fail-safe mode
/// where any outputs are turned off.
void failsafe() {
    digitalWrite(PIN_OUT_HEAT, LOW);
    digitalWrite(PIN_OUT_PUMP, LOW);
    digitalWrite(PIN_OUT_FILL_SOLENOID, LOW);

    //digitalWrite(TFT_BL, LOW);
}


void initGpio() {
    // BOOT button used for debugging
    pinMode(0, INPUT);

    // The following devices share the same SPI bus. 
    // Ensure all CS pins are de-asserted.
    pinMode(TFT_CS_LEFT, OUTPUT);
    pinMode(TFT_CS_RIGHT, OUTPUT);
    pinMode(MAX_CS, OUTPUT);
    digitalWrite(MAX_CS, HIGH);
    digitalWrite(TFT_CS_LEFT, HIGH);
    digitalWrite(TFT_CS_RIGHT, HIGH);

    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);

    pinMode(PIN_IN_LEVER, INPUT_PULLDOWN);
    pinMode(PIN_IN_WATER_LOW, INPUT_PULLDOWN);
    pinMode(PIN_IN_WATER_FULL, INPUT_PULLDOWN);
    pinMode(PIN_OUT_HEAT, OUTPUT);
    pinMode(PIN_OUT_PUMP, OUTPUT);
    pinMode(PIN_OUT_FILL_SOLENOID, OUTPUT);

    failsafe();
}

void setHeater(uint8_t duty) {
    // TODO: PWM output for this
    digitalWrite(PIN_OUT_HEAT, (duty > 0));
}

void setPump(uint8_t duty) {
    // TODO: PWM output for this
    digitalWrite(PIN_OUT_PUMP, (duty > 0));
}

void setWaterFillSolenoid(bool fill) {
    digitalWrite(PIN_OUT_FILL_SOLENOID, fill);
    // TODO: Set a watchdog timer that turns this off after X seconds
}

bool isMaxSampleReady() {
    return (digitalRead(MAX_RDY) == LOW);
}

bool isLeverPulled() {
    return (digitalRead(PIN_IN_LEVER) == HIGH);
}

bool isWaterTankLow() {
    return (digitalRead(PIN_IN_WATER_LOW) == HIGH);
}

void initDisplay() {
    Serial.println("Initialize LCD...");

    digitalWrite(TFT_CS_LEFT, LOW);
    tft.begin();
    tft.fillScreen(TFT_BLACK);
    digitalWrite(TFT_CS_LEFT, HIGH);

    digitalWrite(TFT_CS_RIGHT, LOW);
    tft.begin();
    tft.fillScreen(TFT_BLACK);
    digitalWrite(TFT_CS_RIGHT, HIGH);

    // Allocate a buffer for the display
    gfx_left.setColorDepth(8);
    gfx_right.setColorDepth(8);
    if (gfx_left.createSprite(TFT_WIDTH, TFT_HEIGHT) == nullptr || 
        gfx_right.createSprite(TFT_WIDTH, TFT_HEIGHT) == nullptr)
    {
        Serial.println("ERROR: display buffer allocation failed!");
        return;
    }

    gfx_left.setTextSize(2);
    gfx_right.setTextSize(2);
}

void tftClearCanvas() {
    gfx_left.fillSprite(TFT_BLACK);
    gfx_right.fillSprite(TFT_BLACK);
}

void tftUpdateDisplay() {
    digitalWrite(MAX_CS, HIGH);

    digitalWrite(TFT_CS_RIGHT, LOW);
    gfx_right.pushSprite(0,0);
    digitalWrite(TFT_CS_RIGHT, HIGH);

    digitalWrite(TFT_CS_LEFT, LOW);
    gfx_left.pushSprite(0,0);
    digitalWrite(TFT_CS_LEFT, HIGH);

    // Enable backlight
    digitalWrite(TFT_BL, HIGH);
}

void initPressure() {
    Serial.println("Initialize Pressure");
    if (!pressure.begin()) {
        Serial.println("ERROR: No response from pressure transducer");
    }
    else {
        isPressureAvailable = true;
    }
}

void initTemperature() {
    Serial.println("Initialize MAX31865");

    pinMode(MAX_RDY, INPUT);

    rtd.begin(MAX31865_3WIRE);
    rtd.enableBias(true);
    rtd.enable50Hz(true);
    
    //Serial.println(rtd.readRegister8(MAX31865_CONFIG_REG), HEX);
    //Serial.println("ERROR: No response from MAX");

    rtd.readRTD();
    auto fault = rtd.readFault();
    if (fault) {
        Serial.print("Fault 0x"); Serial.println(fault, HEX);
        if (fault & MAX31865_FAULT_HIGHTHRESH) {
            Serial.println("RTD High Threshold"); 
        }
        if (fault & MAX31865_FAULT_LOWTHRESH) {
            Serial.println("RTD Low Threshold"); 
        }
        if (fault & MAX31865_FAULT_REFINLOW) {
            Serial.println("REFIN- > 0.85 x Bias"); 
        }
        if (fault & MAX31865_FAULT_REFINHIGH) {
            Serial.println("REFIN- < 0.85 x Bias (FORCE- open)"); 
        }
        if (fault & MAX31865_FAULT_RTDINLOW) {
            Serial.println("RTDIN- < 0.85 x Bias (FORCE- open)"); 
        }
        if (fault & MAX31865_FAULT_OVUV) {
            Serial.println("Under/Over voltage"); 
        }
    }
    else {
        isRtdAvailable = true;
        rtd.autoConvert(true);

        // Wait for a sample to come in
        auto t1 = millis();
        while (((millis() - t1) < 1000) && (!rtd.isSampleReady()))
            continue;
        
        if (!rtd.isSampleReady()) {
            Serial.println("Error: No sample");
        }
        else if (rtd.readSample() == 0) {
            Serial.println("Error: Sample is zero");
        }

        isRtdAvailable = true;
    }
}

void initFlow() {
    Serial.println("Initialize Pulse Counter");

    pinMode(FLOW_PULSE_PIN, INPUT);
    PulseCounter1.begin(FLOW_PULSE_PIN);
    isFlowAvailable = true;
}


void uiRenderFirmwareUpdate(OtaState state, int param) {
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

void uiRenderWiFiConnect(WifiConnectionStatus state, wl_status_t err = (wl_status_t)0) {
    tftClearCanvas();

    uiRenderLabelCentered(gfx_right, "WIFI CONNECT", -60, TFT_WHITE);

    switch (state) {
        case WifiConnectionStatus::Connecting:
        {
            uiRenderLabelCentered(gfx_right, "CONNECTING TO", 0, TFT_WHITE);
            uiRenderLabelCentered(gfx_right, secrets::wifi_ssid, 30, TFT_LIGHTGREY);
            break;
        }
            break;

        case WifiConnectionStatus::Connected:
        {
            auto ip_str = WiFi.localIP().toString();
            uiRenderLabelCentered(gfx_right, "CONNECTED", 0, TFT_WHITE);
            uiRenderLabelCentered(gfx_right, ip_str.c_str(), 30, TFT_LIGHTGREY);
            break;
        }

        case WifiConnectionStatus::Failure:
        {
            uiRenderLabelCentered(gfx_right, "ERROR", 0, TFT_WHITE);

            const char* err_msg = nullptr;
            switch (err) {
                case WL_NO_SSID_AVAIL:
                    // Bad SSID or AP not present
                    err_msg = "No SSID";
                    break;
                case WL_CONNECT_FAILED:
                    // Bad password
                    err_msg = "Bad Password";
                    break;
                case WL_IDLE_STATUS:
                    err_msg = "Station Idle";
                    break;
                case WL_CONNECTION_LOST:
                    err_msg = "Connection Lost";
                    break;
            }

            if (err_msg != nullptr) {
                uiRenderLabelCentered(gfx_right, err_msg, 30, TFT_LIGHTGREY);
            }
            break;
        }
    }

    tftUpdateDisplay();
}

void initWiFi()
{
    wl_status_t result;

    WiFi.persistent(false);

    while (WiFi.status() != WL_CONNECTED) {
        uiRenderWiFiConnect(WifiConnectionStatus::Connecting);
        Serial.println();

        WiFi.mode(WIFI_STA);
        WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
        WiFi.setHostname(secrets::device_name);
        WiFi.begin(secrets::wifi_ssid, secrets::wifi_pw);
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
}

void initOTA() {
    ArduinoOTA.setHostname(DEVICE_NAME);


	// No authentication by default
	// ArduinoOTA.setPassword((const char *)"123");

	// Password can be set with it's md5 value as well
	// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

	ArduinoOTA.onStart([]() {
        // Put system into a safe state
		failsafe();
        uiState = UiState::FirmwareUpdate;
		Serial.println("OTA Initiated");

        uiRenderFirmwareUpdate(OtaState::Begin, 0);
        tftUpdateDisplay();
	});

	ArduinoOTA.onEnd([]() {
		Serial.println("OTA Done!");
        uiRenderFirmwareUpdate(OtaState::Success, 100);
	});

	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        // Throttle display updates to avoid slowing down the update
		static int last_x = 0;
		int x = (progress / (total / 20));
		if (x != last_x) {
		 	last_x = x;
            int p = (progress * 100) / total;
            uiRenderFirmwareUpdate(OtaState::Progress, p);
        }
	});

	ArduinoOTA.onError([](ota_error_t error) {
		Serial.println();
		Serial.printf("Error[%u]: ", error);
        
        uiState = UiState::Fault;
        uiFault = FaultState::FirmwareUpdateFailure;

		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");

        uiRenderFirmwareUpdate(OtaState::Failure, (int)error);

		delay(1000);
		ESP.restart();
	});

	// Enable OTA
	ArduinoOTA.begin();
}

void setup() {
    Serial.begin(9600);

    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();

    initGpio();
    initDisplay();
    initWiFi();
    initOTA();
    
    initPressure();
    initTemperature();
    initFlow();
    

    //uiState = UiState::Preheat;
    uiState = UiState::SensorTest;

    // If sensors could not be initialized, indicate fault
    if (uiState != UiState::SensorTest && 
        (!isRtdAvailable || !isFlowAvailable))// || !isPressureAvailable))
    {
        uiFault = FaultState::SensorFailure;
        uiState = UiState::Fault;
        return;
    }


    Serial.println("Done!");
}

void uiDrawStatusCircle(TFT_eSprite& gfx) {
    uint32_t ring_w = 10;
    unsigned long t = millis();


    const char* status_str = nullptr;
    uint32_t color = TFT_WHITE;

    switch (uiState) {
        case UiState::Init:
            status_str = "INIT";
            color = TFT_BLUE;
            break;

        case UiState::Preheat:
        {
            status_str = "PREHEATING";
            color = TFT_ORANGERED;

            // Pulse animation
            
            ring_w += 5 + (sinf((t / 10.0f) * deg2rad + PI) * 5.0f);

            break;
        }

        case UiState::Fault:
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
            float f = sinf((t / 2.0f) * deg2rad + PI) * 0.5f + 0.5f;
            int16_t c = f*f*255.0f;
            color = TFT_RGB656(c, 0, 0);
            break;
        }

        case UiState::Ready:
            // TODO: Connect the above pulse animation and relax back into an idle state

            //status_str = "READY";
            color = TFT_DARKGREEN;

            break;

        case UiState::FirmwareUpdate:
            status_str = "UPDATING";
            color = TFT_SKYBLUE;
            break;

        default:
            return;
    }

    gfx.drawSmoothArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-ring_w, 0, 360, color, TFT_BLACK);

    if (status_str != nullptr) {
        int16_t tw = gfx.textWidth(status_str);
        int16_t th = 14;
        gfx.setCursor(TFT_WIDTH/2 - tw/2, TFT_HEIGHT/2 - th/2);
        gfx.print(status_str);
    }
}

void uiRenderMargin() {
    // uint32_t margin = 5;
    // uint32_t ring_w = 10 + margin;
    // gfx_right.drawArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-ring_w, 0, 360, TFT_BLACK, TFT_BLACK, false);
}

void uiRenderGauge() {
    //float value_norm = (values.t - 15.0f) / (25.0f - 15.0f);
    float min_pressure = 1.0f; // Ambient air pressure
    float max_pressure = 12.0f;
    float value_norm = (values.p * 0.001f - min_pressure) / (max_pressure - min_pressure);
    if (value_norm < 0.0f) value_norm = 0.0f;
    if (value_norm > 1.0f) value_norm = 1.0f;
    uint32_t min_angle = 90-30;
    uint32_t max_angle = 270+30;
    uint32_t angle = (value_norm * (max_angle - min_angle)) + min_angle;
    // if (angle > max_angle) angle = max_angle;
    // if (angle < min_angle) angle = min_angle;
    gfx_right.drawSmoothArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-10, min_angle, max_angle, TFT_DARKESTGREY, TFT_BLACK, true);
    if (angle > min_angle) {
        gfx_right.drawSmoothArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-10, min_angle, angle, TFT_DARKGREEN, TFT_BLACK, true);
    }
}

unsigned long t_last = 0;

void uiRenderLabelCentered(TFT_eSprite& gfx, const char* s, int16_t y, uint16_t color = TFT_WHITE) {
    // Place below the status label
    int16_t tw = gfx.textWidth(s);
    int16_t th = 14; // TODO: Measure from font
    gfx.setCursor(TFT_WIDTH/2 - tw/2, TFT_HEIGHT/2 - th/2 + y);
    gfx.setTextColor(color);
    gfx.print(s);
}

void uiRenderTemperatureGraph(TFT_eSprite& gfx, uint16_t color = TFT_WHITE) {
    float min_value = 10.0f;
    float max_value = 30.0f;

    int32_t margin = 20;
    int32_t x = margin;
    int32_t y = TFT_HEIGHT / 2; // Vertically centered
    int32_t h = TFT_HEIGHT / 3;
    int32_t w = TFT_WIDTH - (margin * 2);

    // The sample array may be greater than the width.
    // Offset into the array so that the last sample drawn is the most recent available.
    int start = temperatureSamples.size() - w;
    if (start < 0) start = 0;

    for (int i = start; i < temperatureSamples.size(); i++) {
        auto sample = temperatureSamples[i];

        if (sample < min_value) sample = min_value;
        else if (sample > max_value) sample = max_value;

        int32_t offset = (((sample - min_value) / (max_value - min_value)) * h);
        int32_t yy = y + h - offset;
        gfx.drawPixel(x, yy, color);
        x++;
    }


    //gfx_right.drawLine()
}

void uiRenderBrewGraph(TFT_eSprite& gfx) {
    int32_t margin = 20;
    int32_t x = margin;
    int32_t y = TFT_HEIGHT / 2; // Vertically centered
    int32_t h = TFT_HEIGHT / 3;
    int32_t w = TFT_WIDTH - (margin * 2);

    {
        float min_value = 1000.0f;
        float max_value = 2000.0f;

        // The sample array may be greater than the width.
        // Offset into the array so that the last sample drawn is the most recent available.
        int start = pressureSamples.size() - w;
        if (start < 0) start = 0;

        for (int i = start; i < pressureSamples.size(); i++) {
            auto sample = pressureSamples[i];

            if (sample < min_value) sample = min_value;
            else if (sample > max_value) sample = max_value;

            int32_t offset = (((sample - min_value) / (max_value - min_value)) * h);
            int32_t yy = y + h - offset;
            gfx.drawPixel(x, yy, TFT_DARKGREEN);
            x++;
        }
    }

    {
        float min_value = 0.0f;
        float max_value = 200.0f;

        // The sample array may be greater than the width.
        // Offset into the array so that the last sample drawn is the most recent available.
        int start = flowSamples.size() - w;
        if (start < 0) start = 0;

        for (int i = start; i < flowSamples.size(); i++) {
            auto sample = flowSamples[i];

            if (sample < min_value) sample = min_value;
            else if (sample > max_value) sample = max_value;

            int32_t offset = (((sample - min_value) / (max_value - min_value)) * h);
            int32_t yy = y + h - offset;
            gfx.drawPixel(x, yy, TFT_SKYBLUE);
            x++;
        }
    }
}


void uiRenderPreheat() {

    uiRenderTemperatureGraph(gfx_right);
    uiRenderMargin();
    // Render current temperature
    char buffer[20];
    if (values.t_valid) {
        snprintf(buffer, sizeof(buffer), "%.2f C", values.t);
    } else {
        snprintf(buffer, sizeof(buffer), "- C");
    }
    buffer[sizeof(buffer)-1] = '\0';

    // Place below the status label
    uiRenderLabelCentered(gfx_right, buffer, 20);
}

void uiRenderReady() {
    // Left Display:
    //     - Temperature Gauge & Graph ??
    // Right Display:
    //     - Green ring indicating "ready"

    if (brew.start_brew_time > 0) {
        // Left Display:
        //     - Pressure & flow chart
        //     - Average pressure & flow
        //     - Average temperature
        // Right Display:
        //     - Final time
        //     - mL delivered

        // Display snapshot of last brew graph
        uiRenderBrewGraph(gfx_right);
        uiRenderMargin();

        float brewTimeSec = (brew.end_brew_time - brew.start_brew_time) / 1000.0f;
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%.1f", brewTimeSec);
        buffer[sizeof(buffer)-1] = '\0';

        uiRenderLabelCentered(gfx_right, "BREW TIME:", -20);
        uiRenderLabelCentered(gfx_right, buffer, 0);
    }
    else {
        uiRenderLabelCentered(gfx_right, "READY", 0);
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
        float brewTimeSec = (millis() - brew.start_brew_time) / 1000.0f;
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%.1f", brewTimeSec);
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, 20);
    }

    {
        // Render pressure
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%d", values.p);
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, 40);
    }

    uiRenderGauge();
}

void uiRenderSensorTest()
{

    uiRenderTemperatureGraph(gfx_right, TFT_RED);
    uiRenderBrewGraph(gfx_right);
    uiRenderMargin();

    {
        char buffer[20];
        if (values.p_valid) {
            snprintf(buffer, sizeof(buffer), "%d mBar", values.p);
        } else {
            snprintf(buffer, sizeof(buffer), "- mBar");
        }
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, -60, TFT_DARKGREEN);
    }

    {
        char buffer[20];
        if (values.f_valid) {
            snprintf(buffer, sizeof(buffer), "%.1f /s", values.f);
        } else {
            snprintf(buffer, sizeof(buffer), "- /s");
        }
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, -30, TFT_SKYBLUE);
    }


    {
        char buffer[20];
        if (values.f_valid) {
            snprintf(buffer, sizeof(buffer), "%.1f mL", values.f_accum);
        } else {
            snprintf(buffer, sizeof(buffer), "- mL");
        }
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, 0, TFT_DARKCYAN);
    }

    {
        char buffer[20];
        if (values.t_valid) {
            snprintf(buffer, sizeof(buffer), "%.1f C", values.t);
        } else {
            snprintf(buffer, sizeof(buffer), "- C");
        }
        buffer[sizeof(buffer)-1] = '\0';

        // Place below the status label
        uiRenderLabelCentered(gfx_right, buffer, 30, TFT_RED);
    }
}

void readSensors() {
    if (isPressureAvailable && !values.p_reading)
    {
        pressure.startSample();
        values.p_reading = true;

        // while (!pressure.isSampleReady())
        //     continue;
    }

    // while (!rtd.isSampleReady())
    //     continue;
    // while (digitalRead(MAX_RDY) != LOW)
    //     continue;

    if (isRtdAvailable && rtd.isSampleReady()) {// isMaxSampleReady()) {
        auto raw_rtd = rtd.readSample();
        //Serial.printf("T: %d\n", raw_rtd);
        values.t = rtd.calculateTemperature(raw_rtd, RTD_NOMINAL_RESISTANCE, RTD_REFERENCE_RESISTANCE);
        values.t_valid = (values.t > RTD_MIN_TEMP && values.t < RTD_MAX_TEMP);
    }

    //while (!FreqCountESP.available())
    // while (!PulseCounter1.isSampleReady())
    //     continue;
    // auto f = PulseCounter1.getFrequency();

    if (isFlowAvailable && PulseCounter1.isSampleReady()) {
        // Value typically ranges from 40-270 Hz
        values.f = PulseCounter1.getFrequency();
        values.f_valid = true;
    }

    if (isPressureAvailable && pressure.isSampleReady()) {
        auto r = pressure.readSample();
        values.p_valid = r.is_valid;
        values.p = r.pressure;
        values.p_reading = false;
    }

    // auto p = readPressure();
    // auto t = readTemperature();

    //auto t2 = rtd.readRTD();
    //float t2 = rtd.temperature(RTD_NOMINAL_RESISTANCE, RTD_REFERENCE_RESISTANCE);

    // Record a sample
    unsigned long t = millis();
    unsigned long sample_delta = (t - t_last);

    // Calculate time delta between ticks for the specified time window (approximate)
    const int32_t time_window = 30000;
    const unsigned long t_delta = (time_window / temperatureSamples.capacity());
    
    if (sample_delta >= t_delta) {
        t_last = t;

        if (values.t_valid) {
            temperatureSamples.add(values.t);
        }

        // Only update the graph for these if we are currently brewing,
        // so we can get a frozen snapshot at the end
        if (uiState == UiState::Brewing || uiState == UiState::SensorTest) {
            if (values.p_valid) {
                pressureSamples.add(values.p);
            }

            if (values.f_valid) {
                flowSamples.add(values.f);

                // Accumulate flow
                values.f_accum += values.f;
            }
        }
    }
}

bool isBootBtnPressed() {
    return (digitalRead(0) == LOW);
}

void printState(UiState uiState) {
    Serial.print("State: ");
    int s = (int)uiState;
    if (s < sizeof(UiState_Str)) {
        Serial.println(UiState_Str[s]);
    }
    else {
        Serial.println(s);
    }
}

void processState()
{
    static UiState _lastUiState = UiState::Init;
    if (uiState != _lastUiState) {
        _lastUiState = uiState;
        printState(uiState);
    }

    // NOTE: Process faults first.

/*
    if (uiState != UiState::SensorTest && 
        (!values.f_valid || !values.p_valid || !values.t_valid))
    {
        uiFault = FaultState::SensorFailure;
        uiState = UiState::Fault;
        return;
    }
*/

    // If water tank is low at any point, indicate fault
    if (isWaterTankLow()) {
        uiFault = FaultState::LowWater;
        uiState = UiState::Fault;
        return;
    }

    if (uiState == UiState::Preheat) {
        // Device is ready once temperature rises above the configured threshold
        if (values.t_valid && (values.t > CONFIG_PREHEAT_TEMPERATURE_C)) {
            uiState = UiState::Ready;
            return;
        }
        
        // TESTING:
        if (millis() > 5000) {
            uiState = UiState::Ready;
        }
    }

    if (uiState == UiState::Fault) {
        // If fault was low tank, and tank is no longer low, clear the fault
        if (uiFault == FaultState::LowWater && !isWaterTankLow()) {
            uiFault = FaultState::NoFault;
            uiState = UiState::Ready;
        }
    }

    // If lever is actuated at any time, move to brew phase.
    if ((uiState != UiState::Brewing) && isLeverPulled()) {
        uiState = UiState::Brewing;

        // Restart brew timer
        brew.start_brew_time = millis();
        brew.end_brew_time = 0;

        // Reset flow accumulation
        values.f_accum = 0.0f;
        return;
    }

    // If lever is released, stop brewing
    if (uiState == UiState::Brewing && !isLeverPulled()) {
        //uiState = UiState::PostBrew;
        uiState = UiState::Ready;

        // Stop brew timer
        brew.end_brew_time = millis();
        return;
    }
}

void render()
{
    // Drawing
    {
        gfx_left.fillSprite(TFT_BLACK);
        gfx_right.fillSprite(TFT_BLACK);

        //uiRenderGraph();

        uiDrawStatusCircle(gfx_right);
        uiDrawStatusCircle(gfx_left);

        switch (uiState) {
            case UiState::Preheat:
                uiRenderPreheat();
                break;
            case UiState::Ready:
                uiRenderReady();
                break;
            case UiState::Brewing:
                uiRenderBrewing();
                break;
            case UiState::SensorTest:
                uiRenderSensorTest();
                break;
            // case UiState::PostBrew:
            //     uiRenderPostBrew();
            //     break;
            default:
                break;
        }

#if false


        gfx.setCursor(30, 80);
        //lcd.printf("P: %.2f", p * 0.001f);
        if (values.p_valid) {
            gfx.printf("P: %.2f", values.p * 0.001f);
        } else {
            gfx.printf("P: -");
        }

        gfx.setCursor(30, 110);
        gfx.printf("F: %.2f", values.f);

        gfx.setCursor(30, 140);
        if (values.t_valid) {
            gfx.printf("T: %.2fC", values.t);
        } else {
            gfx.printf("T: -");
        }
#endif

        tftUpdateDisplay();
    }
}

void loop()
{
    ArduinoOTA.handle();

    if (uiState != UiState::FirmwareUpdate)
    {
        readSensors();

        processState();

        render();
    }
}
