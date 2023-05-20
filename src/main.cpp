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
#include "PressureTransducer.h"
#include "PulseCounter.h"
#include <TFT_eSPI.h>
#include "config.h"
#include "hardware.h"
#include "value_array.h"
#include "SensorSampler.h"

#define TFT_RGB656(r,g,b)      ((((r & 0xFF) >> 3) << 11) | (((g & 0xFF) >> 2) << 5) | ((b & 0xFF) >> 3))
#define TFT_DARK_GOLDENROD  (TFT_RGB656(0xB8,0x86,0x0B))
#define TFT_ORANGERED       (TFT_RGB656(0xFF,0x45,0x00))
#define TFT_DARKESTGREY     0x38E7


const char* DEVICE_NAME = "LUPA";

TFT_eSPI    tft;
TFT_eSprite gfx_left  { &tft };
TFT_eSprite gfx_right { &tft };

constexpr float deg2rad      = 3.14159265359/180.0;

Adafruit_MAX31865   rtd(MAX_CS, &tft.getSPIinstance());
PressureTransducer  pressure(PRESSURE_FULL_SCALE);


const int numSamples = 300;
extern ValueArray<float, numSamples> temperatureSamples;
extern ValueArray<float, numSamples> pressureSamples;
extern ValueArray<float, numSamples> flowSamples;

ValueArray<float, numSamples> pressureSamplesFrozen;
ValueArray<float, numSamples> flowSamplesFrozen;

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

// struct {
//     float t;
//     bool t_valid;

//     int p;
//     bool p_valid;
//     bool p_reading;

//     float f;
//     bool f_valid;
//     float f_accum;
// } values = {0};

struct {
    unsigned long start_brew_time;
    unsigned long end_brew_time;

    float total_flow;
} brew = {0};

//unsigned long startBrewTime = 0;

// ValueArray<float, TFT_WIDTH> temperatureSamples;
// ValueArray<float, TFT_WIDTH> pressureSamples;
// ValueArray<float, TFT_WIDTH> flowSamples;

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
    //return (digitalRead(PIN_IN_LEVER) == HIGH);

    // Detect lever by measuring water flow
    return (
        (uiState != UiState::Preheat) && 
        SensorSampler::isFlowRateValid() && 
        (SensorSampler::getFlowRate() > 1.0f)
    );
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
//    PulseCounter1.begin(FLOW_PULSE_PIN);
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

    SensorSampler::Initialize();
    SensorSampler::Start();
    

    uiState = UiState::Preheat;
    //uiState = UiState::SensorTest;

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
    uint32_t color = TFT_BLACK;

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
            //color = TFT_DARKGREEN;

            break;

        case UiState::FirmwareUpdate:
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

void uiRenderMargin() {
    // uint32_t margin = 5;
    // uint32_t ring_w = 10 + margin;
    // gfx_right.drawArc(TFT_WIDTH/2, TFT_HEIGHT/2, TFT_WIDTH/2, (TFT_WIDTH/2)-ring_w, 0, 360, TFT_BLACK, TFT_BLACK, false);
}

void uiRenderGauge(TFT_eSprite& gfx, float value_norm, uint32_t fg_color, int32_t margin = 0) {
    int32_t ring_w = 10;

    if (value_norm < 0.0f) value_norm = 0.0f;
    if (value_norm > 1.0f) value_norm = 1.0f;
    uint32_t min_angle = 90-30;
    uint32_t max_angle = 270+30;
    uint32_t angle = (value_norm * (max_angle - min_angle)) + min_angle;
    // if (angle > max_angle) angle = max_angle;
    // if (angle < min_angle) angle = min_angle;

    int32_t x = TFT_WIDTH/2;
    int32_t y = TFT_HEIGHT/2;
    int32_t r1 = TFT_WIDTH/2 - margin;
    int32_t r2 = r1 - ring_w;
    gfx.drawSmoothArc(x, y, r1, r2, min_angle, max_angle, TFT_DARKESTGREY, TFT_BLACK, true);
    if (angle > min_angle) {
        gfx.drawSmoothArc(x, y, r1, r2, min_angle, angle, fg_color, TFT_BLACK, true);
    }
}

void uiRenderPressureGauge(TFT_eSprite& gfx) {
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
    if ((uiState != UiState::Preheat) && (SensorSampler::getTemperature() >= 80.0f)) {
        // Use smaller range when up to temperature
        return 80.0f;
    }
    else {
        return 20.0f;
    }
}

void uiRenderTemperatureGauge(TFT_eSprite& gfx) {
    float temperature = SensorSampler::getTemperature();
    float min_temp = getTemperatureMinRange();
    float max_temp = getTemperatureMaxRange();
    float value_norm = (temperature - min_temp) / (max_temp - min_temp);
    uiRenderGauge(gfx, value_norm, TFT_RED);
}

void uiRenderFlowGauge(TFT_eSprite& gfx) {
    float flow = SensorSampler::getFlowRate();
    float min_flow = 0.0f;
    float max_flow = 10.0f;
    float value_norm = (flow - min_flow) / (max_flow - min_flow);
    uiRenderGauge(gfx, value_norm, TFT_SKYBLUE, 15);
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

template <typename T, size_t N>
void uiRenderGraph(TFT_eSprite& gfx, ValueArray<T,N>& samples, float min_value, float max_value, uint16_t color = TFT_WHITE) {

    int32_t margin = 40;
    int32_t x = margin;
    int32_t y = TFT_HEIGHT / 2; // Vertically centered
    int32_t h = TFT_HEIGHT / 3;
    int32_t w = TFT_WIDTH - (margin * 2);

    // The sample array may be greater than the width.
    // Offset into the array so that the last sample drawn is the most recent available.
    int start = samples.size() - w;
    if (start < 0) start = 0;

    // Align the graph to the right side of the screen
    int offset = w - samples.size();
    if (offset < 0) offset = 0;
    x += offset;

    int32_t last_x = -1;
    int32_t last_y = -1;
    for (int i = start; i < samples.size(); i++) {
        auto sample = samples[i];

        if (sample < min_value) sample = min_value;
        else if (sample > max_value) sample = max_value;

        int32_t offset = (((sample - min_value) / (max_value - min_value)) * h * 2);
        int32_t yy = y + h - offset;
        int32_t d = last_y - yy;
        if ((last_y != -1) && (d > 1 || d < -1)) {
            gfx.drawLine(last_x, last_y, x, yy, color);
        }
        else {
            gfx.drawPixel(x, yy, color);
        }
        last_x = x;
        last_y = yy;
        x++;
    }
}

void uiFreezeGraphs() {
    for (int i = 0; i < pressureSamples.size(); i++) {
        pressureSamplesFrozen.add(pressureSamples[i]);
    }
    for (int i = 0; i < flowSamples.size(); i++) {
        flowSamplesFrozen.add(flowSamples[i]);
    }
}

void uiRenderTemperatureGraph(TFT_eSprite& gfx, uint16_t color = TFT_WHITE) {
    float min_value = getTemperatureMinRange();
    float max_value = getTemperatureMaxRange();
    uiRenderGraph(gfx, temperatureSamples, min_value, max_value, color);
}

void uiRenderBrewGraph(TFT_eSprite& gfx, bool freeze = false) {
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

        uiRenderLabelCentered(gfx_left, buffer, -15);
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

        uiRenderLabelCentered(gfx_left, buffer, 15);
    }

    uiRenderTemperatureGauge(gfx_left);
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

    if (brew.start_brew_time > 0) {
        // Left Display:
        //     - Pressure & flow chart
        //     - Average pressure & flow
        //     - Average temperature
        // Right Display:
        //     - Final time
        //     - mL delivered

        // Display snapshot of last brew graph
        uiRenderBrewGraph(gfx_right, true);

        float brewTimeSec = (brew.end_brew_time - brew.start_brew_time) / 1000.0f;
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%.1f", brewTimeSec);
        buffer[sizeof(buffer)-1] = '\0';

        uiRenderLabelCentered(gfx_right, "BREW TIME:", -20);
        uiRenderLabelCentered(gfx_right, buffer, 0);
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
        if (SensorSampler::isTemperatureValid() && (SensorSampler::getTemperature() > CONFIG_PREHEAT_TEMPERATURE_C)) {
            uiState = UiState::Ready;
            return;
        }
        
        // TESTING:
        // if (millis() > 5000) {
        //     uiState = UiState::Ready;
        // }
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
        SensorSampler::resetFlowCounter();
        return;
    }

    // If lever is released, stop brewing
    if (uiState == UiState::Brewing && !isLeverPulled()) {
        //uiState = UiState::PostBrew;
        uiState = UiState::Ready;

        uiFreezeGraphs();

        // Stop brew timer
        brew.end_brew_time = millis();
        return;
    }

    if (uiState == UiState::Ready && ((millis() - brew.end_brew_time) > 60000)) {
        // Reset ready page after some timeout
        brew.end_brew_time = 0;
        brew.start_brew_time = 0;
    }
}

void render()
{
    // Drawing
    {
        auto t1 = millis();

        gfx_left.fillSprite(TFT_BLACK);
        gfx_right.fillSprite(TFT_BLACK);


        switch (uiState) {
            case UiState::Preheat:
                uiRenderPreheat();
                uiRenderTemperatureLeft();
                break;
            case UiState::Ready:
                uiRenderReady();
                uiRenderTemperatureLeft();
                break;
            case UiState::Brewing:
                uiRenderBrewing();
                uiRenderTemperatureLeft();
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

        uiDrawStatusCircle(gfx_right);

        auto t2 = millis();

        tftUpdateDisplay();

        auto t3 = millis();

        //Serial.printf("Render %dms Update %dms\n", (t2-t1), (t3-t2));
    }
}

void loop()
{
    ArduinoOTA.handle();

    if (uiState != UiState::FirmwareUpdate)
    {
        processState();

        render();
    }
}
