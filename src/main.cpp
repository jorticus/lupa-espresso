#include <Arduino.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_GC9A01A.h>
//#include <Arduino_GFX.h>
#include <Wire.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_MAX31865.h>
#include "pressure_transducer.h"
#include "PulseCounter.h"
#include <TFT_eSPI.h>
#include "hardware.h"

TFT_eSPI    tft;
TFT_eSprite gfx { &tft };

Adafruit_MAX31865   rtd(MAX_CS, MAX_MOSI, MAX_MISO, MAX_CLK);
PressureTransducer  pressure(PRESSURE_FULL_SCALE);

bool isRtdAvailable = false;
bool isPressureAvailable = false;
bool isFlowAvailable = false;

int counter = 0;

void initDisplay() {
    Serial.println("Initialize LCD...");

    tft.begin();
    tft.fillScreen(TFT_BLACK);

    // lcd.begin();
    // lcd.fillScreen(0);
    tft.setCursor(120, 120);
    tft.setTextSize(2);
    //lcd.setTextColor();
    tft.print("Init");

    // Allocate a buffer for the display
    gfx.setColorDepth(8);
    if (gfx.createSprite(TFT_WIDTH, TFT_HEIGHT) == nullptr) {
        Serial.println("ERROR: display buffer allocation failed!");
        tft.fillScreen(TFT_RED);
    }
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

    rtd.begin(MAX31865_3WIRE);
    rtd.enableBias(true);
    rtd.enable50Hz(true);
    pinMode(MAX_RDY, INPUT);
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
    }
}

void initFlow() {
    Serial.println("Initialize Pulse Counter");

    pinMode(FLOW_PULSE_PIN, INPUT_PULLUP);
    PulseCounter1.begin(FLOW_PULSE_PIN);
    isFlowAvailable = true;

    gfx.fillSprite(TFT_BLACK);
    gfx.pushSprite(0,0);
    gfx.setTextSize(2);
}

void setup() {
    Serial.begin(9600);
    
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    

    initDisplay();
    initPressure();
    initTemperature();
    initFlow();

    Serial.println("Done!");
}

struct {
    float t;
    bool t_valid;

    int p;
    bool p_valid;

    float f;
    bool f_valid;
} values = {0};

void loop() {

    if (isPressureAvailable)
    {
        pressure.startSample();

        while (!pressure.isSampleReady())
            continue;
    }

    // while (!rtd.isSampleReady())
    //     continue;
    // while (digitalRead(MAX_RDY) != LOW)
    //     continue;

    if (isRtdAvailable) {
        if (digitalRead(MAX_RDY) == LOW) {
            auto raw_rtd = rtd.readSample();
            values.t = rtd.calculateTemperature(raw_rtd, RTD_NOMINAL_RESISTANCE, RTD_REFERENCE_RESISTANCE);
            values.t_valid = (values.t > RTD_MIN_TEMP && values.t < RTD_MAX_TEMP);
        }
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

    if (isPressureAvailable) {
        auto r = pressure.readSample();
        values.p_valid = r.is_valid;
        values.p = r.pressure;
    }


    // auto p = readPressure();
    // auto t = readTemperature();

    //auto t2 = rtd.readRTD();
    //float t2 = rtd.temperature(RTD_NOMINAL_RESISTANCE, RTD_REFERENCE_RESISTANCE);


    gfx.fillSprite(TFT_BLACK);
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

    gfx.pushSprite(0,0);
    delay(10);
}
