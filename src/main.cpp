#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Adafruit_MAX31865.h>
#include <driver/timer.h>
#include <driver/pcnt.h>
#include <FreqCountESP.h>
#include "pressure_transducer.h"
#include "PulseCounter.h"
#include "hardware.h"

// 240x240px, 16-bit BGR pixel format
Adafruit_GC9A01A    lcd(LCD_CS, LCD_DC);
Adafruit_MAX31865   rtd(MAX_CS, MAX_MOSI, MAX_MISO, MAX_CLK);
PressureTransducer  pressure(PRESSURE_FULL_SCALE);

int counter = 0;


// volatile int16_t pulseCount = 0;
// volatile bool pulseValueReady = false;




// bool initPulseCounter() {
// }

// int getPulseCount() {
//     int16_t count = 0;
//     ESP_ERROR_CHECK(pcnt_get_counter_value(PCNT_UNIT_0, &count));
//     return count;
// }

void setup() {
    Serial.begin(9600);
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    Wire.setPins(I2C_SDA, I2C_SCL);
    //Wire.setClock();
    Wire.begin();
    
    Serial.println("Initialize LCD...");
    lcd.begin();
    lcd.fillScreen(0);
    lcd.setCursor(120, 120);
    lcd.setTextSize(2);
    //lcd.setTextColor();
    lcd.print("Init");

    Serial.println("Initialize Pressure");
    if (!pressure.begin()) {
        Serial.println("ERROR: No response from pressure transducer");
    }

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

    rtd.autoConvert(true);

    Serial.println("Initialize Pulse Counter");

    pinMode(FLOW_PULSE_PIN, INPUT_PULLUP);
    PulseCounter1.begin(FLOW_PULSE_PIN);
   

    Serial.println("Done!");
}

struct {
    float t;
    int p;
    float f;
} values;

void loop() {

    pressure.startSample();
    //rtd.startSample();

    while (!pressure.isSampleReady())
        continue;

    // while (!rtd.isSampleReady())
    //     continue;
    // while (digitalRead(MAX_RDY) != LOW)
    //     continue;

    if (digitalRead(MAX_RDY) == LOW) {
        auto raw_rtd = rtd.readSample();
        values.t = rtd.calculateTemperature(raw_rtd, RTD_NOMINAL_RESISTANCE, RTD_REFERENCE_RESISTANCE);
    }

    //while (!FreqCountESP.available())
    // while (!PulseCounter1.isSampleReady())
    //     continue;
    // auto f = PulseCounter1.getFrequency();

    if (PulseCounter1.isSampleReady()) {
        values.f = PulseCounter1.getFrequency();
    }

    auto p = pressure.readSample();


    // auto p = readPressure();
    // auto t = readTemperature();

    //auto t2 = rtd.readRTD();
    //float t2 = rtd.temperature(RTD_NOMINAL_RESISTANCE, RTD_REFERENCE_RESISTANCE);



    lcd.fillScreen(0);
    lcd.setCursor(30, 80);
    //lcd.printf("P: %.2f", p * 0.001f);
    if (p.is_valid) {
        lcd.printf("P: %.2f", p.pressure * 0.001f);
    } else {
        lcd.printf("P: -");
    }

    lcd.setCursor(30, 110);
    lcd.printf("F: %.2f", values.f);

    lcd.setCursor(30, 140);
    if (values.t > RTD_MIN_TEMP && values.t < RTD_MAX_TEMP) {
        lcd.printf("T: %.2fC", values.t);
    } else {
        lcd.printf("T: -");
    }

    delay(10);
}
