#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Adafruit_MAX31865.h>
#include "pressure_transducer.h"
#include "hardware.h"

// 240x240px, 16-bit BGR pixel format
Adafruit_GC9A01A    lcd(LCD_CS, LCD_DC);
Adafruit_MAX31865   rtd(MAX_CS, MAX_MOSI, MAX_MISO, MAX_CLK);
PressureTransducer  pressure(PRESSURE_FULL_SCALE);

int counter = 0;

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
    Serial.println(rtd.readRegister8(MAX31865_CONFIG_REG), HEX);
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

    Serial.println("Done!");
}

void loop() {

    pressure.startSample();

    while (!pressure.isSampleReady())
        continue;

    auto p = pressure.readSample();
    
    // auto p = readPressure();
    // auto t = readTemperature();

    //auto t2 = rtd.readRTD();
    float t2 = rtd.temperature(RTD_NOMINAL_RESISTANCE, RTD_REFERENCE_RESISTANCE);


    lcd.fillScreen(0);
    lcd.setCursor(30, 80);
    //lcd.printf("P: %.2f", p * 0.001f);
    if (p.is_valid) {
        lcd.printf("P: %.2f", p.pressure * 0.001f);
    } else {
        lcd.printf("P: -");
    }

    lcd.setCursor(30, 140);
    if (t2 > RTD_MIN_TEMP && t2 < RTD_MAX_TEMP) {
        lcd.printf("T: %.2fC", t2);
    } else {
        lcd.printf("T: -");
    }

    delay(10);
}
