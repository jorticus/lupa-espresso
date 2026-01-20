
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "WebSrv.h"
#include "Debug.h"
#include "Data.h"

#include "SensorSampler.h"
#include "HeatControl.h"
#include "BrewControl.h"
#include "IO.h"

//static WebServer server(80);
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

static int clients_connected = 0;
static const unsigned long UPDATE_INTERVAL_MS = 500;

static const int JSON_BUFFER_SIZE = 1000;
static StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;


// Get Sensor Readings and return JSON object
void updateSensorBuffer(String& response) {
    jsonBuffer.clear();
    JsonObject& json = jsonBuffer.createObject();

    // Sensors
    json["t1"] = SensorSampler::getTemperature();
    json["t2"] = SensorSampler::getTemperature2();
    json["t3"] = SensorSampler::getEstimatedGroupheadTemperature();
    json["p1"] = SensorSampler::getPressure();
    json["flow"]  = SensorSampler::getFlowRate();
    json["vol"] = SensorSampler::getTotalFlowVolume();
    json["is_flowing"] = SensorSampler::isFlowing();

    // PID controller internal state
    json["pid_i"]  = HeatControl::pid_i.last();
    json["pid_d1"] = HeatControl::pid_d.last();
    json["pid_d2"] = HeatControl::pid_d2.last();
    json["t_sp"]   = HeatControl::getSetpoint();

    // Boiler state
    json["b_pwr"] = IO::getHeatPower();
    json["b_on"]  = IO::isHeaterOn();

    // I/O state
    json["pull"]  = IO::isLeverPulled();
    json["brew"]  = IO::isBrewing();

    // System
    json["mem"] = esp_get_free_heap_size();
    
    json.printTo(response);
}

void sendSensorReadings() {
    String response;
    updateSensorBuffer(response);

    ws.textAll(response);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    //data[len] = 0;
    //String message = (char*)data;
    // Check if the message is "getReadings"
    //if (strcmp((char*)data, "getReadings") == 0) {
      //if it is, send current sensor readings
        //String sensorReadings = getSensorReadings();
        sendSensorReadings();

    //}
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            //Debug.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            clients_connected++;
            break;
        case WS_EVT_DISCONNECT:
            //Debug.printf("WebSocket client #%u disconnected\n", client->id());
            if (clients_connected > 0) {
                clients_connected--;
            }
            break;
        case WS_EVT_DATA:
            //handleWebSocketMessage(arg, data, len);
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

static void handleTemperature(AsyncWebServerRequest * req) {
    String response;
    updateSensorBuffer(response);

    req->send(200, "application/json", response);
}

void WebSrv::setup() {
#if 0 // TODO: Fix crash
/*
assert failed: tcp_alloc /IDF/components/lwip/lwip/src/core/tcp.c:1854 (Required to lock TCPIP core functionality!)    a


Backtrace: 0x4037d809:0x3fcdc360 0x4037d7d1:0x3fcdc380 0x403848c2:0x3fcdc3a0 0x4203f9cf:0x3fcdc4e0 0x4203fb35:0x3fcdc50s0 0x4200a4d9:0x3fcdc520 0x42018b65:0x3fcdc570 0x42020f23:0x3fcdc590 0x4202187e:0x3fcdc610 0x4037eb7d:0x3fcdc630        
  #0  0x4037d809 in panic_abort at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/esp_system/panic.c:477
  #1  0x4037d7d1 in esp_system_abort at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/esp_system/port/esp_system_chip.c:87
  #2  0x403848c2 in __assert_func at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/newlib/src/assert.c:80  #3  0x4203f9cf in tcp_alloc at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/lwip/lwip/src/core/tcp.c:1854 (discriminator 1)
  #4  0x4203fb35 in tcp_new_ip_type at /home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/lwip/lwip/src/core/tcp.c:2010
  #5  0x4200a4d9 in AsyncServer::begin() at E:/Prog/PlatformIO/lib/AsyncTCP_ID1826/src/AsyncTCP.cpp:1255
  #6  0x42018b65 in AsyncWebServer::begin() at .pio/libdeps/lupa-espresso-v2/ESPAsyncWebServer/src/WebServer.cpp:119   
  #7  0x42020f23 in WebSrv::setup() at .pio/libdeps/lupa-espresso-v2/ArduinoJson/src/ArduinoJson/Serialization/JsonSerializerImpl.hpp:86
  #8  0x4202187e in taskNetworkFunc(void*) at .pio/libdeps/lupa-espresso-v2/ArduinoJson/src/ArduinoJson/Serialization/JsonSerializerImpl.hpp:86
*/
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        // source: data/index.html
        request->send_P(200, "text/html", data::index_html_bytes, data::index_html_size);
    });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        // source: data/script.js
        request->send_P(200, "application/javascript", data::script_js_bytes, data::script_js_size);
    });
    server.on("/sensors", handleTemperature);

    //server.serveStatic("/static/", SPIFFS, "/");
 
    // Allow remote access so we can run HTML from another server
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

    ws.onEvent(onEvent);
    server.addHandler(&ws);

    server.begin();
#endif
}

void WebSrv::process() {
    static unsigned long t_last = millis();

    //server.handleClient();
    ws.cleanupClients();

    if (clients_connected > 0) {
        auto now = millis();
        if ((now - t_last) > UPDATE_INTERVAL_MS) {
            t_last = now;

            sendSensorReadings();
        }
    }
}
