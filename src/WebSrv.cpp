
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "WebSrv.h"
#include "Debug.h"

#include "SensorSampler.h"
#include "HeatControl.h"
#include "PressureControl.h"
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
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    server.on("/sensors", handleTemperature);
    //server.serveStatic("/static/", SPIFFS, "/");

    // Allow remote access so we can run HTML from another server
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

    ws.onEvent(onEvent);
    server.addHandler(&ws);

    server.begin();
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
