#include "WebSrv.h"
#include "Debug.h"

#include "HeatControl.h"
#include "SensorSampler.h"
#include "StateMachine.h"
#include "BrewControl.h"
#include "IO.h"

#include "Data.h"

#include <esp_http_server.h>
#include <esp_wifi.h>
#include <esp_event.h>

#include <ArduinoJson.h>
#include <cstring>
#include <string>

static httpd_handle_t s_server;
static httpd_req_t* s_wsreq;
static constexpr size_t JSON_CAP = 1024;
static constexpr size_t WS_RECV_CAP = 512;

static const int JSON_BUFFER_SIZE = 1000;
static StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;

static uint8_t s_wsBuffer[32];

static const unsigned long UPDATE_INTERVAL_MS = 500;

struct SensorPacket {
    uint8_t version;

    // Brew
    float   t1, t2, t3;
    float   p1;
    float   flow;
    float   vol;
    float   setpoint;
    float   avg_err;
    uint8_t is_flowing;

    // Boiler
    float   pid_i, pid_d1, pid_d2;
    float   t_sp;
    float   b_pwr;
    uint8_t b_on;

    // State
    uint8_t state;
    uint8_t pull;
    uint8_t brew;
    uint32_t mem;
} __attribute__((packed));

static size_t packSensorPacket(SensorPacket &pkt) {
    pkt.version = 1;
    pkt.t1 = SensorSampler::getTemperature();
    pkt.t2 = SensorSampler::getTemperature2();
    pkt.t3 = SensorSampler::getEstimatedGroupheadTemperature();
    pkt.p1 = SensorSampler::getPressure();
    pkt.flow = SensorSampler::getFlowRate();
    pkt.vol = SensorSampler::getTotalFlowVolume();
    pkt.setpoint = BrewControl::getCurrentSetpoint();
    pkt.avg_err = BrewControl::getTargetError();
    pkt.is_flowing = SensorSampler::isFlowing() ? 1 : 0;

    pkt.pid_i  = HeatControl::pid_i.last();
    pkt.pid_d1 = HeatControl::pid_d.last();
    pkt.pid_d2 = HeatControl::pid_d2.last();
    pkt.t_sp   = HeatControl::getSetpoint();

    pkt.b_pwr = IO::getHeatPower();
    pkt.b_on  = IO::isHeaterOn() ? 1 : 0;

    pkt.state = (uint8_t)State::getState();
    pkt.pull  = IO::isLeverPulled() ? 1 : 0;
    pkt.brew  = IO::isBrewing() ? 1 : 0;

    pkt.mem = esp_get_free_heap_size();

    return sizeof(pkt);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        Debug.println("Websocket opened");
        s_wsreq = req;
        return ESP_OK;
    }

    // esp_err_t ret;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = s_wsBuffer;
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(s_wsBuffer));
    if (ret != ESP_OK) {
        Debug.printf("Error receiving ws frame: %d\n", ret);
        return ret;
    }

    // if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && ws_pkt.payload != NULL) {
    //     // ...
    // }

    SensorPacket pkt;
    size_t pkt_len = packSensorPacket(pkt);

    // Debug.printf("Send WS Frame\n");
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    ws_pkt.payload = reinterpret_cast<uint8_t*>(&pkt);
    ws_pkt.len = pkt_len;

    ret = httpd_ws_send_frame(req, &ws_pkt);
    if (ret != ESP_OK) {
        Debug.printf("Error sending ws frame: %d\n", ret);
    }

    return ret;
}

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    // For any other URI send 404 and close socket
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "404 Not Found");
    return ESP_FAIL;
}


static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    Debug.println("Stopping webserver");
    httpd_stop(s_server);
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 3*1024;

    if (httpd_start(&s_server, &config) != ESP_OK) {
        Debug.println("WebSrv: failed to start server");
        s_server = nullptr;
        return;
    }

    static const httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = [] (httpd_req_t* req) -> esp_err_t { 
            httpd_resp_set_type(req, "text/html");
            httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
            return httpd_resp_send(req, (const char*)data::index_html_bytes, data::index_html_size);
        },
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_server, &index_uri);

    static const httpd_uri_t script_js_uri = {
        .uri = "/script.js",
        .method = HTTP_GET,
        .handler = [] (httpd_req_t* req) -> esp_err_t { 
            httpd_resp_set_type(req, "application/javascript");
            httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
            return httpd_resp_send(req, (const char*)data::script_js_bytes, data::script_js_size);
        },
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_server, &script_js_uri);

    static const httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = nullptr,
        .is_websocket = true
    };
    httpd_register_uri_handler(s_server, &ws_uri);

    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, http_404_error_handler);

    Debug.println("Webserver started");
}


void WebSrv::setup() {
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &s_server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &s_server));
}

void WebSrv::process() {

}
