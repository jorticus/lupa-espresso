#include "WebSrv.h"
#include "Debug.h"

#include "HeatControl.h"
#include "SensorSampler.h"
#include "IO.h"

#include "Data.h"

#include <esp_http_server.h>
#include <esp_wifi.h>
#include <esp_event.h>

#include <ArduinoJson.h>
#include <cstring>
#include <string>

static httpd_handle_t s_server;
static constexpr size_t JSON_CAP = 1024;
static constexpr size_t WS_RECV_CAP = 512;

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

// esp_err_t get_sensor_data(httpd_req_t *req) {
//     // char response[1024];
//     // snprintf(response, sizeof(response), 
//     //     "{\"t1\":%.2f,\"t2\":%.2f,\"t3\":%.2f,\"p1\":%.2f,\"flow\":%.2f,\"vol\":%.2f,\"is_flowing\":%s,"
//     //     "\"pid_i\":%.2f,\"pid_d1\":%.2f,\"pid_d2\":%.2f,\"t_sp\":%.2f,\"b_pwr\":%.2f,\"b_on\":%s,"
//     //     "\"pull\":%s,\"brew\":%s,\"mem\":%d}",
//     //     SensorSampler::getTemperature(),
//     //     SensorSampler::getTemperature2(),
//     //     SensorSampler::getEstimatedGroupheadTemperature(),
//     //     SensorSampler::getPressure(),
//     //     SensorSampler::getFlowRate(),
//     //     SensorSampler::getTotalFlowVolume(),
//     //     SensorSampler::isFlowing() ? "true" : "false",
//     //     HeatControl::pid_i.last(),
//     //     HeatControl::pid_d.last(),
//     //     HeatControl::pid_d2.last(),
//     //     HeatControl::getSetpoint(),
//     //     IO::getHeatPower(),
//     //     IO::isHeaterOn() ? "true" : "false",
//     //     IO::isLeverPulled() ? "true" : "false",
//     //     IO::isBrewing() ? "true" : "false",
//     //     esp_get_free_heap_size()
//     // );

//     char response[JSON_BUFFER_SIZE];
//     StaticJsonDocument<JSON_BUFFER_SIZE> jsonDoc;

//     jsonDoc["t1"] = SensorSampler::getTemperature();
//     jsonDoc["t2"] = SensorSampler::getTemperature2();
//     jsonDoc["t3"] = SensorSampler::getEstimatedGroupheadTemperature();
//     jsonDoc["p1"] = SensorSampler::getPressure();
//     jsonDoc["flow"] = SensorSampler::getFlowRate();
//     jsonDoc["vol"] = SensorSampler::getTotalFlowVolume();
//     jsonDoc["is_flowing"] = SensorSampler::isFlowing();
//     jsonDoc["pid_i"] = HeatControl::pid_i.last();
//     jsonDoc["pid_d1"] = HeatControl::pid_d.last();
//     jsonDoc["pid_d2"] = HeatControl::pid_d2.last();
//     jsonDoc["t_sp"] = HeatControl::getSetpoint();
//     jsonDoc["b_pwr"] = IO::getHeatPower();
//     jsonDoc["b_on"] = IO::isHeaterOn();
//     jsonDoc["pull"] = IO::isLeverPulled();
//     jsonDoc["brew"] = IO::isBrewing();
//     jsonDoc["mem"] = esp_get_free_heap_size();

//     serializeJson(jsonDoc, response);
//     httpd_resp_send(req, response, strlen(response));
//     return ESP_OK;

//     httpd_resp_set_type(req, "application/json");
//     httpd_resp_send(req, response, strlen(response));
//     return ESP_OK;
// }

// // HTTP GET /index
// static esp_err_t get_sensors(httpd_req_t* req)
// {
//     // String response;
//     // updateSensorBuffer(response);

//     httpd_resp_set_type(req, "text/html");
//     // httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
//     // httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
//     return httpd_resp_send(req, (const char*)data::index_html_bytes, data::index_html_size);
// }

// // HTTP GET /sensors -> returns JSON
// static esp_err_t get_sensors(httpd_req_t* req)
// {
//     String response;
//     updateSensorBuffer(response);

//     httpd_resp_set_type(req, "application/json");
//     httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
//     httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
//     return httpd_resp_send(req, response.c_str(), response.length());
// }

struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
};


static void ws_async_send(void *arg)
{
    static const char * data = "Async data";
    struct async_resp_arg *resp_arg = reinterpret_cast<struct async_resp_arg*>(arg);
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)data;
    ws_pkt.len = strlen(data);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
}

static esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
    struct async_resp_arg *resp_arg = new (struct async_resp_arg);
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    return httpd_queue_work(handle, ws_async_send, resp_arg);
}

/* WebSocket handler for /ws
   - on incoming frames: echo text frames back
   - you can extend this to push sensor JSON on demand or on a timer by storing connections
*/
static esp_err_t ws_handler(httpd_req_t *req) {
    uint8_t buf[128] = { 0 };
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = buf;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 128);
    if (ret != ESP_OK) {
        // TODO: Returns ESP_ERR_INVALID_STATE
        // Debug.printf("httpd_ws_recv_frame failed with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char*)ws_pkt.payload,"Trigger async") == 0) {
        return trigger_async_send(req->handle, req);
    }

    ret = httpd_ws_send_frame(req, &ws_pkt);
    if (ret != ESP_OK) {
        Debug.printf("httpd_ws_send_frame failed with %d", ret);
    }

    return ret;
#if 0
    httpd_ws_frame_t frame;
    std::memset(&frame, 0, sizeof(frame));
    uint8_t recv_buf[WS_RECV_CAP];
    frame.payload = recv_buf;

    while (true) {
        frame.len = sizeof(recv_buf);
        int ret = httpd_ws_recv_frame(req, &frame, 0);
        if (ret < 0) {
            // error receiving; end connection handling
            return ESP_FAIL;
        }

        if (frame.type == HTTPD_WS_TYPE_CONTINUE) {
            continue;
        }

        Debug.printf("WS: 0x%x %d\n", frame.type, frame.len);

        if (frame.type == HTTPD_WS_TYPE_TEXT) {
            // received text - check if client requested sensors
            const char* txt = reinterpret_cast<const char*>(frame.payload);
            if (frame.len >= 7 && std::memcmp(txt, "sensors", 7) == 0) {
                String payload;
                updateSensorBuffer(payload);
                
                httpd_ws_frame_t out;
                std::memset(&out, 0, sizeof(out));
                out.type = HTTPD_WS_TYPE_TEXT;
                out.payload = (uint8_t*)payload.c_str();
                out.len = payload.length();
                httpd_ws_send_frame(req, &out);
                continue;
            }

            // otherwise echo text back
            httpd_ws_frame_t out;
            std::memset(&out, 0, sizeof(out));
            out.type = HTTPD_WS_TYPE_TEXT;
            out.payload = frame.payload;
            out.len = frame.len;
            httpd_ws_send_frame(req, &out);
        } else if (frame.type == HTTPD_WS_TYPE_CLOSE || frame.type == HTTPD_WS_TYPE_PING) {
            // respond to close/ping automatically or break for close
            if (frame.type == HTTPD_WS_TYPE_CLOSE) {
                return ESP_OK;
            }
            // ignore/continue for ping/pong handled by server if needed
        } else {
            // binary or other types - ignore for now
        }

        // reset length for next recv
        
    }

    // unreachable
    return ESP_OK;
#endif
}

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    // if (strcmp("/hello", req->uri) == 0) {
    //     httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
    //     /* Return ESP_OK to keep underlying socket open */
    //     return ESP_OK;
    // } else if (strcmp("/echo", req->uri) == 0) {
    //     httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
    //     /* Return ESP_FAIL to close underlying socket */
    //     return ESP_FAIL;
    // }
    /* For any other URI send 404 and close socket */
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
    static const httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    Debug.println("Starting webserver...");

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

    static const httpd_uri_t sensor_uri = {
        .uri = "/sensors",
        .method = HTTP_GET,
        .handler = [] (httpd_req_t* req) -> esp_err_t {
            String response;
            updateSensorBuffer(response);

            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            return httpd_resp_send(req, response.c_str(), response.length());
        },
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_server, &sensor_uri);

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
