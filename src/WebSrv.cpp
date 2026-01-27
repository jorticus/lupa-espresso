/**
 * Light-weight webserver with websockets for providing real-time data
 * 
 * This has had a bit of a journey:
 * - Arduino WebServerAsync - too unstable, slow fetching HTML
 * - ESP-IDF httpd - uses heap allocations and was causing OOM, slow fetching HTML
 * - Mongoose - Fast! Memory efficient! No problems!
 * 
 */

#include "WebSrv.h"
#include "Debug.h"
#include "StaticHeap.h"

#include "HeatControl.h"
#include "SensorSampler.h"
#include "StateMachine.h"
#include "BrewControl.h"
#include "IO.h"
#include "config.h"

#include "Data.h"

#include <mongoose.h>

// #include <esp_http_server.h>
#include <esp_wifi.h>
#include <esp_event.h>

#include <cstring>
#include <string>
#include <memory_resource>

#define MG_C_STR(a) { (char *) (a), sizeof(a) - 1 }

static constexpr std::size_t HEAP_SIZE = 4 * 1024;
static StaticHeap heap(HEAP_SIZE);

static const size_t CHUNK_SIZE = 512;

extern "C" {

void* mg_calloc(std::size_t n, std::size_t size) {
    return heap.calloc(n, size);
}

void mg_free(void* ptr) {
    return heap.free(ptr);
}

} // extern "C"

static struct mg_mgr s_mgr;
static struct mg_connection *s_listener = nullptr;
static TaskHandle_t s_mg_task = nullptr;
static volatile bool s_mg_running = false;

struct SensorPacket {
    uint8_t packet_type; // 1

    // Brew
    float   t1, t2, t3;
    float   p1;
    float   flow;
    float   vol;
    float   setpoint;
    float   deviation;
    float   effort;
    float   conductance;
    float   dP;
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

struct BrewStatsPacket {
    uint8_t packet_type; // 2

    unsigned long start_brew_time;
    unsigned long end_brew_time;
    float preinfuse_volume;
    float total_volume;
    float avg_target_error;

    float avg_brew_pressure;
    int brew_pressure_avg_count;
} __attribute__((packed));

struct MetadataPacket {
    uint8_t packet_type; // 3

    char profileName[16];
} __attribute__((packed));

struct PidParamPacket {
    uint8_t packet_type; // 4
    uint8_t pid_id;
    float sp;
    float p, i, d;
    float po;
} __attribute__((packed));

static void packSensorPacket(SensorPacket &pkt) {
    pkt.packet_type = 1;
    pkt.t1 = SensorSampler::getTemperature();
    pkt.t2 = SensorSampler::getTemperature2();
    pkt.t3 = SensorSampler::getEstimatedGroupheadTemperature();
    pkt.p1 = SensorSampler::getPressure();
    pkt.flow = SensorSampler::getFlowRate();
    pkt.vol = SensorSampler::getTotalFlowVolume();
    // pkt.setpoint = BrewControl::getCurrentSetpoint();
    pkt.is_flowing = SensorSampler::isFlowing() ? 1 : 0;

    auto metrics = BrewControl::getMetrics();
    pkt.deviation   = metrics.deviation;
    pkt.effort      = metrics.effort;
    pkt.conductance = metrics.conductance;
    pkt.setpoint    = metrics.pressure_target;
    pkt.dP          = metrics.dP;

    pkt.pid_i  = HeatControl::pid_i.last();
    pkt.pid_d1 = HeatControl::pid_d.last();
    pkt.pid_d2 = HeatControl::pid_d2.last();
    pkt.t_sp   = HeatControl::getSetpoint();

    pkt.b_pwr = IO::getHeatPower() * CONFIG_BOILER_FULL_POWER_WATTS;
    pkt.b_on  = IO::isHeaterOn() ? 1 : 0;

    pkt.state = (uint8_t)State::getState();
    pkt.pull  = IO::isLeverPulled() ? 1 : 0;
    pkt.brew  = IO::isBrewing() ? 1 : 0;

    pkt.mem = esp_get_free_heap_size();
}

static void packBrewStatsPacket(BrewStatsPacket &pkt) {
    pkt.packet_type = 2;

    auto& stats = State::brewStats;
    // pkt.start_brew_time = 
}

static void packMetadataPacket(MetadataPacket &pkt) {
    pkt.packet_type = 3;
}

static void packPidParamPacket(PidParamPacket &pkt) {
    pkt.packet_type = 4;

    // TODO: Support one of 2 PID engines
}



struct ConnState {
    const uint8_t *data;
    size_t len;
    size_t off;
};

static void mongoose_ev(struct mg_connection *c, int ev, void *ev_data) {

    if (ev == MG_EV_WS_MSG) {
        struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;
        const char *msg = (const char *) wm->data.buf;
        size_t len = wm->data.len;

        // Simple text commands:
        // "get"  -> send latest SensorPacket (binary)
        // "meta" -> send MetadataPacket (binary)
        // "pidN" -> send PidParamPacket for id N (binary)
        // "brew" -> send BrewStatsPacket (binary)
        if (len >= 3 && strncmp(msg, "get", 3) == 0) {
            SensorPacket pkt;
            packSensorPacket(pkt);
            mg_ws_send(c, &pkt, sizeof(pkt), WEBSOCKET_OP_BINARY);
            return;
        }

        if (len >= 4 && strncmp(msg, "meta", 4) == 0) {
            MetadataPacket mp;
            packMetadataPacket(mp);
            mg_ws_send(c, &mp, sizeof(mp), WEBSOCKET_OP_BINARY);
            return;
        }

        if (len >= 3 && strncmp(msg, "pid", 3) == 0) {
            uint8_t id = 0;
            if (len > 3 && msg[3] >= '0' && msg[3] <= '9') id = (uint8_t)(msg[3] - '0');
            PidParamPacket pp;
            // fill id if pack function supports it; otherwise caller can ignore pid_id field
            pp.pid_id = id;
            packPidParamPacket(pp);
            mg_ws_send(c, &pp, sizeof(pp), WEBSOCKET_OP_BINARY);
            return;
        }

        if (len >= 4 && strncmp(msg, "brew", 4) == 0) {
            BrewStatsPacket bp;
            packBrewStatsPacket(bp);
            mg_ws_send(c, &bp, sizeof(bp), WEBSOCKET_OP_BINARY);
            return;
        }

        // unknown text frame -> ignore / echo
        // mg_ws_send(c, msg, len, WEBSOCKET_OP_TEXT);
        return;
    }

    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message *hm = (struct mg_http_message *) ev_data;
        Debug.printf("GET: %.*s\n", (int)hm->uri.len, hm->uri.buf);

        // WebSocket upgrade on /ws
        if (mg_strcmp(hm->uri, MG_C_STR("/ws")) == 0) {
            Debug.println("WEBSOCKET OPENED");
            mg_ws_upgrade(c, hm, NULL);
            return;
        }

        if (mg_strcmp(hm->uri, MG_C_STR("/")) == 0) {
            const uint8_t *body = data::index_html_bytes;
            size_t body_len = (size_t)data::index_html_size;

            // create connection state and store in c->fn_data
            ConnState *s = (ConnState*)mg_calloc(1, sizeof(ConnState));
            if (!s) {
                mg_http_reply(c, 500, "Content-Type: text/plain\r\n", "Out of memory");
                return;
            }
            s->data = body;
            s->len  = body_len;
            s->off  = 0;
            c->fn_data = s;

            // send headers with chunked transfer and Connection: close
            mg_http_reply(c, 200,
                          "Content-Type: text/html\r\n"
                          "Cache-Control: no-cache\r\n"
                          "Transfer-Encoding: chunked\r\n"
                          "Connection: close\r\n",
                          "");

            // send first chunk (keep chunks reasonably small)
            size_t first = (s->len > CHUNK_SIZE) ? CHUNK_SIZE : s->len;
            if (first > 0) {
                mg_http_write_chunk(c, (const char*)(s->data + s->off), first);
                s->off += first;
            }

            // if finished, send terminating empty chunk and free state
            if (s->off >= s->len) {
                mg_http_write_chunk(c, "", 0);
                mg_free(s);
                c->fn_data = nullptr;
            }
            return;
        }

        mg_http_reply(c, 404, "Content-Type: text/plain\r\n", "Not found");
        return;
    }

    if (ev == MG_EV_WRITE) {
        ConnState *s = (ConnState*)c->fn_data;
        if (s) {
            size_t rem = s->len - s->off;
            if (rem > 0) {
                size_t chunk = rem > CHUNK_SIZE ? CHUNK_SIZE : rem;
                mg_http_write_chunk(c, (const char*)(s->data + s->off), chunk);
                s->off += chunk;
            }
            if (s->off >= s->len) {
                mg_http_write_chunk(c, "", 0);
                mg_free(s);
                c->fn_data = nullptr;
            }
        }
        return;
    }

    if (ev == MG_EV_CLOSE) {
        ConnState *s = (ConnState*)c->fn_data;
        if (s) {
            mg_free(s);
            c->fn_data = nullptr;
        }
        return;
    }
}

static void mongoose_task(void *arg) {
    (void)arg;
    s_mg_running = true;
    while (s_mg_running) {
        mg_mgr_poll(&s_mgr, 100);
    }
    mg_mgr_free(&s_mgr);
    s_listener = nullptr;
    s_mg_task = nullptr;
    vTaskDelete(NULL);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    Debug.println("Stopping webserver");

    if (s_mg_task) {
        s_mg_running = false;
        // wait for task to exit (short timeout loop)
        for (int i = 0; i < 50 && s_mg_task; ++i) 
            vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    if (s_mg_task) {
        Debug.println("Mongoose already running");
        return;
    }

    mg_log_set(MG_LL_ERROR);

    // Reset the local heap
    heap.reset();

    mg_mgr_init(&s_mgr);
    s_listener = mg_http_listen(&s_mgr, "http://0.0.0.0:80", mongoose_ev, NULL);
    if (!s_listener) {
        Debug.println("Mongoose: failed to bind");
        mg_mgr_free(&s_mgr);
        return;
    }

    xTaskCreate(mongoose_task, "mongoose", 3*1024, NULL, 5, &s_mg_task);

    Debug.println("Webserver started");
}




void WebSrv::setup() {
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, NULL));
}

void WebSrv::stop() {
    s_mg_running = false;
}

void WebSrv::process() {

}
