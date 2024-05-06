#include "HomeAssistant.h"
#include <hacomponent.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>
#include "secrets.h"
#include "UI.h"
#include "SensorSampler.h"
#include "StateMachine.h"
#include "IO.h"
#include "config.h"
#include "PressureControl.h"
#include "MqttParamManager.h"
#include "Debug.h"
#include <lwip/dns.h>
//#include "version.h"

/// @brief MQTT connection timeout.
/// Must be less than the watchdog timer!!
const int NET_CONNECT_TIMEOUT_SEC = 2;

const int NET_RECONNECT_INTERVAL_MS = 3000;

// version.py
extern const char* GEN_BUILD_VERSION;

extern Stream& Debug;

WiFiClient net;
//WiFiClientSecure net;

PubSubClient client(net);
ComponentContext context(client);

const unsigned long sensor_sample_interval_ms = 5000;
const unsigned long slow_sensor_sample_interval = 60*1000; // 1 min
const unsigned long power_sample_interval_ms = 10*1000;

static bool isInitialized = false;

HAAvailabilityComponent availability(context);

HAComponent<Component::Switch> switch_power(context, 
    "power", 
    "Power", 
    State::setPowerControl,
    "mdi:coffee"
);

HAComponent<Component::Sensor> sensor_temperature(context, 
    "boiler_t", 
    "Boiler Temperature", 
    sensor_sample_interval_ms, 
    0.0f, 
    SensorClass::Temperature
);

HAComponent<Component::Sensor> sensor_pressure(context, 
    "pressure", 
    "Grouphead Pressure", 
    sensor_sample_interval_ms, 
    0.0f, 
    SensorClass::Pressure
);

HAComponent<Component::BinarySensor> sensor_isbrewing(context, 
    "is_brewing", 
    "Is Brewing", 
    BinarySensorClass::Undefined, 
    "mdi:coffee-to-go"
);

HAComponent<Component::Sensor> sensor_power(context,
    "boiler_w",
    "Boiler Watts",
    power_sample_interval_ms,
    0.0f,
    SensorClass::Power
);

void onMessageReceived(char* topic, byte* payload, unsigned int length) {
    String topic_s(topic);
    String payload_s((const char*)payload, length);

    Debug.print("MQTT "); Debug.println(topic_s);

    if (MqttParam::handleUpdate(topic_s, payload_s)) {
        return;
    }

    HAComponentManager::onMessageReceived(topic, payload, length);
}

void HomeAssistant::init() {
    static String mac = WiFi.macAddress();
    context.mac_address = mac.c_str();
    context.device_name = secrets::device_name;
    context.fw_version = GEN_BUILD_VERSION;
    
    context.manufacturer = "ViscTronics";
    context.friendly_name = "LUPA Espresso";
    context.model = "VBM Domobar Junior";

    HAComponentManager::initializeAll();

    // For SSL
    //net.setCACert(secrets::ca_root_cert);
    //net.setInsecure();

    client.setBufferSize(HA_MQTT_MAX_PACKET_SIZE);
    client.setServer(secrets::mqtt_server, secrets::mqtt_port);

    client.setCallback(onMessageReceived);

    isInitialized = true;
}

void reportState() {
    if (!isInitialized)
        return;

    // State changed, report it...
    auto state = State::getState();
    
    // Don't report state on transition to sleep,
    // as that causes a bounce back call to State::setPowerControl(true),
    // which in turn bumps the state back to Preheat.
    if (state != State::MachineState::Sleep) {
        switch_power.setState((state != State::MachineState::Off));
    }

    sensor_isbrewing.reportState((state == State::MachineState::Brewing));
}

void onConnect() {
    // Publish all components to HomeAssistant, and subscribe to any required topics
    HAComponentManager::publishConfigAll();

    // Publish parameters to MQTT, and subscribe for change notifications
    MqttParam::publish();
}

#include <esp_task_wdt.h>
const uint32_t WDT_TIMEOUT_SEC = 4;

static EventGroupHandle_t _arduino_event_group = NULL;

static void wifi_dns_found_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
    if (ipaddr) {
        (*reinterpret_cast<IPAddress*>(callback_arg)) = ipaddr->u_addr.ip4.addr;
    }
    xEventGroupSetBits(_arduino_event_group, WIFI_DNS_DONE_BIT);
}

/// @brief Resolve a hostname by querying DNS, with timeout
/// @details Reimplementation of WiFiGeneric::hostByName that lets you specify a timeout
/// @param hostname Hostname to query
/// @param result IP address of the hostname
/// @param timeout_ms Timeout, in milliseconds
/// @return True if DNS query was successful
static bool hostByName(const char* hostname, IPAddress& result, uint32_t timeout_ms = 1000) {
    if (!result.fromString(hostname)) {
        ip_addr_t addr;
        result = static_cast<uint32_t>(0);

        if (!_arduino_event_group) {
            _arduino_event_group = xEventGroupCreate();
            xEventGroupSetBits(_arduino_event_group, WIFI_DNS_IDLE_BIT);
        }

        xEventGroupWaitBits(
            _arduino_event_group, 
            WIFI_DNS_IDLE_BIT,
            pdFALSE,
            pdTRUE,
            timeout_ms / portTICK_PERIOD_MS);

        xEventGroupClearBits(_arduino_event_group, WIFI_DNS_IDLE_BIT | WIFI_DNS_DONE_BIT);

        err_t err = dns_gethostbyname(hostname, &addr, &wifi_dns_found_callback, &result);
        if (err == ERR_OK && addr.u_addr.ip4.addr) {
            result = addr.u_addr.ip4.addr;
        } else if (err == ERR_INPROGRESS) {
            xEventGroupWaitBits(
                _arduino_event_group, 
                WIFI_DNS_DONE_BIT,
                pdFALSE,
                pdTRUE,
                timeout_ms / portTICK_PERIOD_MS );

            xEventGroupClearBits(_arduino_event_group, WIFI_DNS_DONE_BIT);
            xEventGroupSetBits(_arduino_event_group, WIFI_DNS_IDLE_BIT);

            if ((uint32_t)result == 0) {
                Debug.println("ERROR: DNS lookup timeout");
            }
        }
    }

    return (uint32_t)result != 0;
}

void HomeAssistant::process() {
    static unsigned long t_last = 0;
    static unsigned long t_last_connect = 0;
    static State::MachineState last_ui_state = State::MachineState::Init;

    if (!isInitialized)
        return;

    // TODO: We should really just stick this into a separate FreeRTOS task
    // to ensure any blocks don't hold up the main loop.
    if (!client.connected()) {
        // Throttle reconnection attempts
        if ((millis() - t_last_connect) > NET_RECONNECT_INTERVAL_MS) {
            Debug.printf("Connecting to MQTT @ %s:%d\n", secrets::mqtt_server, secrets::mqtt_port);

            // Perform DNS lookup, returning immediately if address was not found.
            // This prevents blocking of the main loop if the server is not available.
            // If the server is available, the next iteration of the loop should pick up the cached value immediately.
            IPAddress host_addr;
            if (hostByName(secrets::mqtt_server, host_addr, 0)) {
                esp_task_wdt_reset();

                // Connect WiFi client with timeout
                // NOTE: This may halt the main loop for up to the timeout.
                // For best results it would be beneficial to figure out how to do this asynchronously...
                net.setTimeout(NET_CONNECT_TIMEOUT_SEC);
                int r = net.connect(host_addr, secrets::mqtt_port);
                if (r && net.connected()) {
                    esp_task_wdt_reset();
                    
                    // The socket is now connected, try to establish an MQTT connection.
                    // This may also introduce a delay for up to the timeout, but should only occur once on startup.
                    // Note: The timeout below is for the MQTT connection itself, not the underlying socket. 
                    client.setSocketTimeout(NET_CONNECT_TIMEOUT_SEC);
                    bool connected = HAComponentManager::connectClientWithAvailability(client, 
                        secrets::device_name, 
                        secrets::mqtt_username, 
                        secrets::mqtt_password);

                    esp_task_wdt_reset();

                    if (connected) {
                        Debug.println("MQTT connected, publishing config...");
                        onConnect();
                        reportState();
                    }
                }
            }
            
            t_last_connect = millis();
        }
    }
    else {
        auto state = State::getState();
        if (state != last_ui_state) {
            last_ui_state = state;

            reportState();
        }

        // Use fast report rate when on, slow report rate when off
        auto interval = (state != State::MachineState::Off) ?
            sensor_sample_interval_ms : 
            slow_sensor_sample_interval;

        if ((millis() - t_last) >= interval) {
            t_last = millis();

            // Report current boiler temperature to HA
            if (SensorSampler::isTemperatureValid()) {
                auto t = SensorSampler::getTemperature();
                sensor_temperature.update(t);
            }

            // Report current grouphead pressure to HA
            if (SensorSampler::isPressureValid()) {
                auto p = SensorSampler::getPressure();
                sensor_pressure.update(p);
            }

            // Estimate boiler power and report to HA
            float estimated_power = IO::getHeatPower() * CONFIG_BOILER_FULL_POWER_WATTS;
            sensor_power.update(estimated_power);
        }
    }

    client.loop();
}

void HomeAssistant::publishData(const char* topic, const char* payload) {
    client.publish(topic, payload);
}
