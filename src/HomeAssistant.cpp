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
//#include "version.h"

// version.py
extern const char* GEN_BUILD_VERSION;

extern Stream& Debug;

WiFiClient net;
//WiFiClientSecure net;

PubSubClient client(net);
ComponentContext context(client);

const unsigned long sensor_sample_interval_ms = 1000;
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

    client.setCallback(HAComponentManager::onMessageReceived);

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
}

void HomeAssistant::process() {
    static unsigned long t_last = 0;
    static unsigned long t_last_connect = 0;
    static State::MachineState last_ui_state = State::MachineState::Init;

    if (!isInitialized)
        return;

    if (!client.connected()) {
        // Throttle reconnection attempts
        if ((millis() - t_last_connect) > 1000) {
            Serial.println("Connecting MQTT");
            bool connected = HAComponentManager::connectClientWithAvailability(client, 
                secrets::device_name, 
                secrets::mqtt_username, 
                secrets::mqtt_password);

            if (connected) {
                Serial.println("MQTT connected, publishing config...");
                onConnect();
                reportState();
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
