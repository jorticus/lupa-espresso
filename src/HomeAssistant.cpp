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
//#include "version.h"

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

void HomeAssistant::process() {
    static unsigned long t_last = 0;
    static unsigned long t_last_connect = 0;
    static State::MachineState last_ui_state = State::MachineState::Init;

    if (!isInitialized)
        return;

    if (!client.connected()) {
        // Throttle reconnection attempts
        if ((millis() - t_last_connect) > 1000) {
            Debug.printf("Connecting to MQTT @ %s:%d\n", secrets::mqtt_server, secrets::mqtt_port);

            client.setSocketTimeout(2); // Must be less than the watchdog timer
            bool connected = HAComponentManager::connectClientWithAvailability(client, 
                secrets::device_name, 
                secrets::mqtt_username, 
                secrets::mqtt_password);

            if (connected) {
                Debug.println("MQTT connected, publishing config...");
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
