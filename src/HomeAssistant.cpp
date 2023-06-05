#include "HomeAssistant.h"
#include <hacomponent.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>
#include "secrets.h"
#include "UI.h"
#include "SensorSampler.h"
//#include "version.h"

// version.py
extern const char* GEN_BUILD_VERSION;

extern Stream& Debug;

WiFiClient net;
//WiFiClientSecure net;

PubSubClient client(net);
ComponentContext context(client);

const unsigned long sensor_sample_interval_ms = 1000;
const unsigned long temperature_report_interval_ms = 10000;

HAAvailabilityComponent availability(context);

HAComponent<Component::Switch> switch_power(context, 
    "power", 
    "Power", 
    UI::setPowerControl,
    "mdi:coffee"
);

HAComponent<Component::Sensor> sensor_temperature(context, 
    "boiler_t", 
    "Boiler Temperature", 
    temperature_report_interval_ms, 
    0.0f, 
    SensorClass::Temperature
);

HAComponent<Component::BinarySensor> sensor_isbrewing(context, 
    "is_brewing", 
    "Is Brewing", 
    BinarySensorClass::Undefined, 
    "mdi:coffee-to-go"
);

void HomeAssistant::init() {
    static String mac = WiFi.macAddress();
    context.mac_address = mac.c_str();
    context.device_name = secrets::device_name;
    context.fw_version = GEN_BUILD_VERSION;
    
    context.manufacturer = "ViscTronics";
    context.friendly_name = "LUPA Espresso";
    context.model = "VBM Domobar Junior";

    HAComponentManager::InitializeAll();

    // For SSL
    //net.setCACert(secrets::ca_root_cert);
    //net.setInsecure();

    client.setBufferSize(HA_MQTT_MAX_PACKET_SIZE);
    client.setServer(secrets::mqtt_server, secrets::mqtt_port);

    client.setCallback(HAComponentManager::OnMessageReceived);

}

void reportState() {
    // State changed, report it...
    auto state = UI::getState();
    
    switch_power.SetState((state != UI::UiState::Off));

    sensor_isbrewing.ReportState((state == UI::UiState::Brewing));
}

void onConnect() {
    // Publish all components to HomeAssistant, and subscribe to any required topics
    HAComponentManager::PublishConfigAll();

    // Notify HomeAsisstant that our device is now alive
    availability.Connect();
}

void HomeAssistant::process() {
    static unsigned long t_last = 0;
    static UI::UiState last_ui_state = UI::UiState::Init;

    if (!client.connected()) {
        String will_topic 		= HAAvailabilityComponent::inst->getWillTopic();
        const char* will_msg 	= HAAvailabilityComponent::OFFLINE;
        uint8_t will_qos 		= 0;
        bool will_retain 		= true;

        auto connected = client.connect(
            secrets::device_name, secrets::mqtt_username, secrets::mqtt_password, 
            will_topic.c_str(), will_qos, will_retain, will_msg);

        if (connected) {
            onConnect();
            reportState();
        }
    }
    else {
        auto state = UI::getState();
        if (state != last_ui_state) {
            last_ui_state = state;

            reportState();
        }

        // Only report sensor readings when machine is on
        if (state != UI::UiState::Off) {
            if ((millis() - t_last) >= sensor_sample_interval_ms) {
                t_last = millis();

                if (SensorSampler::isTemperatureValid()) {
                    auto t = SensorSampler::getTemperature();
                    sensor_temperature.Update(t);
                }
            }
        }
    }

    client.loop();
}

void HomeAssistant::reportPowerControlState(bool pwr) {
//    switch_power.SetState(pwr);
}
