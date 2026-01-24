
#include "hacomponent.h"
#include <math.h>
#include <ArduinoJson.h>
#include <string>

extern Stream& Debug;

template<Component c> const char* HACompBase<c>::m_component                        = nullptr;
template<>            const char* HACompBase<Component::Sensor>::m_component        = "sensor";
template<>            const char* HACompBase<Component::BinarySensor>::m_component  = "binary_sensor";
template<>            const char* HACompBase<Component::Switch>::m_component        = "switch";
template<>            const char* HACompBase<Component::Select>::m_component        = "select";
template<>            const char* HACompBase<Component::Number>::m_component        = "number";

template<SensorClass c> const char* SensorClassString<c>::name = nullptr;
template<>              const char* SensorClassString<SensorClass::Undefined>::name = nullptr;
template<>              const char* SensorClassString<SensorClass::Temperature>::name = "temperature";
template<>              const char* SensorClassString<SensorClass::Humidity>::name = "humidity";
template<>              const char* SensorClassString<SensorClass::Battery>::name = "battery";
template<>              const char* SensorClassString<SensorClass::Illuminance>::name = "illuminance";
template<>              const char* SensorClassString<SensorClass::Pressure>::name = "pressure";
template<>              const char* SensorClassString<SensorClass::Power>::name = "power";
template<>              const char* SensorClassString<SensorClass::Energy>::name = "energy";
template<>              const char* SensorClassString<SensorClass::Voltage>::name = "voltage";

// template<BinarySensorClass c> const char* BinarySensorClass<c>::name = nullptr;
// template<>                    const char* BinarySensorClass<BinarySensorClass::connectivity>::name = "connectivity";
// // TODO...

// Static instantiations
std::vector<HACompItem*>                        HACompItem::m_components;
std::unordered_map<std::string, HACompItem*>    HACompItem::m_subscriptionMap;
static HAComponent<Component::BinarySensor>*    s_component = nullptr;

// Warning: HomeAssistant is case sensitive! These are the default state values...
const char*                                     HAComponent<Component::Switch>::ON = "ON";
const char*                                     HAComponent<Component::Switch>::OFF = "OFF";

HAAvailabilityComponent*                        HAAvailabilityComponent::inst = nullptr;
const char*                                     HAAvailabilityComponent::ONLINE = "online";
const char*                                     HAAvailabilityComponent::OFFLINE = "offline";

// ---------------------------------------------------------------------------------------
// HA Component Manager
// ---------------------------------------------------------------------------------------
#ifndef REGION_HA_COMPONENT_MANAGER

void HAComponentManager::onMessageReceived(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';

    String topic_s(topic);
    String payload_s((char*)payload);

    // Look up the component to invoke via the subscription map.
    // Each topic that is subscribed will add the owning component to this map.
    std::string topic_str(topic);
    auto item = m_subscriptionMap.find(topic_str);
    if (item != m_subscriptionMap.end()) {
        HACompItem* component = item->second;
        component->handleMqttUpdate(topic_s, payload_s);
    }
    else {
        Debug.print("Unhandled MQTT Topic: "); Debug.println(topic_s);
    }
}

bool HAComponentManager::connectClientWithAvailability(PubSubClient& client, const char* id, const char* user, const char* password) {
    HAAvailabilityComponent* avail = HAAvailabilityComponent::inst;
    if (avail != nullptr) {
        String will_topic 		= avail->getWillTopic();
        const char* will_msg 	= HAAvailabilityComponent::OFFLINE;
        uint8_t will_qos 		= 0;
        bool will_retain 		= true;

        bool connected = client.connect(
            id, user, password,
            will_topic.c_str(), will_qos, will_retain, will_msg
        );

        if (connected) {
            // Report alive status to availability topic
            avail->connect();
        }
        return connected;
    }
    else {
        return client.connect(
            id, user, password
        );
    }
    return false;
}

static void getDeviceInfo(JsonObject& json, ComponentContext& context) {
    auto& deviceInfo = json.createNestedObject("device");

    deviceInfo["identifiers"]   = context.mac_address;
    deviceInfo["name"]          = context.friendly_name;
    deviceInfo["sw_version"]    = context.fw_version;
    deviceInfo["model"]         = context.model;
    deviceInfo["manufacturer"]  = context.manufacturer;

    // This tells HA if the component is available (connected) or not.
    if (HAAvailabilityComponent::inst != nullptr) {
        // NOTE: This doesn't seem to have any effect as long as you make sure the availability
        // sensor is published with the same device information as other sensors...
        //json["availability_topic"] = HAAvailabilityComponent::inst->getWillTopic();
    }
}
#endif

// ---------------------------------------------------------------------------------------
// Base Template
// ---------------------------------------------------------------------------------------
#ifndef REGION_BASE_TEMPLATE

template<Component c>
void HACompBase<c>::getConfigInfo(JsonObject& json)
{
}

template<Component c>
void HACompBase<c>::handleMqttUpdate(String topic, String value)
{
}

template<Component c>
void HACompBase<c>::subscribe(String topic)
{
    Debug.print("Subscribe: "); Debug.println(topic.c_str());

    // Register this component as an owner for this topic,
    // so the update can get routed to the correct component.
    std::string s = topic.c_str();
    m_subscriptionMap[s] = this;

    // Subscribe to this MQTT topic so we get updates
    context.client.subscribe(topic.c_str());
}

template<Component c>
void HACompBase<c>::initialize()
{
    char state_topic[TOPIC_BUFFER_SIZE];
    snprintf(state_topic, sizeof(state_topic), 
        "%s/%s/%s/state\0", 
        context.device_name, m_component, m_id);
    m_state_topic = String(state_topic);
}

// Generic publish implementation used for all component types
template<Component c>
void HACompBase<c>::publishConfig(bool present)
{
    // Generic implementation
    char topic[TOPIC_BUFFER_SIZE];

    snprintf(topic, sizeof(topic), 
        "homeassistant/%s/%s/%s/config\0", 
        m_component, context.device_name, m_id);

    if (present) {
        StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();

        //Led::SetBuiltin(true);

        json["name"]    = m_name;
        json["stat_t"]  = m_state_topic;

        // For a complete list of JSON parameters you can set, see:
        // https://www.home-assistant.io/docs/mqtt/discovery/
        getConfigInfo(json);

        // Add unique ID for component
        char uid[TOPIC_BUFFER_SIZE];
        snprintf(uid, sizeof(uid),
            "%s_%s\0",
            context.device_name, m_id);
        
        json["unique_id"] = uid;
        json["object_id"] = uid; // Used for generation of entity_id

        //json["entity_category"] = "config"/"diagnostic";

        if (m_icon != nullptr) {
            // Optional icon for HA UI
            // eg. "mdi:plug"
            json["icon"] = m_icon;
        }

        // Add device information
        getDeviceInfo(json, context);

        String payload;
        json.printTo(payload);

        Debug.print("publish: ");
        Debug.print(topic);
        if (!present) {
            Debug.print(" (not present)");
        } else {
            //Debug.print(" -> ");
            //Debug.print(payload.c_str());
        }
        Debug.println();

        if (!context.client.publish(topic, payload.c_str(), true)) {
            Debug.println("ERROR PUBLISHING TOPIC");
        }
    } 
    else {
        // If not present, we should unpublish the topic
        Debug.print("unpublish: ");
        Debug.println(topic);

        // IMPORTANT: Use 4-arg overload. The 2 & 3-arg overloads try to call strlen() on payload
        context.client.publish(topic, nullptr, 0, true);

        // Also unpublish the parent node
        snprintf(topic, sizeof(topic), 
            "homeassistant/%s/%s/%s\0", 
            m_component, context.device_name, m_id);
        context.client.publish(topic, nullptr, 0, true);

        // And finally clear the current state
        // (so it's clear the last retained sensor value is no longer valid)
        clearState();
    }
}

// Generic publish implementation for sending sensor readings
template<Component c>
void HACompBase<c>::publishState(const char* value, bool retain)
{
    context.client.publish(m_state_topic.c_str(), value, retain);
}

template<Component c>
void HACompBase<c>::clearState()
{
    // Un-publish the state topic
    // IMPORTANT: Use 4-arg overload. The 2 & 3-arg overloads try to call strlen() on payload
    context.client.publish(m_state_topic.c_str(), nullptr, 0, true);
}

#endif

// ---------------------------------------------------------------------------------------
// Sensor Entity
// ---------------------------------------------------------------------------------------
#ifndef REGION_SENSOR_ENTITY

// Topic specialization for sensor components
void HAComponent<Component::Sensor>::getConfigInfo(JsonObject& json)
{
    // https://www.home-assistant.io/components/sensor.mqtt/

    // Sensor component topic
    const char* units = "";
    const char* device_class = nullptr;

    // Update sensor state even if value hasn't changed.
    // This ensures Graphite/Grafana get regularly spaced samples!
    json["frc_upd"] = true; // "force_update"

    switch (m_sensor_class) {
        case SensorClass::Temperature:
            device_class = SensorClassString<SensorClass::Temperature>::name;
            units = "°C";
            break;
        case SensorClass::Humidity:
            device_class = SensorClassString<SensorClass::Humidity>::name;
            units = "%";
            break;
        case SensorClass::Battery:
            device_class = SensorClassString<SensorClass::Battery>::name;
            units = "%";
            break;
        case SensorClass::Illuminance:
            device_class = SensorClassString<SensorClass::Illuminance>::name;
            units = "lx";
            break;
        case SensorClass::Pressure:
            device_class = SensorClassString<SensorClass::Pressure>::name;
            // units = "mbar"; // "hPa"
            units = "Bar";
            break;
        case SensorClass::Power:
            device_class = SensorClassString<SensorClass::Power>::name;
            units = "W";
            break;
        case SensorClass::Energy:
            device_class = SensorClassString<SensorClass::Energy>::name;
            units = "Wh";
            break;
        case SensorClass::Voltage:
            device_class = SensorClassString<SensorClass::Voltage>::name;
            units = "V";
            break;
        case SensorClass::Dust:
            device_class = nullptr;
            units = "ug/m³";
            break;
        case SensorClass::PPM:
            device_class = nullptr;
            units = "ppm";
            break;
        case SensorClass::PPB:
            device_class = nullptr;
            units = "ppb";
            break;
    }

    json["unit_of_meas"] = units; // "unit_of_measurement"
    if (device_class != nullptr) {
        json["dev_cla"] = device_class;
    }
}


// Sensor reading publish implementation
void HAComponent<Component::Sensor>::update(float value)
{
    // Ensure value is not NaN or Infinity
    if (!std::isfinite(value)) {
        return;
    }

    // Accumulate value
    m_samples++;
    m_sum += value;

    long ts = millis();
    if (ts - m_last_ts > m_sample_interval) {
        m_last_ts = ts;

        // Compute mean over the sample window
        float avg_value = (m_samples > 0) ? (m_sum / (float)m_samples) : m_sum;
        m_samples = 0;
        m_sum = 0.f;

        //Debug.print(m_id); Debug.print(": "); Debug.println(avg_value);

        // Only publish if the value is significant
        if (std::isfinite(avg_value) && ((m_hysteresis == 0.0f) || (avg_value <= m_last_value - m_hysteresis || avg_value >= m_last_value + m_hysteresis))) {
            m_last_value = avg_value;

            String value_s(avg_value);
            HACompBase<Component::Sensor>::publishState(value_s.c_str());
        }
    }
}

float HAComponent<Component::Sensor>::getCurrent()
{
    return m_last_value;
}


// Topic specialization for sensor components
void HAComponent<Component::BinarySensor>::getConfigInfo(JsonObject& json)
{
    // https://www.home-assistant.io/components/binary_sensor.mqtt/

    // Sensor component topic
    const char* device_class = nullptr;

    switch (m_sensor_class) {
        // TODO...
        // default:
        //     Debug.println("WARNING: Device class unsupported!");
    }

    if (device_class != nullptr) {
        json["dev_cla"] = device_class;
    }
}

void HAComponent<Component::BinarySensor>::reportState(bool state)
{
    publishState(state ? "ON" : "OFF");
}

#endif

// ---------------------------------------------------------------------------------------
// Switch Entity
// ---------------------------------------------------------------------------------------
#ifndef REGION_SWITCH_ENTITY

void HAComponent<Component::Switch>::initialize()
{
    HACompBase<Component::Switch>::initialize();

    char cmd_topic[TOPIC_BUFFER_SIZE];
    snprintf(cmd_topic, sizeof(cmd_topic), 
        "%s/%s/%s/ctrl\0", 
        context.device_name, m_component, m_id);
    m_cmd_topic = String(cmd_topic);
}

// Topic specialization for switch components
void HAComponent<Component::Switch>::getConfigInfo(JsonObject& json)
{
    // https://www.home-assistant.io/components/switch.mqtt/

    json["cmd_t"]   = m_cmd_topic; // "command_topic"

    subscribe(m_cmd_topic);

    reportState();
}

void HAComponent<Component::Switch>::setState(bool state)
{
    m_state = state;
    m_callback(state);

    reportState();
}

void HAComponent<Component::Switch>::reportState()
{
    context.client.publish(m_state_topic.c_str(), (m_state ? ON : OFF), true);
}

void HAComponent<Component::Switch>::handleMqttUpdate(String topic, String value)
{
    if (topic == m_cmd_topic) {
        if (value.equalsIgnoreCase(ON)) {
            setState(true);
        }
        else if (value.equalsIgnoreCase(OFF)) {
            setState(false);
        }
        else {
            Debug.print("Invalid payload received for switch: ");
            Debug.println(value);
        }
    }
}

#endif

// ---------------------------------------------------------------------------------------
// Select Entity
// ---------------------------------------------------------------------------------------
#ifndef REGION_SELECT_ENTITY

void HAComponent<Component::Select>::initialize()
{
    HACompBase<Component::Select>::initialize();

    char cmd_topic[TOPIC_BUFFER_SIZE];
    snprintf(cmd_topic, sizeof(cmd_topic), 
        "%s/%s/%s/ctrl\0", 
        context.device_name, m_component, m_id);
    m_cmd_topic = String(cmd_topic);
}

// Topic specialization for switch components
void HAComponent<Component::Select>::getConfigInfo(JsonObject& json)
{
    // https://www.home-assistant.io/components/select.mqtt/

    json["cmd_t"]   = m_cmd_topic; // "command_topic"
    // json["cmd_t"]   = m_state_topic; // "command_topic"

    // Tell HA to retain the value it publishes,
    // so we can pick up the value again on reboot.
    json["retain"]  = "true";

    auto& options = json.createNestedArray("options");
    for (auto& o : m_options) {
        String s { o.c_str() };
        options.add(s);
    }

    subscribe(m_cmd_topic);

    reportState();
}

void HAComponent<Component::Select>::setState(std::string state)
{
    for (auto& opt : m_options) {
        if (opt == state) {
            m_state = state;
            m_callback(state);
            break;
        }
    }

    reportState();
}

void HAComponent<Component::Select>::reportState()
{
    context.client.publish(m_state_topic.c_str(), m_state.c_str(), true);
}

void HAComponent<Component::Select>::handleMqttUpdate(String topic, String value)
{
    if (m_cmd_topic == topic) {
        std::string s { value.c_str() };
        setState(s);
    }
}

#endif

// ---------------------------------------------------------------------------------------
// Number Entity
// ---------------------------------------------------------------------------------------
#ifndef REGION_NUMBER_ENTITY

void HAComponent<Component::Number>::initialize()
{
    if (m_cmd_topic.isEmpty()) {
        HACompBase<Component::Number>::initialize();

        char cmd_topic[TOPIC_BUFFER_SIZE];
        snprintf(cmd_topic, sizeof(cmd_topic), 
            "%s/%s/%s/ctrl\0", 
            context.device_name, m_component, m_id);
        m_cmd_topic = String(cmd_topic);
    }
}

// Topic specialization for switch components
void HAComponent<Component::Number>::getConfigInfo(JsonObject& json)
{
    // https://www.home-assistant.io/components/number.mqtt/

    json["cmd_t"]   = m_cmd_topic; // "command_topic"

    // Tell HA to retain the value it publishes,
    // so we can pick up the value again on reboot.
    json["retain"]  = "true";

    json["min"] = String(m_range.min);
    json["max"] = String(m_range.max);
    json["step"] =  String(m_range.step);
    json["mode"] = "slider"; // Can be "box" or "slider"
    if (m_range.units != nullptr) {
        json["unit_of_meas"] = String(m_range.units);
    }

    subscribe(m_cmd_topic);

    reportState();
}


void HAComponent<Component::Number>::setState(float state)
{
    if (state < m_range.min) state = m_range.min;
    else if (state > m_range.max) state = m_range.max;

    m_state = state;
    m_callback(state);

    reportState();
}

void HAComponent<Component::Number>::reportState()
{
    String s_state((float)m_state);
    context.client.publish(m_state_topic.c_str(), s_state.c_str(), true);
}

void HAComponent<Component::Number>::handleMqttUpdate(String topic, String value)
{
    if (m_cmd_topic == topic) {
        float f = std::strtof(value.c_str(), nullptr);
        Debug.printf("Set %s = %.1f\n", m_name, f);
        setState(f);
    }
}
#endif

// ---------------------------------------------------------------------------------------
// Availability Entity
// ---------------------------------------------------------------------------------------
#ifndef REGION_AVAILABILITY_ENTITY

HAAvailabilityComponent::HAAvailabilityComponent(ComponentContext& context)
    : HACompBase(context, "status", "Status")
{
    HAAvailabilityComponent::inst = this;
}

void HAAvailabilityComponent::initialize()
{
    char state_topic[TOPIC_BUFFER_SIZE];
    snprintf(state_topic, sizeof(state_topic), 
        "%s/%s\0", 
        context.device_name, m_id);
    m_state_topic = String(state_topic);
}

void HAAvailabilityComponent::getConfigInfo(JsonObject& json)
{
    // TODO: values should be "online" and "offline"
    json["payload_on"]  = ONLINE;
    json["payload_off"] = OFFLINE;

    json["dev_cla"] = "connectivity";
}

String HAAvailabilityComponent::getWillTopic()
{
    return m_state_topic;
}

void HAAvailabilityComponent::connect()
{
    publishState(ONLINE);
}

#endif

// ---------------------------------------------------------------------------------------
// Explicit Template Init
// ---------------------------------------------------------------------------------------

// Explicit template instantiations. Required to make the CPP linker happy
template class HACompBase<Component::Sensor>;
template class HAComponent<Component::Sensor>;
template class HACompBase<Component::Switch>;
template class HAComponent<Component::Switch>;
template class HACompBase<Component::BinarySensor>;
template class HAComponent<Component::BinarySensor>;
template class HACompBase<Component::Select>;
template class HAComponent<Component::Select>;
template class HACompBase<Component::Number>;
template class HAComponent<Component::Number>;
