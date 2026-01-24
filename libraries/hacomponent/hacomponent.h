#pragma once

#include <Arduino.h>
#include <vector>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <string>
#include <span>
#include <unordered_map>

#define HA_MQTT_MAX_PACKET_SIZE (1024)

#define TOPIC_BUFFER_SIZE (120)
#define JSON_BUFFER_SIZE (HA_MQTT_MAX_PACKET_SIZE)

class ComponentContext {
public:
    PubSubClient& client;
    const char* mac_address;
    const char* device_name;
    const char* friendly_name;
    const char* fw_version;
    const char* model;
    const char* manufacturer;

    ComponentContext(PubSubClient& client)
        : client(client)
    { }
};

enum class Component {
    Undefined,
    Sensor,
    BinarySensor,
    Switch,
    Select,
    Number
};

class InputRange {
public:
    InputRange(float min, float max, float step = 0.1f, const char* units = nullptr)
        : min(min)
        , max(max)
        , step(step)
        , units(units)
    { }

    const float min, max, step;
    const char* units;
};

template<Component c>
struct ComponentString {
public:
    static const char* name;
};

// See https://www.home-assistant.io/components/sensor/ for supported device classes
// https://github.com/home-assistant/home-assistant/blob/70ce9bb7bc15b1a66bb1d21598efd0bb8b102522/homeassistant/components/sensor/__init__.py#L26
enum class SensorClass {
    Undefined,
    Battery,
    Humidity,
    Illuminance,
    Temperature,
    Pressure,
    Energy,
    Power,
    Voltage,

// Custom classes with predefined units (device_class == null)
    Dust,
    PPM,
    PPB
};

template<SensorClass c>
struct SensorClassString {
public:
    static const char* name;
};

// TODO: Need to define strings for these
// https://github.com/home-assistant/home-assistant/blob/70ce9bb7bc15b1a66bb1d21598efd0bb8b102522/homeassistant/components/binary_sensor/__init__.py
enum class BinarySensorClass {
    battery,       // On means low, Off means normal
    cold,          // On means cold, Off means normal
    connectivity,  // On means connected, Off means disconnected
    door,          // On means open, Off means closed
    garage_door,   // On means open, Off means closed
    gas,           // On means gas detected, Off means no gas (clear)
    heat,          // On means hot, Off means normal
    light,         // On means light detected, Off means no light
    lock,          // On means open (unlocked), Off means closed (locked)
    moisture,      // On means wet, Off means dry
    motion,        // On means motion detected, Off means no motion (clear)
    moving,        // On means moving, Off means not moving (stopped)
    occupancy,     // On means occupied, Off means not occupied (clear)
    opening,       // On means open, Off means closed
    plug,          // On means plugged in, Off means unplugged
    power,         // On means power detected, Off means no power
    presence,      // On means home, Off means away
    problem,       // On means problem detected, Off means no problem (OK)
    safety,        // On means unsafe, Off means safe
    smoke,         // On means smoke detected, Off means no smoke (clear)
    sound,         // On means sound detected, Off means no sound (clear)
    vibration,     // On means vibration detected, Off means no vibration
    window,        // On means open, Off means closed
    Undefined
};

// Abstract class that allows us to initialize and publish
// any type of component
class HACompItem
{
    friend class HAComponentManager;

protected:
    static std::vector<HACompItem*> m_components;
    static std::unordered_map<std::string, HACompItem*> m_subscriptionMap;

    virtual void initialize() = 0;
    virtual void publishConfig(bool present) = 0;
    virtual void handleMqttUpdate(String topic, String value) = 0;
};

// Manager class for interacting with all registered components
class HAComponentManager: public HACompItem
{
public:
    /// @brief Initialize all registered copmonents with the provided context.
    /// MQTT connection is not required yet.
    static void initializeAll() {
        for (auto item : HACompItem::m_components) {
            item->initialize();
        }
    }

    /// @brief Publish all registered components to HomeAssistant.
    /// Requires an active MQTT connection.
    /// @param present true to publish, false to unpublish
    static void publishConfigAll(bool present = true) {
        for (auto item : HACompItem::m_components) {
            item->publishConfig(present);
        }
    }

    /// @brief Helper function for establishing MQTT connection with
    //  appropriate will topics
    static bool connectClientWithAvailability(PubSubClient& client, const char* id, const char* user, const char* password);

    /// @brief Callback for receiving MQTT messages
    static void onMessageReceived(char* topic, byte* payload, unsigned int length);
};

// Base class to get around templating quirks. Do not use directly.
template<Component c>
class HACompBase : public HACompItem
{
protected:
    const char*     m_device_class;
    const char*     m_name;
    const char*     m_id;
    const char*     m_icon;
    String          m_state_topic;
    ComponentContext& context;

    static const char* m_component;

    virtual void getConfigInfo(JsonObject& json);
    virtual void handleMqttUpdate(String topic, String value) override;
    void subscribe(String topic);

public:
    HACompBase(ComponentContext& context, const char* id, const char* name)
        : m_id(id), m_name(name), context(context)
    {
        m_components.push_back(this);
    }

    void initialize() override;
    void publishConfig(bool present = true) override;

    void publishState(const char* value, bool retain = true);
    void clearState();
};

// Generic Component
template<Component component>
class HAComponent : public HACompBase<component>
{ 
    using HACompBase<component>::HACompBase; // constructor
};

// Specialization of Component of type Sensor
template<>
class HAComponent<Component::Sensor> : public HACompBase<Component::Sensor>
{
protected:
    SensorClass m_sensor_class;

    // Hysteresis
    float m_hysteresis;
    float m_last_value;

    // Averaging
    float m_sum;
    int m_samples;

    // Sampling
    int m_last_ts;
    int m_sample_interval;

    virtual void getConfigInfo(JsonObject& json);
public:
    HAComponent(ComponentContext& context, const char* id, const char* name, int sample_interval_ms, float hysteresis = 0.0f, SensorClass sclass = SensorClass::Undefined, const char* icon = nullptr) :
        HACompBase(context, id, name),
        m_sample_interval(sample_interval_ms),
        m_sensor_class(sclass),
        m_hysteresis(hysteresis),
        m_sum(0.f),
        m_last_ts(0),
        m_samples(0)
    { 
        m_icon = icon;
    }

    void update(float value);
    float getCurrent();
};

// Specialization of Component of type Switch
template<>
class HAComponent<Component::Switch> : public HACompBase<Component::Switch>
{
    friend class HAComponentManager;
protected:
    bool m_state;
    String m_cmd_topic;
    std::function<void(boolean)> m_callback;

    virtual void getConfigInfo(JsonObject& json);
    void handleMqttUpdate(String topic, String value) override;
public:
    HAComponent(ComponentContext& context, const char* id, const char* name, std::function<void(boolean)> callback, const char* icon = nullptr) :
        HACompBase(context, id, name),
        m_callback(callback),
        m_state(false)
    {
        m_icon = icon;
        // HAComponent<Component::Switch>::m_switches.push_back(this);
    }


    void initialize() override;
    void setState(bool state);
    void reportState();

    static const char* ON;
    static const char* OFF;
};

// Specialization of Component of type BinarySensor
template<>
class HAComponent<Component::BinarySensor> : public HACompBase<Component::BinarySensor>
{
protected:
    BinarySensorClass m_sensor_class;

    virtual void getConfigInfo(JsonObject& json);
public:
    HAComponent(ComponentContext& context, const char* id, const char* name, BinarySensorClass sensor_class = BinarySensorClass::Undefined, const char* icon = nullptr) :
        HACompBase(context, id, name),
        m_sensor_class(sensor_class)
    {
        m_icon = icon;
    }

    void reportState(bool state);
};

// Specialization of Component of type Select
template<>
class HAComponent<Component::Select> : public HACompBase<Component::Select>
{
protected:
    std::string m_state;
    String m_cmd_topic;
    std::function<bool(std::string)> m_callback;
    std::vector<std::string> m_options;

    virtual void getConfigInfo(JsonObject& json);
    void handleMqttUpdate(String topic, String value) override;
public:
    HAComponent(ComponentContext& context, const char* id, const char* name, std::function<bool(std::string)> callback, std::vector<std::string> options, const char* icon = nullptr) :
        HACompBase(context, id, name),
        m_callback(callback),
        m_options(options),
        m_state{}
    {
        m_icon = icon;
    }

    void addOption(std::string value) {
        m_options.push_back(value);
    }
    void addOptions(std::vector<std::string> options) {
        for (auto& opt : options) {
            m_options.push_back(opt);
        }
    }

    void initialize() override;
    void setState(std::string state);
    void reportState();
};


// Specialization of Component of type Number
template<>
class HAComponent<Component::Number> : public HACompBase<Component::Number>
{
protected:
    float m_state;
    String m_cmd_topic;
    std::function<void(float)> m_callback;
    InputRange m_range;

    virtual void getConfigInfo(JsonObject& json);
    void handleMqttUpdate(String topic, String value) override;
public:
    HAComponent(ComponentContext& context, const char* id, const char* name, std::function<void(float)> callback, InputRange range, const char* icon = nullptr) :
        HACompBase(context, id, name),
        m_callback(callback),
        m_state(0.0f),
        m_range(range)
    {
        m_icon = icon;
    }

    HAComponent(ComponentContext& context, const char* id, const char* name, const char* m_topic, InputRange range, const char* icon = nullptr) :
        HACompBase(context, id, name),
        m_cmd_topic(m_topic),
        m_state(0.0f),
        m_range(range)
    {
        m_icon = icon;
        m_state_topic = m_topic;
    }

    void initialize() override;
    void setState(float state);
    void reportState();
};

// Device availability component
class HAAvailabilityComponent : public HACompBase<Component::BinarySensor>
{
protected:
    virtual void getConfigInfo(JsonObject& json);
public:
    HAAvailabilityComponent(ComponentContext& context);

    static const char* ONLINE;
    static const char* OFFLINE;

    String getWillTopic();
    void initialize() override;
    void connect();

    // Singleton
    static HAAvailabilityComponent* inst;
};
