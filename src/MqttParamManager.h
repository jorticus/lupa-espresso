#pragma once
#include <vector>
#include <functional>
#include <PubSubClient.h>

extern Stream& Debug;

const int MAX_PARAM_NAME_LEN = 64;

class MqttParamManager;

class MqttParamBase {
public:
    MqttParamBase(const char* name) : name(name) { }
    const char* name;

    virtual bool handleUpdate(String payload) = 0;
    virtual void publish() = 0;
};

class MqttParamManager {
public:
    MqttParamManager(PubSubClient& client, const char* prefix) : 
        client(client),
        prefix(prefix)
    { }

    bool handleUpdate(String topic, String payload) {
        if (topic.startsWith(this->prefix)) {
            auto prefix_len = strlen(this->prefix);
            auto name = topic.substring(prefix_len);
            // Debug.print("Parameter: "); Debug.print(name);
            // Debug.print(" = ");
            // Debug.print(payload);
            // Debug.println();

            for (auto* param : this->parameters) {
                if (name == param->name) {
                    if (param->handleUpdate(payload)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void subscribe() {
        char buf[MAX_PARAM_NAME_LEN];
        snprintf(buf, sizeof(buf), "%s#", this->prefix);
        Serial.printf("Subscribe to topic %s\n", buf);
        client.subscribe(buf);
    }

    void publishValues() {
        for (auto* param : this->parameters) {
            param->publish();
        }
    }

    void addParam(MqttParamBase& param) {
        parameters.push_back(&param);
    }

public:
    const char* prefix;
    PubSubClient& client;

protected:
    std::vector<MqttParamBase*> parameters;

private:
    
};


template <typename T>
static T strToValue(String& s) {
    return s;
}
template <>
float strToValue(String& s) {
    auto f = s.toFloat();
    return f;
}
template <>
int strToValue(String& s) {
    return s.toInt();
}
template <>
double strToValue(String& s) {
    return s.toDouble();
}
template <>
bool strToValue(String& s) {
    String sl(s);
    sl.toLowerCase();
    return (sl == "true" || sl == "on" || sl == "1");
}

/// @brief Declare a parameter that can be configured via MQTT
/// @details 
/// The parameter may include a callback function that is invoked whenever the parameter is configured via MQTT
/// The type can be [float,double,int,bool,String]
template<typename T>
class MqttParam : public MqttParamBase {
public:

    MqttParam(MqttParamManager& manager, const char* name, T defaultValue = {}) :
        MqttParamBase { name },
        manager(manager), _value(defaultValue), has_cb(false)
    {
        manager.addParam(*this);
    }

    MqttParam(MqttParamManager& manager, const char* name, std::function<void(T)> update_cb) :
        MqttParamBase { name },
        manager(manager), _value{}, update_cb(update_cb), has_cb(true)
    {
        manager.addParam(*this);
    }

    T value() const {
        return this->_value;
    }

    void set(T val) {
        this->_value = val;
        // TODO: Should this publish the value?
    }

    bool handleUpdate(String payload) override {
        this->_value = strToValue<T>(payload);
        Serial.printf("Param %s = ", this->name); Serial.println(this->_value);
        if (has_cb) {
            update_cb(_value);
        }
        return true;
    }

    void publish() override {
        char topic[MAX_PARAM_NAME_LEN];
        snprintf(topic, sizeof(topic), "%s%s", manager.prefix, this->name);
        String value_str(this->_value);
        Serial.printf("Publish topic %s\n", topic);
        manager.client.publish(topic, value_str.c_str());
    }

private:
    bool has_cb;
    T _value;
    MqttParamManager& manager;
    std::function<void(T)> update_cb;
};

