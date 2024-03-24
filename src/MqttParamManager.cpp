#include "MqttParamManager.h"
#include <PubSubClient.h>

extern PubSubClient client;

namespace MqttParam {

    MqttParamManager& MqttParamManager::getInstance() {
        // Global singleton instance
        static MqttParamManager inst { ::client, "lupa/config/" };
        return inst;
    }

    void publish() {
        auto& manager = MqttParamManager::getInstance();
        manager.publishValues();
        manager.subscribe();
    }

    bool handleUpdate(String topic, String payload) {
        auto& manager = MqttParamManager::getInstance();
        return manager.handleUpdate(topic, payload);
    }

}