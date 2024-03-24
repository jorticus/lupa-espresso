#include "MqttParamManager.h"
#include <PubSubClient.h>

extern PubSubClient client;

namespace MqttParam {

    MqttParamManager Manager(client, "lupa/config/");

    void publish() {
        Manager.publishValues();
        Manager.subscribe();
    }

    bool handleUpdate(String topic, String payload) {
        return Manager.handleUpdate(topic, payload);
    }

}