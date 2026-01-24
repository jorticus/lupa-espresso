# HomeAssistant Component Helper

This is a C++ helper for defining HomeAssistant components using MQTT (PubSubClient).

It supports the following components:

- sensor
- binary_sensor
- switch
- select
- availability / connectivity status

## Usage

First define a context using your pubsub client:

```c
Stream& Debug = Serial;

PubSubClient client;
ComponentContext mqtt_context(client);
```

Then define your HomeAssistant components:

```c
// This is a special component that mirrors the device availability state
HAAvailabilityComponent availability(mqtt_context);

// Example analog sensors:
HAComponent<Component::Sensor> sensor_temp(
    mqtt_context, 
    "temp1", // ID (lower_case)
    "Temperature", // Friendly Name
    TEMP_SAMPLE_INTERVAL_MS, // Sample rate
    0.f, // Hysteresis (values smaller than this difference are not reported)
    SensorClass::Temperature
);
HAComponent<Component::Sensor> sensor_humid(
    mqtt_context, 
    "humid1", 
    "Humidity", 
    TEMP_SAMPLE_INTERVAL_MS, 
    0.f, 
    SensorClass::Humidity
);

// Example switch controlling a fan:
HAComponent<Component::Switch> switch_fan(
    mqtt_context, 
    "fan", "Fan",
    [](bool state) {
        Debug.println(state ? "Fan on!" : "Fan off!");
        digitalWrite(FAN_PIN, state);
    },
    "mdi:fan"   // Optional icon
);

// Example binary state:
HAComponent<Component::BinarySensor> my_state(
    mqtt_context,
    "state", "My State",
    BinarySensorClass::Undefined,
    "mdi:coffee" // Optional icon
);

// Example select input:
HAComponent<Component::Select> select_custom(
    mqtt_context, 
    "selector", "Selector",
    [](std::string state) {
        Debug.print("Selected: ");
        Debug.println(state);
        
        if (state == "Option A") {
            // Do something
        }
    },
    {
        "Option A",
        "Option B",
        "Option C"
    }
);
```

To get things started, you need to connect to your MQTT client and use the HAAvailabilityComponent's last will topic, 
and set up a message received callback that forwards messages to the switch component:

```c

void setup() {
    ...
    
    // Important: MAC address string reference must stick around forever
    static String mac = WiFi.macAddress();
    context.mac_address = mac.c_str();

    // Metadata to report to HomeAssistant
    mqtt_context.device_name = "your_device";   // Should be lower_case
    mqtt_context.fw_version = "1.0.0";
    mqtt_context.friendly_name = "Your Device";
    mqtt_context.model = "Device Model";
    mqtt_context.manufacturer = "Manufacturer";
    
    // Initialize each component with the above metadata
    HAComponentManager::initializeAll();

    // Important: HA config payloads need more than the default of 256 bytes
    client.setBufferSize(HA_MQTT_MAX_PACKET_SIZE);

    client.setCallback(HAComponentManager::OnMessageReceived);
    
    while (!client.connected()) {
        // Wrapper for client.connect that sets up the availability will topics
        HAComponentManager::connectClientWithAvailability(client, 
            secrets::device_name, 
            secrets::mqtt_username, 
            secrets::mqtt_password);
    }
    
    // Now MQTT is connected, we can publish the components to HomeAssistant:
    HAComponentManager::publishConfigAll();

     // And report that our device is now alive:
    availability.connect();
    
}
```

Finally you can update your sensor readings as needed:
```c
void loop() {
    static unsigned long t_last = 0;
    
    client.loop();

    float temperature = readTemperature();

    // Note: temperature will only be published at the defined reporting interval
    sensor_temp.update(temperature);
}
```

