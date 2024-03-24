#include "Debug.h"
#include <TelnetSpy.h>

//static WiFiServer server;
static TelnetSpy telnet;
static bool s_booted = false;
//Stream& Debug = Serial;
Stream& Debug = telnet;

int custom_vprintf(const char* str, va_list l) {
    static char buf[128];
    auto len = vsnprintf(buf, sizeof(buf), str, l);
    Debug.write("ESP:");
    return Debug.write(buf, len);
}


void DebugLogger::init() {
    telnet.begin(9600);
    telnet.setWelcomeMsg("Connected to LUPA\n");
    telnet.setRejectMsg("Connection rejected\n");
    //telnet.setBufferSize(3000);
    telnet.setStoreOffline(true);
    telnet.setSerial(&Serial);
    telnet.write("-- Boot Log --\n");

    esp_log_set_vprintf(custom_vprintf);

    //xTaskCreatePinnedToCore(&wifi_logger, "wifi_logger", 4096, NULL, 2, NULL, 1);
    //wifi_logger_queue = xQueueCreate(MESSAGE_QUEUE_SIZE, sizeof(char*))
}

void DebugLogger::process() {
    if (!s_booted) {
        s_booted = true;
        // Stop collecting boot messages in buffer if no client is connected
        telnet.write("-- End of Boot Log --\n");
        telnet.setStoreOffline(false);
    }

    telnet.handle();
}
