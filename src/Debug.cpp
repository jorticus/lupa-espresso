#include "Debug.h"
#include "hardware.h"
// #include <TelnetSpy.h> // TODO: Doesn't compile with latest Arduino framework?

//static WiFiServer server;
// static TelnetSpy telnet; // TODO
static bool s_booted = false;
Stream& Debug = Serial;
// Stream& Debug = telnet;

int custom_vprintf(const char* str, va_list l) {
    static char buf[128];
    auto len = vsnprintf(buf, sizeof(buf), str, l);
    Debug.write("ESP:");
    return Debug.write(buf, len);
}


void DebugLogger::init() {
#if 0
    telnet.begin(UART_DEBUG_BAUD);
    telnet.setWelcomeMsg("Connected to LUPA\n");
    telnet.setRejectMsg("Connection rejected\n");
    //telnet.setBufferSize(3000);
    telnet.setStoreOffline(true);
    telnet.setSerial(&Serial);
    telnet.write("-- Boot Log --\n");

    esp_log_set_vprintf(custom_vprintf);

    //xTaskCreatePinnedToCore(&wifi_logger, "wifi_logger", 4096, NULL, 2, NULL, 1);
    //wifi_logger_queue = xQueueCreate(MESSAGE_QUEUE_SIZE, sizeof(char*))
#endif
}

void DebugLogger::process() {
#if 0
    if (!s_booted) {
        s_booted = true;
        // Stop collecting boot messages in buffer if no client is connected
        telnet.write("-- End of Boot Log --\n");
        telnet.setStoreOffline(false);
    }

    telnet.handle();
#endif
}
