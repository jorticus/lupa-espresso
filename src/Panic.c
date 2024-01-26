#include <Arduino.h>
#include <esp_attr.h>
#include <esp_private/panic_internal.h>

#include <stdint.h>

#ifdef USE_PANIC_WRAPPER

// These variables are not erased on reset,
// providing temporary non-volatile storage in RAM
// for the panic reason:
RTC_NOINIT_ATTR char     panic_buffer[1024];
RTC_NOINIT_ATTR uint32_t panic_len;


// components/esp_system/panic.c
// -Wl,-wrap=panic_print_char
void __real_panic_print_char(char c);
void __wrap_panic_print_char(const char c) {
    if (c == '\n' || panic_len == 0) {
        __real_panic_print_char('>');
    }
    __real_panic_print_char(c);
    if (panic_len < sizeof(panic_buffer)) {
        panic_buffer[panic_len++] = c;
    }
}

// // -Wl,-wrap=esp_panic_handler
// void __real_esp_panic_handler(panic_info_t *info);
// void __wrap_esp_panic_handler(panic_info_t *info) {
//     char s[128];
//     int len = snprintf(s, sizeof(s), "PANIC: %s (%s)\n", 
//         info->reason, info->description);

//     // Store formatted string in panic buffer
//     for (int i = 0; i < len; i++) {
//         __wrap_panic_print_char(s[i]);
//     }

//     __real_esp_panic_handler(info);
// }

// void IRAM_ATTR __attribute__((noreturn, no_sanitize_undefined)) __real_panic_abort(const char *details);
// void IRAM_ATTR __attribute__((noreturn, no_sanitize_undefined)) __wrap_panic_abort(const char *details) {
//     char s[128];
//     int len = snprintf(s, sizeof(s), "ABORT: %s\n", details);

//     // Store formatted string in panic buffer
//     for (int i = 0; i < len; i++) {
//         __wrap_panic_print_char(s[i]);
//     }

//     __real_panic_abort(details);
// }


void clear_panic_buffer() {
    panic_len = 0;
}

const char* get_panic_buffer() {
    panic_buffer[panic_len] = '\0';
    panic_len = 0;
    return panic_buffer;
}

void print_panic_buffer() {
    if (panic_len > 0) {
        for (int i = 0; i < 20; i++)
            __real_panic_print_char('-');
        __real_panic_print_char('\n');

        for (int i = 0; i < panic_len; i++) {
            __real_panic_print_char(panic_buffer[i]);
        }

        for (int i = 0; i < 20; i++)
            __real_panic_print_char('-');
        __real_panic_print_char('\n');
        
    }
    panic_len = 0;
}

#else

void clear_panic_buffer() { }

const char* get_panic_buffer() { 
    static char* s = "";
    return s;
}

void print_panic_buffer() { }

#endif
