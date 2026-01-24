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


bool s_enable_heap_log = true;
const size_t s_min_heap_log_size = 2048;

void * __real_malloc(size_t size);
void * __real_calloc(size_t n, size_t size);
void * __real_realloc(void *ptr, size_t size);

static volatile int malloc_trace_reentrant = 0;
static volatile int lwip_wrap_reentrant = 0;

static inline void log_alloc(const char *op, void *ptr, size_t size, void *caller)
{
    // Not interested in small allocations
    if (!s_enable_heap_log || (size < s_min_heap_log_size)) return;

    // Avoid recursion into malloc hooks
    if (__atomic_fetch_add(&malloc_trace_reentrant, 1, __ATOMIC_RELAXED) != 0) {
        __atomic_fetch_sub(&malloc_trace_reentrant, 1, __ATOMIC_RELAXED);
        return;
    }

    const char *task_name = "no-task";

    // Only call task APIs when scheduler is running and we have a current task handle.
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        TaskHandle_t h = xTaskGetCurrentTaskHandle();
        if (h != NULL) {
            const char *n = pcTaskGetName(h);
            if (n && n[0]) task_name = n;
            else task_name = "unnamed-task";
        }
    }

    // Print address of caller (resolve offline with addr2line)
    // %PLATFORMIO%\tools\toolchain-xtensa-esp-elf\bin\xtensa-esp32-elf-addr2line.exe
    //   -e .pio\build\lupa-espresso-v1\firmware.elf 0x4010bc2c
    ets_printf("HEAP %s %u by %p %s\n", op, (unsigned)size, caller, task_name);

    __atomic_fetch_sub(&malloc_trace_reentrant, 1, __ATOMIC_RELAXED);
}

void * __wrap_malloc(size_t size)
{
    void *caller = __builtin_return_address(0);
    void *p = __real_malloc(size);
    log_alloc("malloc", p, size, caller);
    return p;
}

void * __wrap_calloc(size_t n, size_t size)
{
    void *caller = __builtin_return_address(0);
    void *p = __real_calloc(n, size);
    log_alloc("calloc", p, n * size, caller);
    return p;
}

void * __wrap_realloc(void *ptr, size_t size)
{
    void *caller = __builtin_return_address(0);
    void *p = __real_realloc(ptr, size);
    log_alloc("realloc", p, size, caller);
    return p;
}

// // C++ new/delete forward to wrapped malloc/free
// void * operator new(size_t size) {
//     return __wrap_malloc(size);
// }
// void operator delete(void* p) noexcept {
//     __wrap_free(p);
// }
// void * operator new[](size_t size) {
//     return __wrap_malloc(size);
// }
// void operator delete[](void* p) noexcept {
//     __wrap_free(p);
// }