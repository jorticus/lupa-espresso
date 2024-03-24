#pragma once

extern "C" {
    void clear_panic_buffer();
    void print_panic_buffer();
    const char* get_panic_buffer();
}
