#pragma once

#include <freertos/task.h>

#define CORE0 (0)
#define CORE1 (1)

static const uint32_t TASK_STACK_SIZE = CONFIG_ESP_MAIN_TASK_STACK_SIZE;
