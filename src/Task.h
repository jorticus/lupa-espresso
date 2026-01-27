#pragma once

#include <stdint.h>
#include <freertos/task.h>

#define CORE0 (0)
#define CORE1 (1)

static const uint32_t TASK_STACK_SIZE = 1024;

// Higher numbers have higher priority. Idle loop is 0.

#define TASK_PRIORITY_SENSOR_SAMPLER (5)
#define TASK_PRIORITY_BREW_CONTROL   (4)
#define TASK_PRIORITY_CORE           (3)
#define TASK_PRIORITY_NETWORK        (2)
#define TASK_PRIORITY_UI             (1)
