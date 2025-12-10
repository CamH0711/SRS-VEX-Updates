#ifndef CONTROLLER_LOG_H
#define CONTROLLER_LOG_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int error;
    int control_effort;
} controller_sample_t;

extern volatile controller_sample_t controller_sample;

extern void log_controller_sample(int error, int u);

#endif // CONTROLLER_LOG_H
