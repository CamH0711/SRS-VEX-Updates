#include "controller_telemetry.h"
#include "ui.h"
#include "liblvgl/lvgl.h"

volatile controller_sample_t controller_sample = {0};

void log_controller_sample(int error, int u) {
    controller_sample.error = error;
    controller_sample.control_effort = (int)u;
}