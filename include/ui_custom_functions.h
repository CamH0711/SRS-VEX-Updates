#pragma once
#include "../include/liblvgl/lvgl.h"

// Expose the series pointers to other files
extern lv_chart_series_t *u_series;
extern lv_chart_series_t *e_series;
extern lv_chart_series_t *encoder_series;
extern lv_chart_series_t *distance_series;

// Function to create the series after UI loads
void ui_create_chart_series(void);

// Task loop
void graph_update_task(void *param);