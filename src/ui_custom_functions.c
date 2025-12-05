#include "../include/liblvgl/lvgl.h"
#include "../include/ui.h"
#include "../include/ui_custom_functions.h"
#include "main.h"

//Global series pointers for each series
lv_chart_series_t * u_series;
lv_chart_series_t * e_series;
lv_chart_series_t * encoder_series;
lv_chart_series_t * distance_series;

void ui_create_chart_series(void) {

    if (u_series == NULL) {
        u_series = lv_chart_add_series(ui_Chart, 
            lv_color_hex(0xFF0000), LV_CHART_AXIS_PRIMARY_Y);
    }

    if (e_series == NULL) {
        e_series = lv_chart_add_series(ui_Chart, 
            lv_color_hex(0x00FF00), LV_CHART_AXIS_PRIMARY_Y);
    }

    if (encoder_series == NULL) {
        encoder_series = lv_chart_add_series(ui_Chart, 
            lv_color_hex(0x0000FF), LV_CHART_AXIS_SECONDARY_Y);
    }

    if (distance_series == NULL) {
        distance_series = lv_chart_add_series(ui_Chart, 
            lv_color_hex(0xFFFF00), LV_CHART_AXIS_SECONDARY_Y);
    }
}

    void graph_update_task(void *param)
    {
        while (1)
        {
            if (lv_scr_act() == ui_MainScreen) {
    
                //Implement a way to get u values
                if (lv_obj_has_state(ui_PlotUCheckbox, LV_STATE_CHECKED)) {
                    // int u_val = get_encoder_value(); // your function
                    // lv_chart_set_next_value(ui_Chart, u_series, u_val);
                }

                //Implement a way to get e values
                if (lv_obj_has_state(ui_PlotECheckbox, LV_STATE_CHECKED)) {
                    // int e_val = get_distance_value(); // your function
                    // lv_chart_set_next_value(ui_Chart, e_series, e_val);
                }

                if (lv_obj_has_state(ui_PlotEncodersCheckbox, LV_STATE_CHECKED)) {
                    int encoder_val = 0.5 * (readSensor(LeftEncoder) + readSensor(RightEncoder));
                    lv_chart_set_next_value(ui_Chart, encoder_series, encoder_val);
                }

                if (lv_obj_has_state(ui_PlotDistanceCheckbox, LV_STATE_CHECKED)) {
                    int distance_val = readSensor(SonarSensor);
                    lv_chart_set_next_value(ui_Chart, distance_series, distance_val);
                }
    
               
                lv_chart_refresh(ui_Chart);
            }
    
            delay(50);
        }
    
    }
