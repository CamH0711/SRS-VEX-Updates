 #include <string.h>
 #include <sys/stat.h>
 #include "main.h"
 #include "pros/adi.h"
 #include "pros/distance.h"
 #include "../include/ui.h"
 #include "Student_Code.h"

 /**** GLOBAL VARIABLES ****/

 volatile bool print_panel_visible = true;
 char print_buffers[8][51];
 bool print_dirty[8];
 volatile bool pause_active = false;
 int current_y_min = 0;
 int current_y_max = 100;
 bool plot_left_enc_enabled = false;
 bool plot_right_enc_enabled = false;
 bool plot_arm_enabled = false;
 bool plot_sonar_enabled = false;
 bool plot_left_light_enabled = false;
 bool plot_mid_light_enabled = false;
 bool plot_right_light_enabled = false;
 numpad_ctx_t numpad_ctx;
 int plotting_rate = 100;
 variable_slot_t variable_slots[3];
 bool numpad_is_open = false;
 void * current_target_ptr = NULL;
 uint32_t last_config_time = 0;
 bool chart_needs_resize = false;
 int observed_min = INT32_MAX;
 int observed_max = INT32_MIN;
 int shrink_counter = 0;
 bool program_ended_normally_flag = false;
bool is_logging = false;
 bool logger_is_busy = false;
 uint32_t log_start_timestamp = 0;
 int log_rate = 100; // default 100ms sampling rate
 char current_filename[128];
 plot_registry_t session_plots[MAX_CUSTOM_PLOTS];
 int session_plot_count = 0;
 FILE* temp_log_file = NULL;
 lv_timer_t *logging_timer = NULL;


 #define SHRINK_DELAY_TICKS 40   // ~2 second if timer = 50ms
 #define CHART_GROW_STEP 50


 /**** CUSTOM FUNCTIONS ****/

void ui_event_Switch(lv_event_t * e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    print_panel_visible = !print_panel_visible;

    _ui_flag_modify(ui_PrintPanel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
    _ui_flag_modify(ui_Chart, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
    _ui_flag_modify(ui_Legend, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);

    if(print_panel_visible) {
        lv_obj_invalidate(ui_PrintPanel);
    }
}

void ui_event_SettingsButton(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_SettingsScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_SettingsScreen_screen_init);
    }
}

void ui_event_OnScreenStopButton(lv_event_t * e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    // Stop motors immediately
    motor_move(_motorLeft, 0);
    motor_move(_motorRight, 0);
    motor_move(_motorArm, 0);

    // Stop graph updates
    if (graph_timer) {
        lv_timer_del(graph_timer);
        graph_timer = NULL;
    }

    // Stop logging if active
    StopDataLogging();

    // Show STOP banner
    lv_label_set_text(ui_StopText, "STOP BUTTON PRESSED!");
    lv_label_set_text(ui_StopText2, "STOP BUTTON PRESSED!");
    lv_obj_clear_flag(ui_StopPanel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_StopPanel2, LV_OBJ_FLAG_HIDDEN);

    _stopflag = 1;

    // Schedule program exit AFTER 5 seconds
    lv_timer_t * t = lv_timer_create(ExitProgram, 5000, NULL);
    lv_timer_set_repeat_count(t, 1);
}

void ui_event_ResumeButton(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        pause_active = false;
    }
}

/*Custom Functions */

// A function that updates the Y axis of the graph based on the current max and min values
void update_y_axis(int min, int max)
{
    if (max <= min) max = min + 1;

    int divisions = 4;                 // total labeled major ticks - 1
    int major_step = (max - min) / divisions;
    if (major_step < 1) major_step = 1;

    int total_ticks = divisions * 2 + 1;   // because SquareLine uses 2 minor ticks per division

    current_y_max = max;
    current_y_min = min;

    // LEFT Y axis
    lv_scale_set_range(ui_Chart_Yaxis1, min, max);
    lv_scale_set_total_tick_count(ui_Chart_Yaxis1, total_ticks);
    lv_scale_set_major_tick_every(ui_Chart_Yaxis1, 2);
    lv_scale_set_label_show(ui_Chart_Yaxis1, true);
}

// A custom printf-like function for printing to the LVGL screen
void ScreenPrint(int line_number, char* text, ...) {
if (line_number < 1 || line_number > 8) return;
    int index = line_number - 1;

    va_list args;
    va_start(args, text);

    // 1. Attempt to format the string into the buffer.
    int result = vsnprintf(print_buffers[index], 51, text, args);
    
    va_end(args);

    // 2. If result is >= our limit, the string was truncated.
    if (result >= 51) {
        // Overwrite the last 3 visible spots with dots.
        // Index [MAX_PRINT_CHARS] is the \0, so we go back 3 from there.
        print_buffers[index][47] = '.';
        print_buffers[index][48] = '.';
        print_buffers[index][49] = '.';
        print_buffers[index][50]     = '\0'; 
    }

    print_dirty[index] = true;
}

// A student accessible function that displays the graph automatically on startup
void ToggleChart() {
    // 1. Toggle the visibility variable first
    print_panel_visible = !print_panel_visible;

    // 2. Set the switch state based on the new variable
    if (print_panel_visible) {
        lv_obj_clear_state(ui_Switch, LV_STATE_CHECKED);
    } else {
        lv_obj_add_state(ui_Switch, LV_STATE_CHECKED);
    }

    // 3. Toggle the UI elements
    _ui_flag_modify(ui_PrintPanel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
    _ui_flag_modify(ui_Chart, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
    _ui_flag_modify(ui_Legend, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
}

// A student accessible function that pauses the entire program.
void Pause() {
    motor_move(_motorLeft, 0);
    motor_move(_motorRight, 0);
    motor_move(_motorArm, 0);

    pause_active = true;

    // Wait for logger to acknowledge pause (prevents mid-write corruption)
    int timeout = 100; // 100ms timeout
    while (logger_is_busy && timeout > 0) {        
        delay(10);
        timeout = timeout - 10;
    }

    // Flush any buffered data
    if (is_logging && temp_log_file != NULL) {
        fflush(temp_log_file);
    }

    // Stop graph updates while paused
    lv_timer_del(graph_timer);
    graph_timer = NULL;

    while(pause_active) {
        delay(100);
    }

    // Resume graph updates
    graph_timer = lv_timer_create(GraphUpdateTask, plotting_rate, NULL);
}

void ui_event_BackToMainButton(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_MainScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_MainScreen_screen_init);
    }
}


void ui_event_Slot1Button(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        lv_obj_clear_flag(ui_NumPad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_state(ui_PlottingRateDropdown, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot1Button, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot2Button, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot3Button, LV_STATE_DISABLED);

        if (variable_slots[0].active) {
        // Open the Numpad targeting the stored pointer
        NumpadOpen(
            variable_slots[0].var_ptr, 
            variable_slots[0].type, 
            variable_slots[0].slot_label, 
            variable_slots[0].name, 
            6, 2
        );
    }

    }
}

void ui_event_Slot2Button(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        lv_obj_clear_flag(ui_NumPad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_state(ui_PlottingRateDropdown, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot1Button, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot2Button, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot3Button, LV_STATE_DISABLED);

        if (variable_slots[1].active) {
        // Open the Numpad targeting the stored pointer
        NumpadOpen(
            variable_slots[1].var_ptr, 
            variable_slots[1].type, 
            variable_slots[1].slot_label, 
            variable_slots[1].name, 
            6, 2
        );
    }
    }
}

void ui_event_Slot3Button(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        lv_obj_clear_flag(ui_NumPad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_state(ui_PlottingRateDropdown, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot1Button, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot2Button, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot3Button, LV_STATE_DISABLED);
        
        if (variable_slots[2].active) {
        // Open the Numpad targeting the stored pointer
        NumpadOpen(
            variable_slots[2].var_ptr, 
            variable_slots[2].type, 
            variable_slots[2].slot_label, 
            variable_slots[2].name, 
            6, 2
        );
    }
    }
}

void ui_event_ButtonDone(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        lv_obj_add_flag(ui_NumPad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_state(ui_PlottingRateDropdown, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_Slot1Button, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_Slot2Button, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_Slot3Button, LV_STATE_DISABLED);
    //Reset the label color to White
    if (numpad_ctx.target_label != NULL) {
        lv_obj_set_style_text_color(numpad_ctx.target_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    // Safety check: ensure we have a valid pointer
    if (numpad_ctx.target_value == NULL) return;

    // If the student deleted everything, force it to zero
    if (strlen(numpad_ctx.buffer) == 0) {
        strcpy(numpad_ctx.buffer, "0");
    }

    if (numpad_ctx.type == TYPE_DOUBLE) {
        double *val = (double *)numpad_ctx.target_value;
        *val = atof(numpad_ctx.buffer); // Convert string to double
    } 
    else if (numpad_ctx.type == TYPE_INT) {
        int *val = (int *)numpad_ctx.target_value;
        *val = atoi(numpad_ctx.buffer); // Convert string to int
    }

    numpad_is_open = false;      // Reset the flag
    current_target_ptr = NULL;   // Clear the pointer
    }
}

void ui_event_PlottingRateDropdown(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
        char selected[8];
        lv_dropdown_get_selected_str(ui_PlottingRateDropdown, selected, sizeof(selected));
        plotting_rate = atoi(selected);

        if(graph_timer) {
            lv_timer_set_period(graph_timer, plotting_rate);
        }
    }
}

// Show/Hide left wheel encoder series
void ui_event_PlotLeftEncCheckbox(lv_event_t * e) {
     plot_left_enc_enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
}
// Show/Hide right wheel encoder series
void ui_event_PlotRightEncCheckbox(lv_event_t * e) {
     plot_right_enc_enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
}
// Show/Hide arm encoder series
void ui_event_PlotArmEncCheckbox(lv_event_t * e) {
     plot_arm_enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
}
// Show/Hide left distance sensor series
void ui_event_PlotSonarCheckbox(lv_event_t * e) {
     plot_sonar_enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
}

// A function that can be used to check a box automatically to plot data
void PlotSensor(int data_name) {
    if (data_name == SonarSensor) {
        lv_obj_add_state(ui_PlotSonarCheckbox, LV_STATE_CHECKED);
    } else if (data_name == LeftEncoder) {
        lv_obj_add_state(ui_PlotLeftEncCheckbox, LV_STATE_CHECKED);
    } else if (data_name == RightEncoder) {
        lv_obj_add_state(ui_PlotRightEncCheckbox, LV_STATE_CHECKED);
    } else if (data_name == ArmEncoder) {
        lv_obj_add_state(ui_PlotArmEncCheckbox, LV_STATE_CHECKED);
    } else if (data_name == LeftLight) {
        plot_left_light_enabled = true;
    } else if (data_name == MidLight) {
        plot_mid_light_enabled = true;
    } else if (data_name == RightLight) {
        plot_right_light_enabled = true;
    }
}


// A timer function that clears the values from a series when enabled
void ClearSeriesTimer(lv_timer_t * t) {
    // Retrieve the series pointer from the timer's user_data
    lv_chart_series_t * series = (lv_chart_series_t *)t->user_data;
    if (series) {
        lv_chart_set_all_value(ui_Chart, series, LV_CHART_POINT_NONE);
    }}

// A helper function that clears a series using a timer.
void ClearSeries(lv_chart_series_t * series) {
    if (!series) return;
    lv_timer_t * t = lv_timer_create(ClearSeriesTimer, 20, series);
    lv_timer_set_repeat_count(t, 1);
}

// A function that retrieves all of the info about a variable slot when the student clicks adjust on the screen
void NumpadOpen(void *value, numpad_data_type_t type, lv_obj_t *label, const char *prefix, int max_len, int max_dec){
    // Store the label data
    numpad_ctx.target_value = value;
    numpad_ctx.type = type;
    numpad_ctx.target_label = label;
    numpad_ctx.prefix = prefix;

    // Store the limits
    numpad_ctx.max_length = max_len;
    numpad_ctx.max_decimals = max_dec;

    // formatting based on type
    if (type == TYPE_DOUBLE) {
        double *d_ptr = (double *)value;
        snprintf(numpad_ctx.buffer, sizeof(numpad_ctx.buffer), "%.2f", *d_ptr);
    } 
    else if (type == TYPE_INT) {
        int *i_ptr = (int *)value;
        snprintf(numpad_ctx.buffer, sizeof(numpad_ctx.buffer), "%d", *i_ptr);
    }

    current_target_ptr = value;  // Set the target pointer
    numpad_is_open = true;       // Set the flag

    lv_obj_set_style_text_color(numpad_ctx.target_label, lv_color_hex(0xF44336), LV_PART_MAIN | LV_STATE_DEFAULT);

    UpdateTargetLabel();
}

// A function that handles the events from the numpad buttons
void NumpadButtonEvent(lv_event_t *e)
{
    const char *key = lv_event_get_user_data(e);
    size_t len = strlen(numpad_ctx.buffer);
    char *decimal_ptr = strchr(numpad_ctx.buffer, '.');

    // --- HANDLE DELETE ---
    if (strcmp(key, "DEL") == 0) {
        if (len > 0) {
            numpad_ctx.buffer[len - 1] = '\0';
        }
    } 
    // --- HANDLE INPUT ---
    else {
        // 1. TOTAL LENGTH CHECK
        // If we reached the max length, do nothing.
        if (len >= numpad_ctx.max_length) {
            return; 
        }
        // 2. DECIMAL POINT CHECK
        if (key[0] == '.') {
            // If we already have a decimal, do nothing
            if (decimal_ptr != NULL) return;
            
            // If this is an Integer type variable, block decimals entirely
            if (numpad_ctx.type == TYPE_INT) return;
        }
        // 3. DECIMAL PRECISION CHECK
        // If we are currently typing digits AFTER the decimal point
        if (decimal_ptr != NULL && key[0] != '.') {
            // Calculate how many digits are currently after the dot
            // ptr points to '.', so ptr+1 is the first digit after it
            int decimals_existing = strlen(decimal_ptr + 1);

            if (decimals_existing >= numpad_ctx.max_decimals) {
                return;
            }
        }
        // 4. APPEND IF SAFE
        if (len < sizeof(numpad_ctx.buffer) - 1) {
            strcat(numpad_ctx.buffer, key);
        }
    } 
    UpdateTargetLabel();
}

// A function that updates the variable label with the current variable value as the student types
void UpdateTargetLabel(void)
{
if (numpad_ctx.target_label == NULL) return;

    lv_label_set_text_fmt(numpad_ctx.target_label, "%s = %s", 
                         numpad_ctx.prefix, numpad_ctx.buffer);
}

// A helper function - for finding the closest value in the dropdown to the target value
int FindClosestDropdownIndex(lv_obj_t * dropdown, int target_value) {
    const char * options = lv_dropdown_get_options(dropdown);
    if (options == NULL) return 0;

    // Create a local copy because strtok modifies the string
    char options_copy[256]; 
    strncpy(options_copy, options, sizeof(options_copy) - 1);
    options_copy[sizeof(options_copy) - 1] = '\0';

    int closest_index = 0;
    int current_index = 0;
    int min_diff = INT32_MAX;

    // Split the string by the newline character
    char * token = strtok(options_copy, "\n");
    while (token != NULL) {
        int option_val = atoi(token);
        int diff = abs(option_val - target_value);

        if (diff < min_diff) {
            min_diff = diff;
            closest_index = current_index;
        }

        token = strtok(NULL, "\n");
        current_index++;
    }

    return closest_index;
}

// A function to set the plotting rate based on a given value
void SetPlottingRate(int rate_ms) {
    if (!ui_PlottingRateDropdown) return;

    // 1. Find the closest index and snap the UI to it
    int idx = FindClosestDropdownIndex(ui_PlottingRateDropdown, rate_ms);
    lv_dropdown_set_selected(ui_PlottingRateDropdown, idx);

    // 2. Get the actual string value of that index to update our variable
    char buf[16];
    lv_dropdown_get_selected_str(ui_PlottingRateDropdown, buf, sizeof(buf));
    plotting_rate = atoi(buf);

    // 3. Update the hardware timer
    if (graph_timer) {
        lv_timer_set_period(graph_timer, plotting_rate);
    }
}

// A function students can use to configure a variable adjustment slot
void _InternalConfig(int slot_num, void * var, numpad_data_type_t type, const char * name) {
    
    int index = slot_num - 1;
    if (index < 0 || index >= 3) return;

    // Check if slots need to be reset
    if (millis() - last_config_time > 100) {
        ResetVariableSlots();
    }
    last_config_time = millis();

    // Reset the abandonment counter
    variable_slots[index].cycles_since_update = 0;

    if (variable_slots[index].var_ptr != var || strcmp(variable_slots[index].name, name) != 0) {
        variable_slots[index].var_ptr = var;
        variable_slots[index].type = type;
        variable_slots[index].active = true;
        
        // Name truncation logic...
        if (strlen(name) > 16) {
            strncpy(variable_slots[index].name, name, 13);
            variable_slots[index].name[13] = '\0';
            strcat(variable_slots[index].name, "...");
        } else {
            strcpy(variable_slots[index].name, name);
        }
    }

    // Initial update
    char buffer[64];
    if (type == TYPE_DOUBLE) {
        snprintf(buffer, sizeof(buffer), "%s = %.2f", name, *(double*)var);
    } else {
        snprintf(buffer, sizeof(buffer), "%s = %d", name, *(int*)var);
    }
    
    if (variable_slots[index].slot_label) {
        lv_label_set_text(variable_slots[index].slot_label, buffer);
    }
}

// A timer function to refresh all variable labels
void RefreshVariableLabels(lv_timer_t * t) {
    if (_stopflag == 1) return; // Do not update if in stop mode

    for (int i = 0; i < 3; i++) {

        // Safety: skip if not active, no pointer, or no label
        if (!variable_slots[i].active || variable_slots[i].var_ptr == NULL || variable_slots[i].slot_label == NULL) {
            continue; 
        }

        // GUARDRAIL: Do not overwrite the label if the student is currently typing!
        if (numpad_is_open && (current_target_ptr == (void*)variable_slots[i].var_ptr)) {
            continue; 
        }

        char buffer[64];
        if (variable_slots[i].type == TYPE_DOUBLE) {
            // Cast the void pointer to double* then dereference
            double val = *(double*)variable_slots[i].var_ptr;
            snprintf(buffer, sizeof(buffer), "%s = %.2f", variable_slots[i].name, val);
        } else {
            // Cast the void pointer to int* then dereference
            int val = *(int*)variable_slots[i].var_ptr;
            snprintf(buffer, sizeof(buffer), "%s = %d", variable_slots[i].name, val);
        }

        lv_label_set_text(variable_slots[i].slot_label, buffer);
    }
}

// A helper function to reset all variable slots
void ResetVariableSlots() {
    for (int i = 0; i < 3; i++) {
        variable_slots[i].active = false;
        variable_slots[i].var_ptr = NULL; 
        if (variable_slots[i].slot_label) {
            lv_label_set_text(variable_slots[i].slot_label, " - ");
        }
    }
}

/**
  * @brief A function that retrieves the value to be plotted for a given plot slot.
  * @param slot_index (int) Index of the plot slot (0 to MAX_PLOT_SLOTS - 1)
  * @return (int) Value to be plotted
  */
 int GetPlotValue(int slot_index) {
    plot_source_t src = plot_slots[slot_index].source;

    if (src == PLOT_CUSTOM) {
        return plot_slots[slot_index].custom_value;
    }

    switch (src) {
        case PLOT_LEFT_ENC:    return readSensor(LeftEncoder);
        case PLOT_RIGHT_ENC:   return readSensor(RightEncoder);
        case PLOT_ARM_ENC:     return readSensor(ArmEncoder);
        case PLOT_SONAR:       return readSensor(SonarSensor);
        case PLOT_LEFT_LIGHT:  return readSensor(LeftLight);
        case PLOT_MID_LIGHT:   return readSensor(MidLight);
        case PLOT_RIGHT_LIGHT: return readSensor(RightLight);
        default:               return 0;
    }
}

/**
  * @brief A function that checks if a data source is enabled for plotting.
  * @param src (plot_source_t) Data source
  * @return (bool) True if the data source is enabled, false otherwise
  */
 bool IsSourceEnabled(plot_source_t src) {
    switch (src) {
        case PLOT_LEFT_ENC:    return plot_left_enc_enabled;
        case PLOT_RIGHT_ENC:   return plot_right_enc_enabled;
        case PLOT_ARM_ENC:     return plot_arm_enabled;
        case PLOT_SONAR:       return plot_sonar_enabled;
        case PLOT_LEFT_LIGHT:  return plot_left_light_enabled;
        case PLOT_MID_LIGHT:   return plot_mid_light_enabled;
        case PLOT_RIGHT_LIGHT: return plot_right_light_enabled;
        default:               return false;
    }
}

/**
  * @brief A function that returns the name of the data source in order to 
  * display it in the legend label.
  * @param src (plot_source_t) Data source
  * @return (char *) Name of the data source
  */
 char *PlotSourceName(plot_source_t src) {
    switch (src) {
        case PLOT_LEFT_ENC:    return "Left Encoder";
        case PLOT_RIGHT_ENC:   return "Right Encoder";
        case PLOT_ARM_ENC:     return "Arm Encoder";
        case PLOT_SONAR:       return "Sonar Sensor";
        case PLOT_LEFT_LIGHT:  return "Left Light";
        case PLOT_MID_LIGHT:   return "Mid Light";
        case PLOT_RIGHT_LIGHT: return "Right Light";
        default:               return "";
    }
}

/**
  * @brief A function that updates the plot slots based on which data sources are enabled.
  * @param none
  */
 void UpdatePlotSlots(void) {
    plot_source_t requested[] = {
        PLOT_LEFT_ENC,
        PLOT_RIGHT_ENC,
        PLOT_ARM_ENC,
        PLOT_SONAR,
        PLOT_LEFT_LIGHT,
        PLOT_MID_LIGHT,
        PLOT_RIGHT_LIGHT
    };

    /* 1. Cleanup and Logger Sync */
    for (int i = 0; i < MAX_PLOT_SLOTS; i++) {
        // Handle standard sources deactivation
        if (plot_slots[i].active && plot_slots[i].source != PLOT_CUSTOM) {
            if (!IsSourceEnabled(plot_slots[i].source)) {
                ClearSeries(plot_slots[i].series);
                lv_obj_add_flag(plot_slots[i].legend_label, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(plot_slots[i].legend_colour_box, LV_OBJ_FLAG_HIDDEN);
                plot_slots[i].active = false;
                plot_slots[i].source = PLOT_NONE;
            }
        }

        if (plot_slots[i].active && plot_slots[i].source == PLOT_CUSTOM) {
            // Increment the counter every time the UI task runs
            plot_slots[i].cycles_since_update++;

            // If 2 UI cycles pass without a CustomPlot() call, the slot is abandoned
            if (plot_slots[i].cycles_since_update >= 2) {
                
                // 1. Clear UI
                ClearSeries(plot_slots[i].series);
                lv_obj_add_flag(plot_slots[i].legend_label, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(plot_slots[i].legend_colour_box, LV_OBJ_FLAG_HIDDEN);

                // 2. Deactivate Slot
                plot_slots[i].active = false;
                plot_slots[i].source = PLOT_NONE;
                
                // 3. Sync with Logger: Tell it this name is no longer recording
                RegisterPlot(plot_slots[i].last_recorded_name, &plot_slots[i].custom_value, false, i);
                
                // Clear the cached name so it can be re-registered fresh later
                memset(plot_slots[i].last_recorded_name, 0, 32);
                
                continue; 
            }
        }

        if (plot_slots[i].source == PLOT_CUSTOM) {
            RegisterPlot(plot_slots[i].custom_name, &plot_slots[i].custom_value, plot_slots[i].active, i);
        }
    }

    /* 2. Assign new sources to free slots */
    for (int r = 0; r < (int)(sizeof(requested)/sizeof(requested[0])); r++) {
        plot_source_t src = requested[r];
        if (!IsSourceEnabled(src)) continue;

        bool already_assigned = false;
        for (int i = 0; i < MAX_PLOT_SLOTS; i++) {
            if (plot_slots[i].active && plot_slots[i].source == src) {
                already_assigned = true;
                break;
            }
        }

        if (already_assigned) continue;

        for (int i = 0; i < MAX_PLOT_SLOTS; i++) {
            if (!plot_slots[i].active) {
                plot_slots[i].source = src;
                plot_slots[i].active = true;
                lv_label_set_text(plot_slots[i].legend_label, PlotSourceName(src));
                lv_obj_clear_flag(plot_slots[i].legend_label, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(plot_slots[i].legend_colour_box, LV_OBJ_FLAG_HIDDEN);
                break;
            }
        }
    }
}

/**
  * @brief A function that allows custom data to be plotted on the main screen chart.
  * @param slot (int) Plot slot number (1 to MAX_PLOT_SLOTS)
  * @param value (int) Value to plot
  * @param name (const char *) Name to display in the legend label
  */
void CustomPlot(int slot, int value, const char * name) {

    int index = slot - 1;

    // Ensure valid slot (1-4) was entered
    if (index < 0 || index >= MAX_PLOT_SLOTS) return;

    // Reset the abandonment counter
    plot_slots[index].cycles_since_update = 0;

    // Immediate Reset if the Name Changed
    if (strcmp(plot_slots[index].last_recorded_name, name) != 0) {
        ClearSeries(plot_slots[index].series);
        strncpy(plot_slots[index].last_recorded_name, name, 31);

        // Sync with logger: tell it the OLD name is now inactive
        RegisterPlot(name, &plot_slots[index].custom_value, true, index);
    }

    // Update the plot_slots fields
    plot_slots[index].source = PLOT_CUSTOM;
    plot_slots[index].active = true;
    plot_slots[index].custom_value = value;
    plot_slots[index].needs_ui_refresh = true;

    // Copy the string safely into the buffer
    strncpy(plot_slots[index].custom_name, name, 31);
    plot_slots[index].custom_name[31] = '\0'; // Ensure null termination

    // Register new variable for data logging
    RegisterPlot(name, &plot_slots[index].custom_value, true, index);
}

/**
  * @brief A timer task that updates the data being graphed on the main screen chart, and automatically
  * scales the Y axis according to the values being plotted.
  * @param timer (lv_timer_t) Pointer to the timer object
  */
 void GraphUpdateTask(lv_timer_t * timer) {

    int local_min = INT32_MAX;
    int local_max = INT32_MIN;

    if (!ui_Chart) return;

    UpdatePlotSlots();

    for (int i = 0; i < MAX_PLOT_SLOTS; i++) {

        if (!plot_slots[i].active || !plot_slots[i].series)
            continue;

        if (plot_slots[i].source == PLOT_CUSTOM && plot_slots[i].needs_ui_refresh) {
            lv_label_set_text(plot_slots[i].legend_label, plot_slots[i].custom_name);
            lv_obj_clear_flag(plot_slots[i].legend_label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(plot_slots[i].legend_colour_box, LV_OBJ_FLAG_HIDDEN);
            plot_slots[i].needs_ui_refresh = false; // Task complete
        }

        int value = GetPlotValue(i);
        lv_chart_set_next_value(ui_Chart, plot_slots[i].series, value);

        chart_needs_resize = true;
        local_min = min(local_min, value);
        local_max = max(local_max, value);
    }

    if (!chart_needs_resize) return;

    /* Autoscale - Expanding Logic */

    // Add some padding
    int range_padding_min = (local_min < 0) ? abs(local_min) / 5 : 0;
    int range_padding_max = (abs(local_max) > 10) ? abs(local_max) / 5 : 5;

    int target_min = local_min - range_padding_min;
    int target_max = local_max + range_padding_max;

    // Ensure valid range
    if (target_max <= target_min) {
        target_max = target_min + 1;
    }

    bool expanded = false;

    // Only allow min to decrease
    if (target_min < current_y_min) {
        current_y_min = target_min;
        expanded = true;
    }

    // Only allow max to increase
    if (target_max > current_y_max) {
        current_y_max = target_max;
        expanded = true;
    }

    if (expanded) {
    shrink_counter = 0;   // reset shrink logic
    update_y_axis(current_y_min, current_y_max);
    return;
    }

    /* Autoscale - Shrinking Logic */

    int current_range = current_y_max - current_y_min;
    int desired_range = target_max - target_min;

    if (desired_range > 0 &&
        desired_range < (current_range * 6) / 10) {

        shrink_counter++;

        if (shrink_counter >= SHRINK_DELAY_TICKS) {
            current_y_min += (target_min - current_y_min) / 8;
            current_y_max -= (current_y_max - target_max) / 8;
            update_y_axis(current_y_min, current_y_max);
        }
    } else {
        shrink_counter = 0;
    }
}

/**
  * @brief A timer task that displays the "Program Ended Normally" banner when called.
  * @param timer (lv_timer_t) Pointer to the timer object
  */
void ProgramEndedBanner(lv_timer_t *timer) {
    static int ticks = 0;
    
    // First call: show the message
    if(ticks == 0) {
        lv_label_set_text(ui_StopText, "Program ended normally");
        _ui_flag_modify(ui_StopPanel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        lv_label_set_text(ui_StopText2, "Program ended normally");
        _ui_flag_modify(ui_StopPanel2, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
    }

    ticks++;

    // Each tick = 50ms; after 5000ms = 100 ticks, exit
    if(ticks >= 100) {
        lv_timer_del(timer);
        exit(0);
    }
}

/**
  * @brief A timer task that updates the Y axes of the graph when called.
  * @param timer (lv_timer_t) Pointer to the timer object
  */
void ChartUpdateTask(lv_timer_t* timer) {
    if (chart_needs_resize) {
        chart_needs_resize = false;
        lv_chart_set_range(ui_Chart, LV_CHART_AXIS_PRIMARY_Y, current_y_min, current_y_max);
    }
}

/**
  * @brief A timer task that exits the program when called.
  * @param t (lv_timer_t) Pointer to the timer object
  */
void ExitProgram(lv_timer_t * t) { exit(0); }

/**
  * @brief A timer task that displays the Program ended banner.
  * @param t (lv_timer_t) Pointer to the timer object
  */
void ProgramEndedBannerTask(lv_timer_t * t) {

    if (_stopflag == 0 && program_ended_normally_flag) {
        lv_label_set_text(ui_StopText, "Program ended normally");
        lv_obj_clear_flag(ui_StopPanel, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(ui_StopText2, "Program ended normally");
        lv_obj_clear_flag(ui_StopPanel2, LV_OBJ_FLAG_HIDDEN);
    }

}

/**
  * @brief A timer function that constantly makes sure the print lines
  * are being updated.
  * @param t (lv_timer_t) Pointer to the timer object
  */
void PrintUpdateTask(lv_timer_t * t) {
    if (!print_panel_visible) return;

    lv_obj_t* lines[8] = {
        ui_PrintLine1, ui_PrintLine2, ui_PrintLine3, ui_PrintLine4,
        ui_PrintLine5, ui_PrintLine6, ui_PrintLine7, ui_PrintLine8
    };

    for (int i = 0; i < 8; i++) {
        if (print_dirty[i]) {
            lv_label_set_text(lines[i], print_buffers[i]);
            print_dirty[i] = false;
        }
    }
}

/**
  * @brief A timer task that checks if the stop button has been pressed, and if so,
  * displays the "STOP BUTTON PRESSED!" banner and ends the program.
  * @param t (lv_timer_t) Pointer to the timer object
  */
void StopButtonTask(lv_timer_t *t) {
     static bool handled = false;

    if (stop_requested && !handled) {
        handled = true;

        lv_label_set_text(ui_StopText, "STOP BUTTON PRESSED!");
        lv_label_set_text(ui_StopText2, "STOP BUTTON PRESSED!");
        lv_obj_clear_flag(ui_StopPanel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_StopPanel2, LV_OBJ_FLAG_HIDDEN);

        endOfProgram();
    }
}

/**
  * @brief A function that generates a unique file path for data logging.
  * @param name (const char*) The user-defined name for the log file
  * @param out_path (char*) Buffer to store the generated unique file path
  */
void GenerateUniquePath(const char* name, char* out_path) {
    
    int attempt = 0;
    while (true) {
        if (attempt == 0) {
            snprintf(out_path, 128, "/usd/%s.csv", name);
        } else {
            snprintf(out_path, 128, "/usd/%s_%d.csv", name, attempt);
        }

        // Try to open the file just to see if it exists
        FILE* test_file = fopen(out_path, "r");
        if (test_file == NULL) {
            // Success! The file does NOT exist, so we can use this path.
            break; 
        } else {
            // File exists, close it and try the next number
            fclose(test_file);
        }
        attempt++;
    }
}


/**
  * @brief A function that registers a variable for plotting and data logging, 
  * or updates its active status if it's already registered.
  * @param name (const char*) The user-defined name for the log file
  * @param var_ptr (int*) Pointer to the variable that holds the value to be logged
  * @param active (bool) Whether this variable should be actively logged
  * @param slot_index (int) The plot slot index associated with this variable (0 to 
  * MAX_PLOT_SLOTS - 1)
  */
void RegisterPlot(const char* name, int* var_ptr, bool active, int slot_index) {
    if (active) {
        for (int i = 0; i < session_plot_count; i++) {
            if (session_plots[i].slot_index == slot_index) {
                session_plots[i].currently_active = false;
            }
        }
    }

    // 1. Check if this name is already in our session registry
    for (int i = 0; i < session_plot_count; i++) {
        if (strcmp(session_plots[i].name, name) == 0) {
            // It's already registered! Just update the active status and leave.
            session_plots[i].currently_active = active;
            return;
        }
    }

    // 2. If it's a brand new name, add it to the list
    if (session_plot_count < MAX_CUSTOM_PLOTS) {
        strncpy(session_plots[session_plot_count].name, name, MAX_NAME_LEN - 1);
        session_plots[session_plot_count].var_ptr = var_ptr;
        session_plots[session_plot_count].currently_active = active;
        session_plot_count++;
    }
}

/**
  * @brief A function that starts data logging to a CSV file on the SD card.
  * @param name (const char*) The user-defined name for the log file
  */
void StartDataLogging(const char* name) {
    if (is_logging || temp_log_file != NULL) return;
 
    // Reset session tracking
    session_plot_count = 0;
    
    GenerateUniquePath(name, current_filename);
    
    // Open a hidden temp file for raw data
    if (usd_is_installed()) {
        temp_log_file = fopen("/usd/temp_raw.txt", "w");
    }

    log_start_timestamp = millis(); 
    is_logging = true;

    if (logging_timer == NULL) {
        logging_timer = lv_timer_create(SDLoggerTimer, log_rate, NULL);
    }

    pause_active = false; // Ensure we don't start paused
}

/**
  * @brief A function that stops data logging.
  * @param none
  */
void StopDataLogging() {
    if (!is_logging) return;
    is_logging = false;

    // Wait for the logger task to finish its last write
    while(logger_is_busy) delay(10);
    
    delay(20); // Ensure file buffer is flushed

    if (temp_log_file != NULL) {        
        fflush(temp_log_file);  // Force flush all buffered data             
        fclose(temp_log_file);  // Retry once            
        temp_log_file = NULL;
    }

    // 1. Create the final CSV
    FILE* final_csv = fopen(current_filename, "w");
    if (final_csv == NULL) return;

    // 2. Write the dynamic Header
    fprintf(final_csv, "Time,Left Enc,Right Enc,Arm Enc,Sonar,Left Light,Mid Light,Right Light");
    for (int i = 0; i < session_plot_count; i++) {
        fprintf(final_csv, ",%s", session_plots[i].name);
    }
    fprintf(final_csv, "\n");

    // 3. Merge raw data into the final file
    FILE* temp_read = fopen("/usd/temp_raw.txt", "r");
    if (temp_read != NULL) {
        char buffer[1024];
        while (fgets(buffer, sizeof(buffer), temp_read)) {
            fputs(buffer, final_csv);
        }
        fclose(temp_read);
        temp_read = NULL;
    }

    fclose(final_csv);
    final_csv = NULL;

    delay(50); 
    remove("/usd/temp_raw.txt"); // Clean up
}

/**
  * @brief An LVGL timer that runs in the background to log data to the SD card at regular intervals.
  * @param none
  */
void SDLoggerTimer(lv_timer_t * timer) {
    if (!is_logging || pause_active || temp_log_file == NULL) {
        return; 
    }

    logger_is_busy = true;
    uint32_t raw_elapsed = millis() - log_start_timestamp; 
    uint32_t session_time = (raw_elapsed / log_rate) * log_rate;

    // 2. Standard Sensors
    fprintf(temp_log_file, "%u,%d,%d,%d,%d,%d,%d,%d",
            session_time,
            readSensor(LeftEncoder), readSensor(RightEncoder), readSensor(ArmEncoder),
            readSensor(SonarSensor), readSensor(LeftLight), readSensor(MidLight), 
            readSensor(RightLight));

    // 3. Dynamic Columns
    for (int i = 0; i < session_plot_count; i++) {
        if (session_plots[i].currently_active && session_plots[i].var_ptr != NULL) {
            fprintf(temp_log_file, ",%d", *session_plots[i].var_ptr);
        } else {
            fprintf(temp_log_file, ", "); 
        }
    }
    
    fprintf(temp_log_file, "\n");
    fflush(temp_log_file);

    logger_is_busy = false;
}

/**
  * @brief A function that pauses the program and enables variable adjustment when called.
  * @param t (lv_timer_t) Pointer to the timer object
  */
void PauseTask(lv_timer_t * t) {

    if (_stopflag == 1) {
        lv_timer_del(t); 
        lv_obj_add_flag(ui_ResumeButton, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if(pause_active) {
        lv_label_set_text(ui_StopText, "Program Paused");
        lv_obj_clear_flag(ui_StopPanel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_ResumeButton, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_state(ui_PlottingRateDropdown, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_PlotLeftEncCheckbox, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_PlotRightEncCheckbox, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_PlotArmEncCheckbox, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_PlotSonarCheckbox, LV_STATE_DISABLED);

        // --- Conditional Slot Enabling ---
        // Only enable the adjust button if the slot actually has a variable assigned
        if (variable_slots[0].active) lv_obj_clear_state(ui_Slot1Button, LV_STATE_DISABLED);
        else lv_obj_add_state(ui_Slot1Button, LV_STATE_DISABLED);

        if (variable_slots[1].active) lv_obj_clear_state(ui_Slot2Button, LV_STATE_DISABLED);
        else lv_obj_add_state(ui_Slot2Button, LV_STATE_DISABLED);

        if (variable_slots[2].active) lv_obj_clear_state(ui_Slot3Button, LV_STATE_DISABLED);
        else lv_obj_add_state(ui_Slot3Button, LV_STATE_DISABLED);

    } else if (!pause_active && !program_ended_normally_flag) {
        lv_obj_add_flag(ui_StopPanel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_ResumeButton, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_state(ui_PlottingRateDropdown, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot1Button, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot2Button, LV_STATE_DISABLED);
        lv_obj_add_state(ui_Slot3Button, LV_STATE_DISABLED);
        lv_obj_add_state(ui_PlotLeftEncCheckbox, LV_STATE_DISABLED);
        lv_obj_add_state(ui_PlotRightEncCheckbox, LV_STATE_DISABLED);
        lv_obj_add_state(ui_PlotArmEncCheckbox, LV_STATE_DISABLED);
        lv_obj_add_state(ui_PlotSonarCheckbox, LV_STATE_DISABLED);
    }
 }
