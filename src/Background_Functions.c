/**
 * @file Background_Functions.c
 * @author Hazim Namik
 * @brief This file provides functions that will make controlling the VEX robot easier across different platforms (e.g., in the lab and using the simulator).
 * @version 0.1
 * @date 2023-02-16
 *
 * @copyright Copyright (c) 2023
 *
 */

 #include <string.h>
 #include <sys/stat.h>
 #include "main.h"
 #include "pros/adi.h"
 #include "pros/distance.h"
 #include "../include/ui.h"
 #include "Student_Code.h"
 
 adi_ultrasonic_t sonar;
 bool Ultra_Init = false;
 long T1_timer = 0;
 long T2_timer = 0;
 long T3_timer = 0;
 long T4_timer = 0;
 bool chart_needs_resize = false;
 int plot_divider = 0;
 int observed_min = INT32_MAX;
 int observed_max = INT32_MIN;
 int shrink_counter = 0;
 bool program_ended_normally_flag = false;

 // State variables for data logging
 bool is_logging = false;
 bool logger_is_busy = false;
 uint32_t log_start_timestamp = 0;
 int log_rate = 100; // default 100ms sampling rate
 char current_filename[128];
 plot_registry_t session_plots[MAX_CUSTOM_PLOTS];
 int session_plot_count = 0;
 FILE* temp_log_file = NULL;
 lv_timer_t *logging_timer = NULL;



 #define SHRINK_DELAY_TICKS 50   // ~1 second if timer = 50ms
 #define CHART_GROW_STEP 50
 
 // __[ GET MOTOR POWER ]________________________________________________
 /**
  * @brief Reads the voltage being sent to the motor.
  * @param motorName Options include: LeftMotor, RightMotor, or ArmMotor
  * @return motor voltage in mV as an integer.
  */
 int getMotorPower(int motorName)
 {
 
     int motorOutput;
 
     // select motor to read
     switch (motorName)
     {
     case 0: // read right wheel motor power
         motorOutput = motor_get_voltage(_motorRight);
         break;
     case 1: // read left wheel motor power
         motorOutput = motor_get_voltage(_motorLeft);
         break;
     case 2: // read robot arm motor power
         motorOutput = motor_get_voltage(_motorArm);
         break;
     }
     return motorOutput;
 }
 
 // __[ READ SENSOR ]____________________________________________________
 /**
  * @brief Reads the output of the selected sensor.
  * @param sensorName Desired sensor. Options include: LeftEncoder, RightEncoder, ArmEncoder, LeftLight, MidLight, RightLight, SonarSensor, StopButton, LowArmLimit, HighArmLimit.
  * @return iOutput of the selected sensor as an integer.
  */
 int readSensor(int sensorName)
 {
 
     int sensorOutput, currentSonar_cm, currentSonar_mm, prevSonar;
 
     // initialise sonar sensor if not already being read
     if (!Ultra_Init)
     {
         sonar = adi_ultrasonic_init(_sonarPing, _sonarEcho);
         Ultra_Init = true;
         delay(100);	// required to give sensor time to initialise and stabilise.
     }
 
     // select sensor to read
     switch (sensorName)
     {
     case 0: // middle light sensor
         sensorOutput = adi_analog_read(_lightMid);
         break;
     case 1: // left light sensor
         sensorOutput = adi_analog_read(_lightLeft);
         break;
     case 2: // right light sensor
         sensorOutput = adi_analog_read(_lightRight);
         break;
     case 4: // stop button
         sensorOutput = adi_digital_read(_buttonStop);
         break;
     case 5: // right wheel encoder
         sensorOutput = motor_get_position(_encoderRight);
         break;
     case 6: // left wheel encoder
         sensorOutput = motor_get_position(_encoderLeft);
         break;
     case 7: // lower limit switch of robot arm
         sensorOutput = adi_digital_read(_armLimitLow);
         break;
     case 8: // upper limit switch of robot arm
         sensorOutput = adi_digital_read(_armLimitHigh);
         break;
     case 9: // robot arm encoder
         sensorOutput = motor_get_position(_motorArm);
         break;
     case 10: // sonar sensor
         currentSonar_cm = adi_ultrasonic_get(sonar);
         currentSonar_mm = currentSonar_cm * 1; // sonar is actually reading in mm!
         if (prevSonar == 0 && currentSonar_mm == 0)
         {
             sensorOutput = -1;
         }
         else if (prevSonar != 0 && currentSonar_mm == 0)
         {
             sensorOutput = prevSonar;
             prevSonar = 0;
         }
         else
         {
             sensorOutput = currentSonar_mm;
             prevSonar = sensorOutput;
         }
         break;
     case 11: // left distance sensor
        sensorOutput = filteredDistanceLeft;
         break;
     case 12: // right distance sensor
         sensorOutput = filteredDistanceRight;
         break;
    //  case 13: // unfiltered left distance sensor
    //      sensorOutput = distance_get(_distanceLeft);
    //      break;
     }
     return sensorOutput;
 }
 
 // __[ READ TIMER ]_____________________________________________________
 /**
  * @brief Reads the elapsed time of a selected timer.
  * @param timerSelect Available timers: T_1, T_2, T_3, or T_4
  * @return int Elapsed time of timer in milliseconds.
  */
 int readTimer(int timerSelect)
 {
 
     int timerOutput;
 
     // select timer to read
     switch (timerSelect)
     {
     case 0: // timer #1
         timerOutput = millis() - T1_timer;
         break;
     case 1: // timer #2
         timerOutput = millis() - T2_timer;
         break;
     case 2: // timer #3
         timerOutput = millis() - T3_timer;
         break;
     case 3: // timer #4
         timerOutput = millis() - T4_timer;
         break;
     }
     return timerOutput;
 }
 
 // __[ ARM DOWN ]_______________________________________________________
 /**
  * @brief Raises the arm until the upper lower switch is activated. CANNOT be stopped before it reaches the bottom position.
  * @param voltage Integer value for the desired motor voltage in mV.
  */
 void armDown(int voltage)
 {
 
     int armMin = adi_digital_read(_armLimitLow);
 
     // constrain motor power to safe operating range
     int powerOutput = (-1) * saturate(abs(voltage), 0, MOTOR_CEILING);
 
     // lower arm until lower limit switch is reached
     while (armMin == 0)
     {
         motorPower(ArmMotor, powerOutput);
         armMin = adi_digital_read(_armLimitLow);
         delay(20);
     }
 
     // stop arm motor
     motorPower(ArmMotor, 0);
 }
 
 // __[ ARM UP ]_________________________________________________________
 /**
  * @brief Raises the arm until the upper limit switch is activated. CANNOT be stopped before it reaches the top position.
  * @param voltage Integer value for the desired motor voltage in mV.
  */
 void armUp(int voltage)
 {
 
     int armMax = adi_digital_read(_armLimitHigh);
 
     // constrain motor power to safe operating range
     int powerOutput = (1) * saturate(abs(voltage), 0, MOTOR_CEILING);
 
     // raise arm until upper limit switch is reached
     while (armMax == 0)
     {
         motorPower(ArmMotor, powerOutput);
         armMax = adi_digital_read(_armLimitHigh);
         delay(20);
     }
 
     // stop arm motor
     motorPower(ArmMotor, 0);
 }
 
 //------------------------------------ motorPower --------------------------------------
 /**
  * @brief Sets the power of a selected motor to a desired level.
  * @param motorName LeftMotor, RightMotor, or ArmMotor
  * @param voltage Integer value for the desired motor voltage in mV.
  */
 void motorPower(int motorName, int voltage)
 {
 
     if (_stopflag) // if stop button has been pressed, don't allow any motorPower command to send power to any motor
     {
         voltage = 0;
     }
 
     // constrain motor power to safe operating range
     int powerOutput = (int)saturate((double)voltage, MOTOR_FLOOR, MOTOR_CEILING);
 
     // select motor for actuation
     switch (motorName)
     {
     case 0: // right wheel motor (+ive goes forwards)
         powerOutput = (1) * powerOutput;
         motor_move_voltage(_motorRight, powerOutput);
         break;
     case 1: // left wheel motor (+ive goes forwards)
         powerOutput = (1) * powerOutput;
         motor_move_voltage(_motorLeft, powerOutput);
         break;
     case 2: // robot arm motor (+ive raises arm)
         powerOutput = (1) * powerOutput;
         if (adi_digital_read(_armLimitLow) && powerOutput < 0)
         {
             motor_move_voltage(_motorArm, 0);
         }
         else if (adi_digital_read(_armLimitHigh) && powerOutput > 0)
         {
             motor_move_voltage(_motorArm, 0);
         }
         else
         {
             motor_move_voltage(_motorArm, powerOutput);
         }
         break;
     }
 }
 
 // __[ RESET SENSOR ]___________________________________________________
 /**
  * @brief Resets the counts of a selected encoder to zero.
  * @param encoderName Options include: RightEncoder, LeftEncoder, or ArmEncoder
  */
 void resetEncoder(int encoderName)
 {
 
     switch (encoderName)
     {
     case 5: // right wheel encoder
         motor_tare_position(_encoderRight);
         break;
     case 6: // left wheel encoder
         motor_tare_position(_encoderLeft);
         break;
     case 9: // robot arm encoder
         motor_tare_position(_encoderArm);
         break;
     }
 }
 
 // __[ RESET TIMER ]____________________________________________________
 /**
  * @brief Resets the elapsed time of a selected timer to zero.
  * @param timerSelect Options include: T_1, T_2, T_3, or T_4
  */
 void resetTimer(int timerSelect)
 {
 
     switch (timerSelect)
     {
     case 0: // timer #1
         T1_timer = millis();
         break;
     case 1: // timer #2
         T2_timer = millis();
         break;
     case 2: // timer #3
         T3_timer = millis();
         break;
     case 3: // timer #4
         T4_timer = millis();
         break;
     }
 }
 
 /// @brief Call after the student main code to stop all motors and prevent future motor usage.
 void endOfProgram()
 {
     motor_move(_motorLeft, 0);
     motor_move(_motorRight, 0);
     motor_move(_motorArm, 0);
 
    if(graph_timer) {
        lv_timer_del(graph_timer);
        graph_timer = NULL;
    }

    if (is_logging) StopDataLogging();

    ResetVariableSlots();

    program_ended_normally_flag = true;

    lv_timer_t * t = lv_timer_create(ExitProgram, 5000, NULL);
    lv_timer_set_repeat_count(t, 1);
 }
 
 
 // ----------------------------------- General functions ----------------------------------------
 
 /// @brief Returns the lowest of two input numbers
 /// @param num1 First input number to compare
 /// @param num2 Second input number to compare
 /// @return The lower of the two inputs
 double min(double num1, double num2)
 {
     if (num1 < num2)
     {
         return num1;
     }
     else
     {
         return num2;
     }
 }
 
 
 /// @brief Returns the highest of two input numbers
 /// @param num1 First input number to compare
 /// @param num2 Second input number to compare
 /// @return The higher of the two inputs
 double max(double num1, double num2)
 {
     if (num1 > num2)
     {
         return num1;
     }
     else
     {
         return num2;
     }
 }
 
 /// @brief Implementation of the signum function. Returns 1 if the input number is +ve, -1 if negative, and 0 if it's 0.
 /// @param input Input number
 /// @return Sign of the input number (1 if positive, -1 if negative, and 0 if input is 0)
 int sgn(double input)
 {
     if (input == 0)
     {
         return 0;
     }
     else if (input > 0)
     {
         return 1;
     }
     else
     {
         return -1;
     }
 }
 
 // __[ SATURATE ]_______________________________________________________
 /**
  * @brief Constrains an input value between the lower and upper input limits. If the input number is below the lower limit, the function will return the lower limit.
  * Similarly, if the input is higher than the upper limit, the function will return the upper limit. If the input number is between the two limits, the function will
  * return the input number unmodified.
  * @param input (double) Input number
  * @param lower (double) Lower limit of desired range
  * @param upper (double) Upper limit of desired range
  * @return (double) The saturated number guaranteed to be between the lower and upper inputs.
  */
 double saturate(double input, double lower, double upper)
 {
 
     if (input > upper)
     {
         return upper; // limit by upper bound
     }
     else if (input < lower)
     {
         return lower; // limit by lower bound
     }
     else
     {
         return input; // else leave unadjusted
     }
 }
 
 // ----------------------------------- New Functions - UI Updates -----------------------------------
 
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
        case PLOT_LEFT_DIST:   return readSensor(LeftDistance);
        case PLOT_RIGHT_DIST:  return readSensor(RightDistance);
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
        case PLOT_LEFT_DIST:   return plot_left_dist_enabled;
        case PLOT_RIGHT_DIST:  return plot_right_dist_enabled;
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
        case PLOT_LEFT_DIST:   return "Left Distance";
        case PLOT_RIGHT_DIST:  return "Right Distance";
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
        PLOT_LEFT_DIST,
        PLOT_RIGHT_DIST
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

        /* SYNC WITH LOGGER:
           Whenever we encounter a custom plot slot, we update the session registry.
           If 'active' is true, the logger records the value. 
           If 'active' is false, the logger records a placeholder '0'. */
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

    // Copy the string safely into our buffer
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
    fprintf(final_csv, "Time,Left Enc,Right Enc,Arm Enc,Left Dist,Right Dist,Left Light,Mid Light,Right Light");
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
  * @brief A task that runs in the background to log data to the SD card at regular intervals.
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
    fprintf(temp_log_file, "%u,%d,%d,%d,%d,%d,%d,%d,%d",
            session_time,
            readSensor(LeftEncoder), readSensor(RightEncoder), readSensor(ArmEncoder),
            readSensor(LeftDistance), readSensor(RightDistance),
            readSensor(LeftLight), readSensor(MidLight), readSensor(RightLight));

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
  * @brief A function that sets the data logging rate.
  * @param ms (int) Desired logging rate in milliseconds (minimum 50ms)
  */
void SetLogRate(int ms) {
    if (ms < 50) ms = 50;
    
    log_rate = ms;
}


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
        lv_obj_clear_state(ui_Slot1Button, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_Slot2Button, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_Slot3Button, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_PlotLeftEncCheckbox, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_PlotRightEncCheckbox, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_PlotArmEncCheckbox, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_PlotLeftDistanceCheckbox, LV_STATE_DISABLED);
        lv_obj_clear_state(ui_PlotRightDistanceCheckbox, LV_STATE_DISABLED);
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
        lv_obj_add_state(ui_PlotLeftDistanceCheckbox, LV_STATE_DISABLED);
        lv_obj_add_state(ui_PlotRightDistanceCheckbox, LV_STATE_DISABLED);

    }
 }