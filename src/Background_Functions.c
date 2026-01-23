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
 logger_ctx_t current_log = {0, 0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0}};
 int log_rate = 100;
 FILE *log_file = NULL;
 bool is_logging = false;
 char current_filename[64];
 bool logger_is_busy = false;
 uint32_t log_start_timestamp = 0;

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

    if (_stopflag == 0) {
        lv_label_set_text(ui_StopText, "Program ended normally");
        lv_obj_clear_flag(ui_StopPanel, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(ui_StopText2, "Program ended normally");
        lv_obj_clear_flag(ui_StopPanel2, LV_OBJ_FLAG_HIDDEN);
    }

    if (is_logging) {
        StopDataLogging(); 
    }
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
 
 // ----------------------------------- New Functions - SRS ----------------------------------------
 
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

    /* Remove non-custom sources that are no longer enabled */
    for (int i = 0; i < MAX_PLOT_SLOTS; i++) {
        if (plot_slots[i].active && plot_slots[i].source != PLOT_CUSTOM) {
            if (!IsSourceEnabled(plot_slots[i].source)) {
            clearSeries(plot_slots[i].series);

            /* Hide legend elements */
            lv_obj_add_flag(plot_slots[i].legend_label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(plot_slots[i].legend_colour_box,  LV_OBJ_FLAG_HIDDEN);

            plot_slots[i].active = false;
            plot_slots[i].source = PLOT_NONE;
            }
        }
    }

    /* Assign new sources to free slots */
    for (int r = 0; r < (int)(sizeof(requested)/sizeof(requested[0])); r++) {
        plot_source_t src = requested[r];

        if (!IsSourceEnabled(src))
            continue;

        bool already_assigned = false;

        for (int i = 0; i < MAX_PLOT_SLOTS; i++) {
            if (plot_slots[i].active && plot_slots[i].source == src) {
                already_assigned = true;
                break;
            }
        }

        if (already_assigned)
            continue;

        for (int i = 0; i < MAX_PLOT_SLOTS; i++) {
            if (!plot_slots[i].active) {
                plot_slots[i].source = src;
                plot_slots[i].active = true;

                /* Update legend text */
                lv_label_set_text(plot_slots[i].legend_label, PlotSourceName(src));
                /* Show legend elements */
                lv_obj_clear_flag(plot_slots[i].legend_label, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(plot_slots[i].legend_colour_box,  LV_OBJ_FLAG_HIDDEN);
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

    // Set source to external so UpdatePlotSlots() knows to leave it alone
    plot_slots[index].source = PLOT_CUSTOM;
    plot_slots[index].active = true;
    plot_slots[index].custom_value = value;

    // Copy the string safely into our buffer
    strncpy(plot_slots[index].custom_name, name, 31);
    plot_slots[index].custom_name[31] = '\0'; // Ensure null termination

    // Also update the current_log for logging purposes
    if (index >= 0 && index < 4) {
        current_log.custom[index] = value;
    }

    // Tell the UI task it needs to update the label text
    plot_slots[index].needs_ui_refresh = true;
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
  * @brief A function that resets the Distance sensors.
  * @param sensor_name (int) An integer that represents either the
  * left or right distance sensor.
  */
void ResetDistance(int sensor_name) {
    if (sensor_name == LeftDistance) {
        leftInitialised = false;
    } else if (sensor_name == RightDistance) {
        rightInitialised = false;
    }
}

/**
  * @brief A function that constantly checks the Kp and Ki slider labels, making sure they display 
  * the correct text.  It also checks the sliders themselves, making sure their position correctly
  * reflects the current Kp and Ki values. 
  * @param t (lv_timer_t) Pointer to the timer object
  */
void UpdateGainLabels(lv_timer_t * t)
{
    static char buf[32];

    if (lv_obj_has_flag(ui_NumPad, LV_OBJ_FLAG_HIDDEN)) {
        snprintf(buf, sizeof(buf), "Kp = %.2f", Kp);
        lv_label_set_text(ui_KpLabel, buf);

        snprintf(buf, sizeof(buf), "Ki = %.2f", Ki);
        lv_label_set_text(ui_KiLabel, buf);
    }
}

/**
  * @brief A timer function that constantly makes sure the print lines
  * are being updated.
  * @param t (lv_timer_t) Pointer to the timer object
  */
void PrintUpdateTask(lv_timer_t * t)
{
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
  * @param none
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

void StartDataLogging(const char* name) {
    if (is_logging) return; // Already logging

    // Construct the path: /usd/ + user name + .csv
    snprintf(current_filename, sizeof(current_filename), "/usd/%s.csv", name);

    log_file = fopen(current_filename, "w");
    
    if (log_file != NULL) {

        // Write the Header immediately
        fprintf(log_file, "Time,Left Enc,Right Enc,Arm Enc,Left Dist,Right Dist,Left Light,Mid Light,Right Light,Custom1,Custom2,Custom3,Custom4\n");
        fflush(log_file);

        log_start_timestamp = millis();

        is_logging = true;
    }
}

void StopDataLogging() {
    if (!is_logging) return;
    
    is_logging = false;

    uint32_t timeout = millis();
    while (logger_is_busy && (millis() - timeout < 500)) {
        delay(5); // Give it a few ms to finish
    }
    
    if (log_file != NULL) {
        fclose(log_file);
        log_file = NULL;
    }
}

void SDLoggerTask(void* param) {
    uint32_t start_time = millis();
    uint32_t last_wake_time = start_time;

    while (true) {
        if (is_logging && log_file != NULL) {
            logger_is_busy = true;

            uint32_t current_time = millis();
            uint32_t relative_time = (current_time <= log_start_timestamp) ? 0 : (current_time - log_start_timestamp);

            relative_time = (relative_time / log_rate) * log_rate;

            // 1. Update standard sensors
            current_log.left_enc   = readSensor(LeftEncoder);
            current_log.right_enc  = readSensor(RightEncoder);
            current_log.arm_enc    = readSensor(ArmEncoder);
            current_log.left_dist  = readSensor(LeftDistance);
            current_log.right_dist = readSensor(RightDistance);
            current_log.left_light = readSensor(LeftLight);
            current_log.mid_light  = readSensor(MidLight);
            current_log.right_light = readSensor(RightLight);

            // 2. Write the row
            fprintf(log_file, "%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                    relative_time,
                    current_log.left_enc, current_log.right_enc, current_log.arm_enc,
                    current_log.left_dist, current_log.right_dist,
                    current_log.left_light, current_log.mid_light, current_log.right_light,
                    current_log.custom[0], current_log.custom[1],
                    current_log.custom[2], current_log.custom[3]);
            
            fflush(log_file); 

            // Check if the SD card was pulled out mid-run
            if (!usd_is_installed()) {
                StopDataLogging(); // Emergency stop
                continue;
            }
        logger_is_busy = false;
        }

        task_delay_until(&last_wake_time, log_rate);
    }
}

void SetLogRate(int ms) {
    if (ms < 50) ms = 50;
    
    log_rate = ms;
}