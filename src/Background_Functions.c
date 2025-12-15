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

 #include "main.h"
 #include "pros/adi.h"
 #include "pros/distance.h"
 #include "../include/ui.h"
 #include "../include/Background_Functions.h"
 #include "Controller_Telemetry.h"
 #include "ui.h"
 
 adi_ultrasonic_t sonar;
 bool Ultra_Init = false;
 long T1_timer = 0;
 long T2_timer = 0;
 long T3_timer = 0;
 long T4_timer = 0;
 bool chart_needs_resize = false;
 double filteredDistanceLeft = 0;
 double filteredDistanceRight = 0;
 bool leftInitialised = false;
 bool rightInitialised = false;
 int plot_divider = 0;
 int observed_min = INT32_MAX;
 int observed_max = INT32_MIN;

int shrink_counter = 0;

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
        // This sensor constantly reads 14mm less than the other sensor for some reason
        sensorOutput = lowPassFilter(distance_get(_distanceLeft), true);
         break;
     case 12: // right distance sensor
         sensorOutput = lowPassFilter(distance_get(_distanceRight), false);
         break;
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
    lv_timer_t * t = lv_timer_create(exit_program, 5000, NULL);
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
  * @brief A timer task that updates the data being graphed on the main screen chart, and automatically
  * scales the Y axis according to the values being plotted.
  * @param timer (lv_timer_t) Pointer to the timer object
  */
 void graph_update_task(lv_timer_t * timer) {

    plot_divider++;
    if (plot_divider < 2) { return; } //Plot every 4 points

    plot_divider = 0;

    int u_val, e_val, wheel_enc_val, arm_enc_val, left_dist_val, right_dist_val;

    int local_min = INT32_MAX;
    int local_max = INT32_MIN;

    // Ensure chart and series exist
    if (!ui_Chart) return;
    if (!series_U && !series_E && !series_WheelEnc && !series_ArmEnc && !series_LeftDist && !series_RightDist) return;

    /* Plotting Logic */

    if (series_U && lv_obj_has_state(ui_PlotUCheckbox, LV_STATE_CHECKED)) {
        u_val = controller_sample.control_effort; 
        lv_chart_set_next_value(ui_Chart, series_U, u_val);
        chart_needs_resize = true;
        local_min = min(local_min, u_val);
        local_max = max(local_max, u_val);
    }

    if (series_E && lv_obj_has_state(ui_PlotECheckbox, LV_STATE_CHECKED)) {
        e_val = controller_sample.error;
        lv_chart_set_next_value(ui_Chart, series_E, e_val);
        chart_needs_resize = true;
        local_min = min(local_min, e_val);
        local_max = max(local_max, e_val);
    }

    if (series_WheelEnc && lv_obj_has_state(ui_PlotWheelEncCheckbox, LV_STATE_CHECKED)) {
        wheel_enc_val = 0.5 * (readSensor(LeftEncoder) + readSensor(RightEncoder));
        lv_chart_set_next_value(ui_Chart, series_WheelEnc, wheel_enc_val);
        chart_needs_resize = true;
        local_min = min(local_min, wheel_enc_val);
        local_max = max(local_max, wheel_enc_val);
    }

    if (series_ArmEnc && lv_obj_has_state(ui_PlotArmEncCheckbox, LV_STATE_CHECKED)) {
        arm_enc_val = readSensor(ArmEncoder);
        lv_chart_set_next_value(ui_Chart, series_ArmEnc, arm_enc_val);
        chart_needs_resize = true;
        local_min = min(local_min, arm_enc_val);
        local_max = max(local_max, arm_enc_val);
    }

    if (series_LeftDist && lv_obj_has_state(ui_PlotLeftDistanceCheckbox, LV_STATE_CHECKED)) {
        left_dist_val = readSensor(LeftDistance);
        lv_chart_set_next_value(ui_Chart, series_LeftDist, left_dist_val);
        chart_needs_resize = true;
        local_min = min(local_min, left_dist_val);
        local_max = max(local_max, left_dist_val);
    }

    if (series_RightDist && lv_obj_has_state(ui_PlotRightDistanceCheckbox, LV_STATE_CHECKED)) {
        right_dist_val = readSensor(RightDistance);
        lv_chart_set_next_value(ui_Chart, series_RightDist, right_dist_val);
        chart_needs_resize = true;
        local_min = min(local_min, right_dist_val);
        local_max = max(local_max, right_dist_val);
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
void program_ended_banner(lv_timer_t *timer) {
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
void chart_update_task(lv_timer_t* timer) {
    if (chart_needs_resize) {
        chart_needs_resize = false;
        lv_chart_set_range(ui_Chart, LV_CHART_AXIS_PRIMARY_Y, current_y_min, current_y_max);
    }
}

/**
  * @brief A Low Pass Filter that is designed to reduce fluctuation in the outputs 
  * of each distance sensor.  It does this using a discrete, first order LPF algorithm.
  * @param newReading (double): the most recent reading from the distance sensor.
  * @param left (bool): true or false corresponds to left or right distance sensor.
  */
double lowPassFilter(double newReading, bool left) {
    double alpha = 0.2; // Smoothing factor (0 < alpha <= 1)
    if (left) {
        if (!leftInitialised) {
            filteredDistanceLeft = newReading;
            leftInitialised = true;
        } else {
            filteredDistanceLeft =
                alpha * newReading + (1 - alpha) * filteredDistanceLeft;
        }
        return filteredDistanceLeft;
    } else {
        if (!rightInitialised) {
            filteredDistanceRight = newReading;
            rightInitialised = true;
        } else {
            filteredDistanceRight =
                alpha * newReading + (1 - alpha) * filteredDistanceRight;
        }
        return filteredDistanceRight;
    }
}
/**
  * @brief A timer task that exits the program when called.
  * @param timer (lv_timer_t) Pointer to the timer object
  */
void exit_program(lv_timer_t * t) { exit(0); }

/**
  * @brief A function that resets .
  * @param timer (lv_timer_t) Pointer to the timer object
  */
void resetDistance(int sensor_name) {
    if (sensor_name == LeftDistance) {
        leftInitialised = false;
    } else if (sensor_name == RightDistance) {
        rightInitialised = false;
    }
}