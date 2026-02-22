/**
 * @file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

 #ifndef _PROS_MAIN_H_
 #define _PROS_MAIN_H_
 #define PROS_USE_LITERALS

 // New Macros for 2026
 #define MAX_PLOT_SLOTS 4
 #define MAX_CUSTOM_PLOTS 30
 #define MAX_NAME_LEN 32
 
 #include "api.h"
 #include "pros/distance.h"
 
 /* Pin allocations for motors and sensors */
 #define _encoderLeft    18  //left wheel encoder
 #define _motorLeft      18  //left wheel motor
 #define _encoderRight   19  //right wheel encoder
 #define _motorRight     19  //right wheel motor
 #define _encoderArm     20  //robot arm encoder
 #define _motorArm       20  //robot arm motor
 #define _sonarPing      'A' //sonar ping
 #define _sonarEcho      'B' //sonar echo
 #define _lightRight     'C' //right light sensor
 #define _lightMid       'D' //middle light sensor
 #define _lightLeft      'E' //left light sensor
 #define _armLimitLow    'F' //lower limit switch of robot arm
 #define _armLimitHigh   'G' //upper limit switch of robot arm
 #define _buttonStop     'H' //stop button
 
 /* Motor macros for student use */
 #define RightMotor      0   //right wheel motor
 #define LeftMotor       1   //left wheel motor
 #define ArmMotor        2   //robot arm motor
 
 /* Sensor macros for student use */
 #define MidLight        0   //middle light sensor
 #define LeftLight       1   //left light sensor
 #define RightLight      2   //right light sensor
 //#define StartButton     3   //(NOT IMPLEMENTED) start button
 #define StopButton      4   //stop button
 #define RightEncoder    5   //right wheel encoder
 #define LeftEncoder     6   //left wheel encoder
 #define LowArmLimit     7   //lower limit switch of robot arm
 #define HighArmLimit    8   //upper limit switch of robot arm
 #define ArmEncoder      9   //robot arm encoder
 #define SonarSensor     10  //sonar sensor
 
 /* Timer initialisation and macros */
 extern long T1_timer;
 extern long T2_timer;
 extern long T3_timer;
 extern long T4_timer;
 #define T_1             0   //timer #1
 #define T_2             1   //timer #2
 #define T_3             2   //timer #3
 #define T_4             3   //timer #4
 
 /* Macros for universal motor limits */
 #define MOTOR_FLOOR     -5000
 #define MOTOR_CEILING   5000
 #define PI 3.14159265359


 void initialize(void);
 
 /* PROS Tasks */
 extern task_t monitorMotors_Task;
 extern task_t check_sensors;

 /* Function declarations from Background_Functions.c */
 double saturate(double input, double lower, double upper);
 int getMotorPower(int motorName);
 int readSensor (int sensorName);
 int readTimer(int timerSelect);
 void armDown(int voltage);
 void armUp(int voltage);
 void motorPower(int motorName, int voltage);
 void resetTimer(int timerSelect);
 void resetEncoder(int encoderName);
 void endOfProgram();
 double min(double num1, double num2);
 double max(double num1, double num2);
 int sgn(double input);

 /* Function declarations from Background_Tasks.c */
 void monitorMotorPower(void* param);
 void motorStopAll(void);
 void checkSensors(void* param);

 /* Global variables */
 extern int _stopflag;              //1 || 0 - Used to control whether or not robot is to be stopped or in stop mode.
 extern int _arm_State;             // Variable that defines the state of the robot arm (-1 at lower limit, 1 at upper limit, and 0 in between)


 /* Custom Structs and Enums*/

 typedef enum {
    PLOT_NONE = 0,
    PLOT_LEFT_ENC,
    PLOT_RIGHT_ENC,
    PLOT_ARM_ENC,
    PLOT_SONAR,
    PLOT_LEFT_LIGHT,
    PLOT_MID_LIGHT,
    PLOT_RIGHT_LIGHT,
    PLOT_CUSTOM
} plot_source_t;

typedef struct {
    lv_chart_series_t *series;
    plot_source_t source;
    bool active;
    lv_obj_t *legend_label;
    lv_obj_t *legend_colour_box;
    int custom_value;
    char custom_name[32];
    bool needs_ui_refresh;
    char last_recorded_name[32];
    int cycles_since_update;
} plot_slot_t;

 extern plot_slot_t plot_slots[MAX_PLOT_SLOTS];
 extern lv_chart_series_t *slot_series[MAX_PLOT_SLOTS];

typedef struct {
    char name[MAX_NAME_LEN];
    int* var_ptr;
    bool currently_active;
    int slot_index;
} plot_registry_t;

typedef enum {
    TYPE_INT,
    TYPE_DOUBLE
} numpad_data_type_t;

typedef struct {
    void *target_value;       // Void pointer can hold int* OR double*
    numpad_data_type_t type;  // Remembers which type it is
    lv_obj_t *target_label;
    char *prefix;
    char buffer[16];

    int max_length;
    int max_decimals;
} numpad_ctx_t;

extern numpad_ctx_t numpad_ctx;

/* Adjustable Slots Related */
typedef struct {
    void *var_ptr;        // Pointer to the student's variable
    numpad_data_type_t type; // Your enum: NUMPAD_TYPE_DOUBLE or NUMPAD_TYPE_INT
    char name[17];          // Name to display (e.g., "Kp", "Arm Target")
    lv_obj_t * slot_label;  // The UI label showing "Name: Value"
    bool active;            // Whether this slot is in use
    int cycles_since_update;
} variable_slot_t;

extern variable_slot_t variable_slots[3];
extern bool numpad_is_open;       // Tracks if the Numpad is on screen
extern void * current_target_ptr;  // Tracks exactly which variable we are editing
extern uint32_t last_config_time;

#define ConfigSlot(slot, var, name) _Generic((var), \
    double*: _InternalConfig, \
    int*:    _InternalConfig    \
)(slot, var, (sizeof(*(var)) == sizeof(double) ? TYPE_DOUBLE : TYPE_INT), name)


 /* Function declarations for Custom_Functions.c*/
 extern void ui_event_Switch(lv_event_t * e);
 extern void ui_event_SettingsButton(lv_event_t * e);
 extern void update_y_axis(int min, int max);
 extern void ScreenPrint(int line_number, char* text, ...);
 extern void ui_event_OnScreenStopButton(lv_event_t * e);
 extern void ToggleChart();
 extern void ui_event_ResumeButton(lv_event_t * e);
 extern void Pause();
 extern void ui_event_PlotLeftEncCheckbox(lv_event_t * e);
 extern void ui_event_PlotRightEncCheckbox(lv_event_t * e);
 extern void ui_event_PlotArmEncCheckbox(lv_event_t * e);
 extern void ui_event_PlotSonarCheckbox(lv_event_t * e);
 extern void PlotSensor(int sensor_name);
 extern void ClearSeriesTimer(lv_timer_t * t);
 extern void ClearSeries(lv_chart_series_t *series);
 extern void NumpadOpen(void *value, numpad_data_type_t type, lv_obj_t *label, const char *prefix, int max_len, int max_dec);
 extern void NumpadButtonEvent(lv_event_t * e);
 extern void UpdateTargetLabel(void);
 extern void ui_event_PlottingRateDropdown(lv_event_t * e);
 extern int FindClosestDropdownIndex(lv_obj_t * dropdown, int target_value);
 extern void SetPlottingRate(int rate_ms);
 extern void _InternalConfig(int slot_num, void * var, numpad_data_type_t type, const char * name);
 void RefreshVariableLabels(lv_timer_t * t);
 void ResetVariableSlots();
 int GetPlotValue(int slot_index);
 bool IsSourceEnabled(plot_source_t src);
 void UpdatePlotSlots(void);
 char *PlotSourceName(plot_source_t src);
 void CustomPlot(int slot, int value, const char * name);
 extern void StartDataLogging(const char* name);
 extern void StopDataLogging();
 extern void SDLoggerTimer(lv_timer_t * t);
 extern void RegisterPlot(const char* name, int* var_ptr, bool active, int slot_index);
 extern void GenerateUniquePath(const char* name, char* out_path);
 extern void ui_event_BackToMainButton(lv_event_t * e);
 extern void ui_event_Slot1Button(lv_event_t * e);
 extern void ui_event_Slot2Button(lv_event_t * e);
 extern void ui_event_Slot3Button(lv_event_t * e);
 extern void ui_event_ButtonDone(lv_event_t * e);
 void GraphUpdateTask(lv_timer_t * timer);
 void ProgramEndedBanner(lv_timer_t *timer);
 void ChartUpdateTask(lv_timer_t* timer);
 extern void ExitProgram(lv_timer_t * t);
 extern bool chart_needs_resize;
 void PrintUpdateTask(lv_timer_t * t);
 void StopButtonTask(lv_timer_t *t);
 void ProgramEndedBannerTask(lv_timer_t * t);
 void PauseTask(lv_timer_t * t);

 /* Global Variable declarations for Custom_Functions.c*/
 extern volatile bool print_panel_visible;
 extern char print_buffers[8][51];
 extern bool print_dirty[8];
 extern volatile bool pause_active;
 extern int current_y_min;
 extern int current_y_max;
 extern bool plot_left_enc_enabled;
 extern bool plot_right_enc_enabled;
 extern bool plot_arm_enabled;
 extern bool plot_sonar_enabled;
 extern bool plot_left_light_enabled;
 extern bool plot_mid_light_enabled;
 extern bool plot_right_light_enabled;
 extern int point_count;
 extern int plotting_rate;
 extern bool is_logging;
 extern bool logger_is_busy;
 extern uint32_t log_start_timestamp;
 extern int log_rate;
 extern char current_filename[128];
 extern plot_registry_t session_plots[MAX_CUSTOM_PLOTS];
 extern int session_plot_count;
 extern FILE *temp_log_file;
 extern lv_timer_t *logging_timer;
extern bool program_ended_normally_flag;
 extern volatile bool stop_requested;

 #endif  // _PROS_MAIN_H_
