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
 // New functions/global variables for 2026
 void GraphUpdateTask(lv_timer_t * timer);
 void ProgramEndedBanner(lv_timer_t *timer);
 void ChartUpdateTask(lv_timer_t* timer);
 extern void ExitProgram(lv_timer_t * t);
 extern bool chart_needs_resize;
 void PrintUpdateTask(lv_timer_t * t);
 void StopButtonTask(lv_timer_t *t);
 void ProgramEndedBannerTask(lv_timer_t * t);
 void PauseTask(lv_timer_t * t);

 /* Function declarations from Background_Tasks.c */
 void monitorMotorPower(void* param);
 void motorStopAll(void);
 void checkSensors(void* param);

 /* Global variables */
 extern int _stopflag;              //1 || 0 - Used to control whether or not robot is to be stopped or in stop mode.
 extern int _arm_State;             // Variable that defines the state of the robot arm (-1 at lower limit, 1 at upper limit, and 0 in between)
 extern bool program_ended_normally_flag;
 extern volatile bool stop_requested;

 /* Declarations for plotting */
 typedef enum {
    PLOT_NONE = 0,
    PLOT_LEFT_ENC,
    PLOT_RIGHT_ENC,
    PLOT_ARM_ENC,
    PLOT_SONAR,
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

 int GetPlotValue(int slot_index);
 bool IsSourceEnabled(plot_source_t src);
 void UpdatePlotSlots(void);
 char *PlotSourceName(plot_source_t src);
 void CustomPlot(int slot, int value, const char * name);

/* Declarations for Data Logging */

// structure to hold all custom plot info
typedef struct {
    char name[MAX_NAME_LEN];
    int* var_ptr;
    bool currently_active;
    int slot_index;
} plot_registry_t;

//Global variables for data logging
extern bool is_logging;
extern bool logger_is_busy;
extern uint32_t log_start_timestamp;
extern int log_rate;
extern char current_filename[128];
extern plot_registry_t session_plots[MAX_CUSTOM_PLOTS];
extern int session_plot_count;
extern FILE *temp_log_file;
extern lv_timer_t *logging_timer;

// Function Prototypes
extern void StartDataLogging(const char* name);
extern void StopDataLogging();
extern void SDLoggerTimer(lv_timer_t * t);
extern void RegisterPlot(const char* name, int* var_ptr, bool active, int slot_index);
extern void GenerateUniquePath(const char* name, char* out_path);

 #endif  // _PROS_MAIN_H_
