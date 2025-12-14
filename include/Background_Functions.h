#ifndef BACKGROUND_FUNCTIONS_H
#define BACKGROUND_FUNCTIONS_H

#include "main.h"
#include "pros/adi.h"
#include "pros/distance.h"
#include "liblvgl/lvgl.h"

// Declare your functions
int getMotorPower(int motorName);
int readSensor(int sensorName);
int readTimer(int timerSelect);
void armDown(int voltage);
void armUp(int voltage);
void motorPower(int motorName, int voltage);
void resetEncoder(int encoderName);
void resetTimer(int timerSelect);
void endOfProgram(void);

// New functions for SRS
void graph_update_task(lv_timer_t * timer);
void program_ended_banner(lv_timer_t *timer);
void chart_update_task(lv_timer_t* timer);
extern void exit_program(lv_timer_t * t);
extern bool chart_needs_resize;
double lowPassFilter(double newReading, bool left);

#endif
