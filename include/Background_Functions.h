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

// SRS UI graph task
void graph_update_task(lv_timer_t * timer);

#endif