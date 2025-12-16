#ifndef BACKGROUND_TASKS_H
#define BACKGROUND_TASKS_H

#include "main.h"
#include "pros/adi.h"
#include "pros/distance.h"

// Function Declarations
void monitorMotorPower(void *param);
void checkSensors(void *param);
void motorStopAll();

// Global Variable Declarations
extern volatile bool stop_requested;

#endif // BACKGROUND_TASKS_H