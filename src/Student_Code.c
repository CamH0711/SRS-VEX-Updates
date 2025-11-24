/**
 * @file Student_Code.c
 * @author Cameron Hall
 * @brief Copy of the project provided to all ME201 students, but with updated capabilities 
 *        for the distance sensor.
 * @version 0.1
 * @date 14-05-2025
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Libraries. DO NOT REMOVE */
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Student_Code.h"

#define BLACK_LOWER 1100

// ---------------------- Defining physical robot parameters --------------------------
// Update these numbers to match the physical robot (information found in the lab manual)
int drivingWheelDiameter = 103;	    // diameter of the driving wheels [mm]
int robotWidth = 250;				// width of the robot including the wheel thickness [mm]
int wheelWidth = 22;				// width of the driving wheel [mm]
double drivingWheelRatio = 1;	    // ratio of wheel shaft rotations to wheel motor shaft rotations
double armRatio = 7;				// ratio of arm shaft rotations to arm motor shaft rotations
double encCountPerRev = 900;	    // number of encoder ticks per 1 revolution of the motor shaft
// ------------------------------------------------------------------------------------

/* Write your code in the function below. You may add helper functions below the studentCode function. */
void student_Main()
{  
    int distance_left, distance_right, sonar_distance;

    while(true) {
        distance_left = readSensor(LeftDistance);
        distance_right = readSensor(RightDistance);
        sonar_distance = readSensor(SonarSensor);


        lcd_print(2,"Left Distance = %d mm", distance_left);
        lcd_print(3, "Right Distance = %d mm", distance_right);
        lcd_print(5, "Sonar Distance = %d mm", sonar_distance);

        delay(100);
    }
}

// ----------------------------------------------- Function definitions go here  -----------------------------------------------//
// Don't forget to add your function prototypes to Student_Code.h

