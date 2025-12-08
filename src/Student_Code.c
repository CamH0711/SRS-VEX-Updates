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
#include "ui.h"

#define BLACK_LOWER 1100

// ---------------------- Defining physical robot parameters --------------------------
// Update these numbers to match the physical robot (information found in the lab manual)
int drivingWheelDiameter = 103;	    // diameter of the driving wheels [mm]
int robotWidth = 250;				// width of the robot including the wheel thickness [mm]
int wheelWidth = 22;				// width of the driving wheel [mm]
double drivingWheelRatio = 1;	    // ratio of wheel shaft rotations to wheel motor shaft rotations
double armRatio = 7;				// ratio of arm shaft rotations to arm motor shaft rotations
double encCountPerRev = 900;	    // number of encoder ticks per 1 revolution of the motor shaft
//New for Distance Sensor
int sensorWidth = 139;              // Distance between the left and right distance sensor
// ------------------------------------------------------------------------------------

/* Write your code in the function below. You may add helper functions below the studentCode function. */
void student_Main()
{  

// motorPower(ArmMotor, 2000);
// delay(20000);

while (true) {
    lvgl_print(1, "Left Encoder = %d", readSensor(LeftEncoder));
    lvgl_print(2, "Right Encoder = %d", readSensor(RightEncoder));
    delay(50);
}

    // driveStraight(300);

    // driveToObject(400);

    // while (true) {
    //     lcd_print(2, "Distance Sensor = %d mm", readSensor(LeftDistance));
    //     lcd_print(4, "Ultrasonic Range Finder = %d mm", readSensor(SonarSensor));
    //     delay(50);
    // }
}

// ----------------------------------------------- Function definitions go here  -----------------------------------------------//
// Don't forget to add your function prototypes to Student_Code.h

// *** NEW FUNCTIONS FOR SRS PROJECT ***

//A function that calculates the angle at which the object in front of the robot is currently positioned at. Returns a positive 
//angle for an angle of elevation, and negative for an angle of depression, measured from the right corner.
double findObjectAngle() {
    //Find Sensor Distances
    double left_distance = (double) readSensor(LeftDistance); 
    double right_distance = (double) readSensor(RightDistance);
    double elevation = 1.0;

    //Calculate the opposite
    double opposite = left_distance - right_distance;
    if (opposite < 0) {
        opposite *= -1.0;
        elevation = -1.0;
    }
    //Calculate the adjacent
    double adjacent = (double) sensorWidth;

    //Calculate the angle
    if (opposite == 0.0) {
        return 0.0;
    } else {
        return elevation * (180/PI) * atan(opposite/adjacent);
    }
}

void StraightenAngle(double percentPower) {

}


// *** FUNCTIONS COPIED OVER FROM SEM 1 201 PROJECT ***

//A function for driving a specified distance forwards or backwards
void driveStraight(int distance) {

    //Initialise Variables
    int error, errorIntSum = 0, encError;
    int k = 50, errorArray[1000] = {0}; 
    double currentPosition = 0;
    double Kp = 1, Ki = 0.1, Kp_straight = 1.0;
    double u = 100, uL, uR, uDiff;
    double encoderAverage;
    double tolerance = 0.1;
    
    int i;
    
    for (i = 0; i <= 1000; i++) {
        errorArray[i] = 9999;
    }
    
    //Reset Encoders
    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    
    do {
        // ** Drive Controller (PI Controller) **
        //Use encoders to Calculate the current position of the robot
        encoderAverage = (readSensor(LeftEncoder) + readSensor(RightEncoder))*0.5;
        currentPosition = convertPosition(encoderAverage);
        
        //PI controller calculations
        error = distance - currentPosition;
        u = Kp*error + Ki*errorIntSum;
        
        //If Controller is not saturated, add integral
        if (abs(u) < 70) {  					
            errorIntSum = errorIntSum + error;
        }
        u = saturate(u, -80, 80);
    
        //For the 1st second, ramp up voltage to stop twitching
        if (k < 70) {	
            u = (((double) k - 49.0)/20.0) * u;
        }
        //Store all the values of the error in an array
        errorArray[k] = error;
        k = k + 1;
    
        // ** Straight Controller (P controller) **
        //Adjust for difference the difference in motor speeds, stopping the robot from curving
        encError = readSensor(RightEncoder) - readSensor(LeftEncoder);
        uDiff = Kp_straight*encError;
        uR = u - uDiff;
        uL = u + uDiff;
        uR = saturate(uR, -100, 100);
        uL = saturate(uL, -100, 100);
    
        //Use uR and uL to drive the motors
        motorPower(RightMotor, convertPower(uR));
        motorPower(LeftMotor, convertPower(uL));

        lcd_print(2, "Left Distance = %d", readSensor(LeftDistance));
        lcd_print(3, "Right Distance = %d", readSensor(RightDistance));
        lcd_print(5, "Sonar Distance = %d", readSensor(SonarSensor));


        delay(50);
        } while((abs(errorArray[k-1]) > (abs(distance)*tolerance)) || (abs(errorArray[k-40]) > (abs(distance)*tolerance)));
    
        motorPower(LeftMotor, 0);
        motorPower(RightMotor, 0);
    }

int driveToObject(int finalDistance) {
	
	//Ensure Arm is not blocking the sonar
	armUp(4000);

    /* For Sonar */
	// //Initialise Variables
	// int initialDistance = readSensor(SonarSensor);
	// int distance = initialDistance - finalDistance;

    /* For Distance Sensors */
    //Initialise Variables
    int left_distance = readSensor(LeftDistance);
    int right_distance = readSensor(RightDistance);
    int average_distance = (left_distance + right_distance) / 2;
    int distance = average_distance - finalDistance;

	//driveStraight until specified distance from the object
	driveStraight(distance);

	return distance;
}


//Convert a percentage input to a voltage output
int convertPower(double percentPower) {
	
	//Initialise Variables
	int convertedVoltage = 0;
	
	percentPower = saturate(percentPower, -100, 100);	//Saturate input between -100 and 100 - protects motors

	convertedVoltage = 50.0 * percentPower;	//Convert Percentage to Voltage

	return convertedVoltage;
}

//Convert Encoder Counts to position in mm
double convertPosition(double encoderCount) {

	double position = (1/encCountPerRev)*encoderCount*PI*drivingWheelDiameter;

	return position;
} 

//Convert Encoder counts to angle in degrees
double convertAngle(double encoderCount) {

	double angle = (360.0/encCountPerRev)*encoderCount;

	return angle;
} 
    