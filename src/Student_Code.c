/**
 * @file Student_Code.c
 * @author your name (you@domain.com)
 * @brief description of this file
 * @version 0.1
 * @date yyyy-mm-dd
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Libraries. DO NOT REMOVE */
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "Student_Code.h"
#include "../include/ui.h"

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
    ToggleChart();
    PlotSensor(LeftEncoder);

    driveStraight(10000);
}

// ----------------------------------------------- Function definitions go here  -----------------------------------------------//
// Don't forget to add your function prototypes to Student_Code.h

void driveStraight(int distance) {

    //Initialise Variables
    int error = 0, errorIntSum = 0, encError = 0;
    int k = 50, errorArray[1000] = {0}; 
    double currentPosition = 0;
    double u = 0, uL = 0, uR = 0, uDiff = 0;
    double encoderAverage;
    double tolerance = 0.1;
    double Kp = 1.0, Ki = 0.1, Kp_straight = 1.0; 
    
    int i;
    for (i = 0; i <= 1000; i++) {
        errorArray[i] = 9999;
    }
    
    //Reset Encoders
    resetEncoder(LeftEncoder);
    resetEncoder(RightEncoder);
    
    ConfigSlot(1, &Kp, "Kp");
    ConfigSlot(2, &Ki, "Ki");
    ConfigSlot(3, &Kp_straight, "Kp_straight");
    Pause();

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
        u = saturate(u, -60, 60);
    
        // //For the 1st second, ramp up voltage to stop twitching
        // if (k < 70) {	
        //     u = (((double) k - 49.0)/20.0) * u;
        // }
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

        ScreenPrint(5, "Sonar Sensor = %d", readSensor(SonarSensor));
        CustomPlot(1, (int) u, "Control Effort");

        delay(50);
        } while((abs(errorArray[k-1]) > (abs(distance)*tolerance)) || (abs(errorArray[k-40]) > (abs(distance)*tolerance)));

        motorPower(LeftMotor, 0);
        motorPower(RightMotor, 0);
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