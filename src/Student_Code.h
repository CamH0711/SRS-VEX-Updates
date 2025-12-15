#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

void student_Main();    // The main entry point to the student code

// Add your function prototypes below

double findObjectAngle();
void StraightenAngle(double percentPower);


//Functions copied over from 201 Vex project
void driveStraight(int distance);
int convertPower(double percentPower);
double convertPosition(double encoderCount);
double convertAngle(double encoderCount);
int driveToObject(int finalDistance);

// ---------------------- Defining Controller Parameters ------------------------------
double Kp = 1.0;                    // Proportional Controller gain
double Ki = 0.1;                    // Integral Controller gain
double u;                           //Control Effort
int error;                          //Controller Error

// DO NOT ADD ANY PROTOTYPES AFTER THIS LINE
#endif  // STUDENT_CODE_H