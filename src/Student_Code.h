#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

void student_Main();    // The main entry point to the student code

// Add your function prototypes below

//Functions copied over from 201 Vex project
void driveStraight(int distance);
int driveToObject(int finalDistance);
void turnAngle(int targetAngle, int Kp, int tolerance);

int convertPower(double percentPower);
double convertPosition(double encoderCount);
double convertAngle(double encoderCount);

// DO NOT ADD ANY PROTOTYPES AFTER THIS LINE
#endif  // STUDENT_CODE_H