#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

void student_Main();    // The main entry point to the student code

// Add your function prototypes below

double findObjectAngle();


//Functions copied over from MECHENG 201 project
void driveStraight(int distance);
int convertPower(double percentPower);
double convertPosition(double encoderCount);
double convertAngle(double encoderCount);



// DO NOT ADD ANY PROTOTYPES AFTER THIS LINE
#endif  // STUDENT_CODE_H