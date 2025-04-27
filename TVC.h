#pragma once
#include <Servo.h>

extern Servo yawAxisServo;
extern Servo pitchAxisServo;
extern double SETPOINT;

void initializeTVC();
double PID(double setPoint, double currentPoint, unsigned long *currentTime, unsigned long *pastTime, double *pastError, double *integralError);