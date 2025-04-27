#include "TVC.h"
#include "Config.h"

float Kp = 0.4;
float Ki = 0.05;
float Kd = 0.15;
double SETPOINT = 0.0;

Servo yawAxisServo;
Servo pitchAxisServo;

void initializeTVC() {
  yawAxisServo.attach(YAW_AXIS_SERVO_PIN);
  pitchAxisServo.attach(PITCH_AXIS_SERVO_PIN);
  yawAxisServo.write(90);
  pitchAxisServo.write(90);
}

double PID(double setPoint, double currentPoint, unsigned long *currentTime, unsigned long *pastTime, double *pastError, double *integralError) {
  *currentTime = millis();
  double timeChange = (*currentTime - *pastTime) / 1000.0;
  double error = setPoint - currentPoint;
  double deriviativeError = (error - *pastError) / timeChange;
  *integralError += error * timeChange;
  double outputAngle = Kp * error + Ki * (*integralError) + Kd * deriviativeError;
  double angleSaturation = 13;
  if (outputAngle > angleSaturation) outputAngle = angleSaturation;
  else if (outputAngle < -angleSaturation) outputAngle = -angleSaturation;
  *pastError = error;
  *pastTime = *currentTime;
  return outputAngle;
}