#pragma once
#include "MPU6050_6Axis_MotionApps612.h"

void QuaternionToEuler(Quaternion q, float *roll, float *pitch, float *yaw);