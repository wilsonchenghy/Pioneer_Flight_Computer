#pragma once
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

extern MPU6050 mpu;
extern bool dmpReady;
extern uint8_t fifoBuffer[64];
extern volatile bool mpuInterrupt;

void setupMPU6050();
bool readMPU6050();