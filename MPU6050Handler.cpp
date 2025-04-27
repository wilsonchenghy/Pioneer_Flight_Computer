#include "MPU6050Handler.h"
#include "Config.h"

MPU6050 mpu;
bool dmpReady = false;
volatile bool mpuInterrupt = false;
uint8_t fifoBuffer[64];

void dmpDataReady() { mpuInterrupt = true; }

void setupMPU6050() {
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  dmpReady = true;
}

bool readMPU6050() {
  if (!dmpReady) return false;
  return mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
}