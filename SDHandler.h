#pragma once
#include <SD.h>

void setupSDCard();
void writeSDCard(unsigned long time, float yaw, float roll, float pitch, float altitude = 0);