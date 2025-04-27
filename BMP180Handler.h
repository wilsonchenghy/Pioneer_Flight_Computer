#pragma once
#include <Adafruit_BMP085.h>

extern Adafruit_BMP085 bmp;
extern float altitude;
extern float prevAltitude;

void setupBMP180();
float readAltitude();