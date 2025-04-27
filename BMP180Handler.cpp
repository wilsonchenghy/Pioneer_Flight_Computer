#include "BMP180Handler.h"
#include "Config.h"

Adafruit_BMP085 bmp;
float altitude = 0;
float prevAltitude = 0;

void setupBMP180() { bmp.begin(); }

float readAltitude() {
  altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
  return altitude;
}