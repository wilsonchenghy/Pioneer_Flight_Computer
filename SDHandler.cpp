#include "SDHandler.h"
#include "Config.h"

File dataFile;

void setupSDCard() {
  SD.begin(MICRO_SD_CARD_CS_PIN);
  dataFile = SD.open("Data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time(ms),YawAngle,RollAngle,PitchAngle,Altitude");
    dataFile.close();
  }
}

void writeSDCard(unsigned long time, float yaw, float roll, float pitch, float altitude) {
  dataFile = SD.open("Data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(time);
    dataFile.print(",");
    dataFile.print(yaw);
    dataFile.print(",");
    dataFile.print(roll);
    dataFile.print(",");
    dataFile.print(pitch);
    dataFile.print(",");
    dataFile.print(altitude);
    dataFile.println();
    dataFile.close();
  } else {
    Serial.println("Error Opening FlightData.csv");
  }
}