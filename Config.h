#pragma once
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

#define YAW_AXIS_SERVO_PIN 6
#define PITCH_AXIS_SERVO_PIN 5
#define START_BUTTON_PIN 7
#define BLUE_LED_PIN 8
#define RED_LED_PIN 9
#define MICRO_SD_CARD_CS_PIN 10
#define INTERRUPT_PIN 2
#define seaLevelPressure_hPa 975.2

extern bool useMicroSDCard;
extern bool useBMP180;
extern bool useStartButton;
extern float timeInterval;