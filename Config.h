#pragma once
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>

#define YAW_AXIS_SERVO_PIN 6
#define PITCH_AXIS_SERVO_PIN 5
#define PARACHUTE_EJECTION_PIN 4
#define BLUE_LED_PIN 13
#define RED_LED_PIN 9
#define START_BUTTON_PIN 7
#define MICRO_SD_CARD_CS_PIN 10
#define INTERRUPT_PIN 2
#define seaLevelPressure_hPa 975.2