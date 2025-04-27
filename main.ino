#include "Config.h"
#include "State.h"
#include "TVC.h"
#include "MPU6050Handler.h"
#include "BMP180Handler.h"
#include "SDHandler.h"
#include "Utility.h"

bool useMicroSDCard = false;
bool useBMP180 = true;
bool useStartButton = false;
float timeInterval = 20.0;
State currentState = POWER_ON;

unsigned long pastTime = 0;
unsigned long currentTime = 0;
unsigned long pastTimeTVC = 0;
unsigned long pastTimeTVC2 = 0;
unsigned long currentTimeTVC = 0;
unsigned long currentTimeTVC2 = 0;
float yaw = 0, pitch = 0, roll = 0;
float pastError1 = 0.0, pastError2 = 0.0;
float integralError1 = 0.0, integralError2 = 0.0;
bool reachedApogee = false;
bool apogeeAltitudeCondition = false;
float altitudeDescentThreshold = 1.0;

void setup() {
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(START_BUTTON_PIN, INPUT);
  Serial.begin(115200);

  initializeTVC();
  setupMPU6050();
  if (useBMP180) setupBMP180();
  if (useMicroSDCard) setupSDCard();

  digitalWrite(BLUE_LED_PIN, HIGH);
  yawAxisServo.write(90);
  pitchAxisServo.write(90);
  delay(1000);

  currentState = SYSTEM_CHECK;
}

void loop() {
  switch (currentState) {
    case SYSTEM_CHECK:
      if (dmpReady) {
        Serial.println("System Check Passed");
        currentState = READY_TO_LAUNCH;
      } else {
        Serial.println("System Check Failed");
        currentState = ERROR;
      }
      break;

    case READY_TO_LAUNCH:
      for (int i = 0; i < 3; i++) {
        digitalWrite(BLUE_LED_PIN, LOW);
        delay(500);
        digitalWrite(BLUE_LED_PIN, HIGH);
        delay(500);
      }
      currentState = LAUNCHING_COUNTDOWN;
      break;

    case LAUNCHING_COUNTDOWN:
      for (int i = 0; i < 10; i++) {
        digitalWrite(RED_LED_PIN, HIGH);
        delay(500);
        digitalWrite(RED_LED_PIN, LOW);
        delay(500);
      }
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(BLUE_LED_PIN, HIGH);
      delay(1000);
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, LOW);
      pastTime = millis();
      currentState = LAUNCH;
      break;

    case LAUNCH:
      currentTime = millis();
      if ((currentTime - pastTime) >= timeInterval) {
        if (readMPU6050()) {
          Quaternion q;
          memcpy(&q, fifoBuffer, sizeof(q));
          QuaternionToEuler(q, &roll, &pitch, &yaw);

          double yawAngle = PID(0.0, degrees(yaw), &currentTimeTVC, &pastTimeTVC, &pastError1, &integralError1);
          double pitchAngle = PID(0.0, degrees(roll), &currentTimeTVC2, &pastTimeTVC2, &pastError2, &integralError2);

          yawAxisServo.write(90 - yawAngle);
          pitchAxisServo.write(90 + pitchAngle);

          if (useMicroSDCard) {
            writeSDCard(currentTime, yaw, roll, pitch, useBMP180 ? readAltitude() : 0);
          }

          pastTime = currentTime;
        }
      }
      if (useBMP180) {
        float currentAltitude = readAltitude();
        if ((currentAltitude - prevAltitude) < -altitudeDescentThreshold) {
          apogeeAltitudeCondition = true;
        }
        prevAltitude = currentAltitude;

        if (apogeeAltitudeCondition && (mpu.dmpGetAccel(&aa, fifoBuffer), aa.x < 0)) {
          reachedApogee = true;
        }
      }
      if (reachedApogee) {
        currentState = APOGEE;
        pastTime = millis();
      }
      break;

    case APOGEE:
      yawAxisServo.write(0);
      pitchAxisServo.write(0);
      digitalWrite(BLUE_LED_PIN, HIGH);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(10000);
      digitalWrite(BLUE_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, LOW);
      break;

    case ERROR:
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(BLUE_LED_PIN, LOW);
      Serial.println("ERROR STATE");
      break;

    default:
      Serial.println("UNKNOWN STATE");
      break;
  }
}