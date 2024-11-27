#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>

enum State {
  POWER_ON, // blue led on
  SYSTEM_CHECK, // as when transitioned to this state, the blue led will blink once and the mpu6050 will be calibrated, if error occur, it goes to ERROR state and red led turns on while blue led turns off
  READY_TO_LAUNCH, // blue led start blinking for a second or so
  LAUNCHING_COUNTDOWN, // red led blinking while blue led is off
  LAUNCH, // red and blue led both light up for one second, and then ignite motor and all leds are turned off
  APOGEE, // all led is off
  ERROR // red led is on while blue led is off
};

// Disable Function Bool
bool useMicroSDCard = false;
bool useBMP180 = false;
bool useStartButton = false;

State currentState = POWER_ON;

#define YAW_AXIS_SERVO_PIN 6
#define PITCH_AXIS_SERVO_PIN 5
#define START_BUTTON_PIN 7
#define BLUE_LED_PIN 8
#define RED_LED_PIN 9
// #define MOSFET_PIN 3 // N-Channel MOSFET
// #define PARACHUTE_EJECTION_PIN 4
#define MICRO_SD_CARD_CS_PIN 10

#define seaLevelPressure_hPa 975.2

// Time
unsigned long currentTime;
unsigned long pastTime;
float timeInterval = 20.0;



/////// TVC ///////
float Kp = 1.6;
float Ki = 0.2;
float Kd = 1.0;
double SETPOINT = -20.0;
double pastError1 = 0.0;
double pastError2 = 0.0;
double integralError1 = 0.0;
double integralError2 = 0.0;

Servo yawAxisServo;
Servo pitchAxisServo;

// Define the 2 Angles (In degrees)
double yawAngle;
double pitchAngle;



/////// MPU6050 ///////
MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



/////// Parachute Ejection ///////
Servo parachuteEjectionServo;



/////// Micro SD Card Module ///////
File dataFile;



/////// BMP180 ///////
Adafruit_BMP085 bmp;



//////////////// SETUP ////////////////
void setup() {
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(START_BUTTON_PIN, INPUT);
  // pinMode(MOSFET_PIN, OUTPUT);

  // digitalWrite(MOSFET_PIN, HIGH);

  yawAxisServo.attach(YAW_AXIS_SERVO_PIN);
  pitchAxisServo.attach(PITCH_AXIS_SERVO_PIN);
  // parachuteEjectionServo.attach(PARACHUTE_EJECTION_PIN);

  Serial.begin(115200);

  switch (currentState) {
    case POWER_ON:
      digitalWrite(BLUE_LED_PIN, HIGH);

      // Set servo initial angle at 90 degree
      yawAxisServo.write(90);
      pitchAxisServo.write(90);
      parachuteEjectionServo.write(90);

      /////// MPU6050 ///////
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
      #endif

      Serial.println(F("Initializing MPU6050..."));
      mpu.initialize();
      pinMode(INTERRUPT_PIN, INPUT);

      while (!mpu.testConnection()) {
        digitalWrite(BLUE_LED_PIN, LOW);
        digitalWrite(RED_LED_PIN, HIGH);
        Serial.println(F("MPU6050 Connection Failed"));
      }
      Serial.println(F("MPU6050 Connection Successful"));

      /////// Micro SD Card Module ///////
      if (useMicroSDCard) {
        Serial.println(F("Initializing Micro SD Card Module..."));
        while (!SD.begin(MICRO_SD_CARD_CS_PIN)) {
          digitalWrite(BLUE_LED_PIN, LOW);
          digitalWrite(RED_LED_PIN, HIGH);
          Serial.println(F("Micro SD Card Module Initialization Failed"));
        }
        Serial.println(F("Micro SD Card Module Initialization Successful"));
        dataFile = SD.open("Data.csv", FILE_WRITE); // Seems the name of the csv has a character limit (e.g. cannot do FlightData)
        while(!dataFile) {
          digitalWrite(BLUE_LED_PIN, LOW);
          digitalWrite(RED_LED_PIN, HIGH);
          Serial.println(F("Error Opening FlightData.csv"));
        }
        if (dataFile) {
          dataFile.println("Time(ms), YawAngle, PitchAngle, RollAngle, Altitude");
          dataFile.close();
        }
      }

      // BMP180
      if (useBMP180) {
        while (!bmp.begin()) {
          digitalWrite(BLUE_LED_PIN, LOW);
          digitalWrite(RED_LED_PIN, HIGH);
          Serial.println("Cannot Find A Valid BMP180 Sensor");
        }
      }

      // Start Button
      if (useStartButton) {
        while(digitalRead(START_BUTTON_PIN) == LOW);
      }

      // Indicate start button pressed
      digitalWrite(BLUE_LED_PIN, LOW);
      delay(1000);
      digitalWrite(BLUE_LED_PIN, HIGH);

      currentState = SYSTEM_CHECK;
      break;

    case ERROR:
      digitalWrite(BLUE_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH);
      Serial.println("Error occurred during power on");
      break;

    default:
      digitalWrite(BLUE_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH);
      Serial.println("Unknown state");
      break;
  }
}





//////////////// LOOP ////////////////
void loop() {
  switch (currentState) {
    case SYSTEM_CHECK:
      Serial.println(F("Initializing DMP...")); 
      devStatus = mpu.dmpInitialize();

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(51);
      mpu.setYGyroOffset(8);
      mpu.setZGyroOffset(21);
      mpu.setXAccelOffset(1150);
      mpu.setYAccelOffset(-50);
      mpu.setZAccelOffset(1060);
      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
      } else {
        digitalWrite(BLUE_LED_PIN, LOW);
        digitalWrite(RED_LED_PIN, HIGH);
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
      }

      currentState = LAUNCH;
      // currentState = LAUNCH; // !!! temp
      // pastTime = millis(); // !!! temp
      break;

    case READY_TO_LAUNCH:
      for(int i=0; i<3; i++) {
        digitalWrite(BLUE_LED_PIN, LOW);
        delay(500);
        digitalWrite(BLUE_LED_PIN, HIGH);
        delay(500);
      }
      currentState = LAUNCHING_COUNTDOWN;
      break;

    case LAUNCHING_COUNTDOWN:
      digitalWrite(BLUE_LED_PIN, LOW);
      // Countdown 10 sec
      for(int i=0; i<10; i++) {
        digitalWrite(RED_LED_PIN, HIGH);
        delay(500);
        digitalWrite(RED_LED_PIN, LOW);
        delay(500);
      }
      currentState = LAUNCH;

      digitalWrite(BLUE_LED_PIN, HIGH);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(1000);

      pastTime = millis();
      break;

    case LAUNCH:
      // Ignite motor
      // digitalWrite(MOSFET_PIN, HIGH);
      // digitalWrite(BLUE_LED_PIN, LOW);
      // digitalWrite(RED_LED_PIN, LOW);

      currentTime = millis();
      unsigned long timeElasped = currentTime - pastTime;

      if (timeElasped >= timeInterval) {
        if (!dmpReady) return;
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

          // !!! is pitch and roll reversed?
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          // mpu.dmpGetGravity(&gravity, &q);
          // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

          // ypr in radians
          // yaw = degrees(ypr[0]);
          // roll = degrees(ypr[2]);
          // pitch = degrees(ypr[1]);

          // Serial.print(yaw);
          // Serial.print("\t");
          // Serial.print(roll);
          // Serial.print("\t");
          // Serial.print(pitch);
          // Serial.println("\t");

          // Quaternions
          // Serial.print("quat\t");
          // Serial.print(q.w);
          // Serial.print("\t");
          // Serial.print(q.x);
          // Serial.print("\t");
          // Serial.print(q.y);
          // Serial.print("\t");
          // Serial.println(q.z);

          QuaternionToEuler(q, &roll, &pitch, &yaw);
          Serial.print(degrees(yaw));
          Serial.print("\t");
          Serial.print(degrees(roll));
          Serial.print("\t");
          Serial.print(degrees(pitch));
          Serial.print("\t");

          // Get Accel/Gyro sensor readings
          // mpu.dmpGetAccel(&aa, fifoBuffer);
          // Serial.print("\tRaw Accl XYZ\t");
          // Serial.print(aa.x);
          // Serial.print("\t");
          // Serial.print(aa.y);
          // Serial.print("\t");
          // Serial.print(aa.z);
          // mpu.dmpGetGyro(&gy, fifoBuffer);
          // Serial.print("\tRaw Gyro XYZ\t");
          // Serial.print(gy.x);
          // Serial.print("\t");
          // Serial.print(gy.y);
          // Serial.print("\t");
          // Serial.print(gy.z);
          // Serial.println();

          /////// Micro SD Card Module ///////
          if (useMicroSDCard) {
            dataFile = SD.open("Data.csv", FILE_WRITE);
            if (dataFile) {
              dataFile.print(currentTime);
              dataFile.print(", ");
              dataFile.print(yaw);
              dataFile.print(", ");
              dataFile.print(roll);
              dataFile.print(", ");
              dataFile.println(pitch);
              if (useBMP180) {
                dataFile.print(", ");
                dataFile.print(bmp.readAltitude(seaLevelPressure_hPa * 100));
              }
              dataFile.close();
            } else {
              Serial.println("Error Opening FlightData.csv");
            }
          }
        }

        /////// TVC ///////
        yawAngle = PID(SETPOINT, degrees(yaw), timeElasped, &pastError1, &integralError1);
        pitchAngle = PID(SETPOINT, degrees(roll), timeElasped, &pastError2, &integralError2); // For current configuration, use the roll measurement

        // Serial.print(yawAngle);
        // Serial.print("\t");
        // Serial.print(pitchAngle);
        // Serial.println();

        yawAxisServo.write(90+yawAngle); // 90 for the angle offset
        Serial.print(90+yawAngle);
        Serial.println("\t");
        pitchAxisServo.write(90+pitchAngle); // 90 for the angle offset
        // Serial.println(90+pitchAngle);

        pastTime = currentTime;
      }

      // wait until reaching apogee to change state
      bool reachedApogee = false; // to be written
      if (reachedApogee) {
        Serial.println("Reached Apogee!");
        // digitalWrite(MOSFET_PIN, LOW);
        currentState = APOGEE;
        pastTime = millis();
      }
      break;
    
    case APOGEE:
      delay(100); // will adding a delay here be better?
      // Parachute Ejection
      parachuteEjectionServo.write(0);
      break;

    case ERROR:
      digitalWrite(BLUE_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH);
      Serial.println("Error occurred during system check");
      break;

    default:
      digitalWrite(BLUE_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH);
      Serial.println("Unknown state");
      break;
  }
}



double PID(double setPoint, double currentPoint, unsigned long timeChange, double *pastError, double *integralError) {
  double error = setPoint - currentPoint;
  // Serial.print(degrees(currentPoint));
  // Serial.print("\t");
  double deriviativeError = (error - *pastError) / timeChange;

  double outputAngle = Kp * error + Ki * (*integralError) + Kd * deriviativeError;

  if ((abs(outputAngle) >= 6.0) && ((error >= 0 && (*integralError) >= 0) || (error < 0 && (*integralError) < 0))) {
    *integralError = *integralError;
  }
  else {
    *integralError += error;
  }

  *pastError = error;

  return outputAngle;
}



void QuaternionToEuler(Quaternion q, float *roll, float *pitch, float *yaw) {
    // Roll
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    *roll = float(atan2(sinr_cosp, cosr_cosp));

    // Pitch
    double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    *pitch = float(2 * atan2(sinp, cosp) - M_PI / 2);

    // Yaw
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    *yaw = float(atan2(siny_cosp, cosy_cosp));
}