#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <Adafruit_BMP085.h>

#define seaLevelPressure_hPa 1020.0

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float AngleRoll, AnglePitch, AngleYaw;

const float dt = 0.01;  

/////// TVC ///////
float Kp = 0.4;
float Ki = 0.05;
float Kd = 0.15;
double SETPOINT = 0.0;
double pastError1 = 0.0;
double pastError2 = 0.0;
double integralError1 = 0.0;
double integralError2 = 0.0;

// Time
unsigned long currentTime;
unsigned long pastTime;
float timeInterval = 20.0;

unsigned long currentTimeTVC = 0.0;
unsigned long pastTimeTVC = 0.0;
unsigned long currentTimeTVC2 = 0.0;
unsigned long pastTimeTVC2 = 0.0;

Servo yawAxisServo;
Servo pitchAxisServo;

// Define the 2 Angles (In degrees)
double yawAngle;
double pitchAngle;

struct Quaternion {
  float w, x, y, z;
  
  Quaternion() : w(1), x(0), y(0), z(0) {}
  
  void normalize() {
    float norm = sqrt(w*w + x*x + y*y + z*z);
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
  }
};

Quaternion q;

/////// BMP180 ///////
Adafruit_BMP085 bmp;
float altitude= 0;
float prevAltitude = 0;
float altitudeDescentThreshold = 5;

/////// Parachute Ejection ///////
Servo parachuteEjectionServo;



void setup() {
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH); 

  Wire.setClock(400000);                         // Set clock speed of I2C (400kB/s)
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);                  // Start the gyro in power mode
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(); 

  for (int RateCalibrationNum = 0; RateCalibrationNum < 3000; RateCalibrationNum ++) {
    
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=3000;
  RateCalibrationPitch/=3000;
  RateCalibrationYaw/=3000;

  yawAxisServo.attach(6);
  pitchAxisServo.attach(5);

  yawAxisServo.write(90);
  pitchAxisServo.write(90);

  // BMP180
  while (!bmp.begin()) {
    Serial.println("Cannot Find A Valid BMP180 Sensor");
  }

  // Parachute
  parachuteEjectionServo.attach(4);
  parachuteEjectionServo.write(65);
  
}



void loop() {
  
  currentTime = millis();
  unsigned long timeElasped = currentTime - pastTime;

  gyro_signals();

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  updateQuaternion(q, RateRoll, RatePitch, RateYaw, dt);

  QuaternionToEuler(q, &AngleRoll, &AnglePitch, &AngleYaw);

  // Serial.print("Roll Rate [°/s]=");
  // Serial.print(RateRoll);
  // Serial.print(" Pitch Rate [°/s]=");
  // Serial.print(RatePitch);
  // Serial.print(" Yaw Rate [°/s]=");
  // Serial.println(RateYaw);

  AngleRoll = 1.5*degrees(AngleRoll);
  AnglePitch = 1.5*degrees(AnglePitch);
  AngleYaw = 1.5*degrees(AngleYaw);

  Serial.print("Roll Angle [°]=");
  Serial.print(AngleRoll);
  Serial.print(" Pitch Angle [°]=");
  Serial.print(AnglePitch);
  Serial.print(" Yaw Angle [°]=");
  Serial.print(AngleYaw);
  Serial.print("\t");

  yawAngle = PID(SETPOINT, AngleRoll, &currentTimeTVC, &pastTimeTVC, &pastError1, &integralError1);
  pitchAngle = PID(SETPOINT, AnglePitch, &currentTimeTVC2, &pastTimeTVC2, &pastError2, &integralError2); // For current configuration, use the roll measurement

  yawAxisServo.write(90+yawAngle); // 90 for the angle offset
  pitchAxisServo.write(90+pitchAngle); // 90 for the angle offset


  Serial.print(yawAngle);
  Serial.print("\t");
  Serial.print(pitchAngle);
  Serial.println("\t");
  
  pastTime = currentTime;

  // Parachute
  // altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
  // if ((altitude - prevAltitude) < - altitudeDescentThreshold) { // Need to modify threshold
  //   parachuteEjectionServo.write(0);
  //   while(1) {
  //     Serial.println("Apogee Reached");
  //   }
  // }
  // prevAltitude = altitude;

  delay(10);

}

////////////////Set up transmission between MPU6050 and Microcontroller////////////////

void gyro_signals(void) { 
  Wire.beginTransmission(0x68);                  // Start I2C comms with gyro
  Wire.write(0x1A);                              // Switch on low-pass filter
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();                        // Set sensitivity scale factor

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();                        // Access register storing gyro measurements

  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read()<<8 | Wire.read();   // Read gyro measurements around respective axes
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();

  RateRoll = (float)GyroX/65.5;                  // Convert measurements to °/s
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;
}

////////////////Convert Measured Rotation Rates to Quaternion Form////////////////

void updateQuaternion(Quaternion &q, float gx, float gy, float gz, float dt) {
  // Convert gyroscope rates from degrees to radians
  gx *= (M_PI / 180.0);
  gy *= (M_PI / 180.0);
  gz *= (M_PI / 180.0);

  // Compute the quaternion derivative
  float qw = 0.5 * (-q.x * gx - q.y * gy - q.z * gz);
  float qx = 0.5 * (q.w * gx + q.y * gz - q.z * gy);
  float qy = 0.5 * (q.w * gy - q.x * gz + q.z * gx);
  float qz = 0.5 * (q.w * gz + q.x * gy - q.y * gx);

  // Update the quaternion
  q.w += qw * dt;
  q.x += qx * dt;
  q.y += qy * dt;
  q.z += qz * dt;

  // Normalize the quaternion
  q.normalize();
}

////////////////Convert Quaternions to Euler Angles////////////////

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

////////////////PID Controller////////////////

double PID(double setPoint, double currentPoint, unsigned long *currentTime, unsigned long *pastTime, double *pastError, double *integralError) {
  *currentTime = millis();

  double timeChange = (*currentTime - *pastTime) / 1000.0; // timeChange in seconds

  double error = setPoint - currentPoint;

  double deriviativeError = (error - *pastError) / timeChange;

  *integralError += error * timeChange;

  // Serial.print(Kd * deriviativeError);
  // Serial.print("\t");

  double outputAngle = Kp * error + Ki * (*integralError) + Kd * deriviativeError;

  double angleSaturation = 13;
  if (outputAngle > angleSaturation) {
    outputAngle = angleSaturation;
  } else if (outputAngle < -angleSaturation) {
    outputAngle = -angleSaturation;
  }

  *pastError = error;
  *pastTime = *currentTime;

  return outputAngle;
}
