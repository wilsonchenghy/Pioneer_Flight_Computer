#include "MPU6050Handler.h"
#include "Config.h"

// Global variables for gyroscope readings
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float AngleRoll, AnglePitch, AngleYaw;
Quaternion q;

const float dt = 0.01; // Time step for integration

void setupMPU6050() {
  Wire.setClock(400000);                         // Set clock speed of I2C (400kB/s)
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);                  // Start the gyro in power mode
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(); 

  // Calibrate gyroscope
  for (int RateCalibrationNum = 0; RateCalibrationNum < 3000; RateCalibrationNum ++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 3000;
  RateCalibrationPitch /= 3000;
  RateCalibrationYaw /= 3000;
}

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

  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();   // Read gyro measurements around respective axes
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;                  // Convert measurements to °/s
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

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

bool readMPU6050() {
  gyro_signals();

  // Apply calibration offsets
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Update quaternion
  updateQuaternion(q, RateRoll, RatePitch, RateYaw, dt);

  // Convert to Euler angles
  QuaternionToEuler(q, &AngleRoll, &AnglePitch, &AngleYaw);

  // Convert to degrees and apply scaling factor
  AngleRoll = 1.5 * degrees(AngleRoll);
  AnglePitch = 1.5 * degrees(AnglePitch);
  AngleYaw = 1.5 * degrees(AngleYaw);

  return true;
}