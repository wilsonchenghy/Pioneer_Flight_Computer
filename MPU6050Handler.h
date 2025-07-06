#pragma once
#include <Wire.h>
#include <math.h>

// Quaternion structure for attitude estimation
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

// Global variables for gyroscope readings
extern float RateRoll, RatePitch, RateYaw;
extern float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
extern float AngleRoll, AnglePitch, AngleYaw;
extern Quaternion q;

// Function declarations
void setupMPU6050();
void gyro_signals();
void updateQuaternion(Quaternion &q, float gx, float gy, float gz, float dt);
void QuaternionToEuler(Quaternion q, float *roll, float *pitch, float *yaw);
bool readMPU6050();