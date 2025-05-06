#include "Utility.h"
#include <math.h>

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