#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#include "servo_controller.h"

namespace
{
constexpr float PI_F = 3.14159265358979323846f;
constexpr int SERVO_MIN_PULSE = 150;
constexpr int SERVO_MAX_PULSE = 600;
constexpr uint16_t SERVO_PWM_FREQUENCY_HZ = 50;
constexpr float KNEE_LINKAGE_RATIO = 1.08f;

Adafruit_PWMServoDriver pwm(0x40);

enum class JointType : uint8_t
{
  Shoulder,
  Hip,
  Knee
};

struct ServoCalibration
{
  uint8_t channel;
  float neutralDeg;
  float direction;
  float jointMinDeg;
  float jointMaxDeg;
  float servoMinDeg;
  float servoMaxDeg;
  JointType jointType;
};

// Order: FL shoulder/hip/knee, FR, BL, BR.
const ServoCalibration CALIBRATIONS[NUM_SERVOS] =

{
  {0,  60.0f, 1.0f, -17.0f, 20.0f,  43.0f,  80.0f, JointType::Shoulder},
{1, 78.0f, 1.0f, -36.0f, 24.0f, 54.0f, 114.0f, JointType::Hip},
{2, 107.0f, 1.0f, -25.0f, 25.0f,  82.0f, 135.0f, JointType::Knee},

  {3,  91.0f, 1.0f, -17.0f, 20.0f,  74.0f, 111.0f, JointType::Shoulder},
  {4,  92.0f, 1.0f, -36.0f, 24.0f,  56.0f, 116.0f, JointType::Hip},
  {5, 107.0f, 1.0f, -25.0f, 25.0f,  79.0f, 132.0f, JointType::Knee},

  {6,  68.0f, 1.0f, -17.0f, 20.0f,  51.0f,  88.0f, JointType::Shoulder},
  {7,  98.0f, 1.0f, -36.0f, 24.0f,  74.0f, 134.0f, JointType::Hip},
  {8,  45.0f, 1.0f, -25.0f, 25.0f,  20.0f,  70.0f, JointType::Knee},

  {9, 108.0f, 1.0f, -17.0f, 20.0f,  91.0f, 128.0f, JointType::Shoulder},
  {10, 98.0f, 1.0f, -36.0f, 24.0f,  62.0f, 122.0f, JointType::Hip},
  {11,135.0f, 1.0f, -25.0f, 25.0f, 110.0f, 160.0f, JointType::Knee}
};

/*
{
  {0,  60.0f, 1.0f, -17.0f, 20.0f,  43.0f,  80.0f, JointType::Shoulder},
  {1,  95.0f, 1.0f, -36.0f, 24.0f,  71.0f, 131.0f, JointType::Hip},
  {2, 107.0f, 1.0f, -25.0f, 25.0f,  82.0f, 135.0f, JointType::Knee},

  {3,  91.0f, 1.0f, -17.0f, 20.0f,  74.0f, 111.0f, JointType::Shoulder},
  {4,  85.0f, 1.0f, -36.0f, 24.0f,  49.0f, 109.0f, JointType::Hip},
  {5, 107.0f, 1.0f, -25.0f, 25.0f,  79.0f, 132.0f, JointType::Knee},

  {6,  68.0f, 1.0f, -17.0f, 20.0f,  51.0f,  88.0f, JointType::Shoulder},
  {7,  88.0f, 1.0f, -36.0f, 24.0f,  64.0f, 124.0f, JointType::Hip},
  {8, 45.0f, 1.0f, -25.0f, 25.0f, 20.0f, 70.0f, JointType::Knee},
  {9, 108.0f, 1.0f, -17.0f, 20.0f,  91.0f, 128.0f, JointType::Shoulder},
  {10, 98.0f, 1.0f, -36.0f, 24.0f,  62.0f, 122.0f, JointType::Hip},
  {11,135.0f, 1.0f, -25.0f, 25.0f, 110.0f, 160.0f, JointType::Knee}
};

*/
 // {8,  135.0f, 1.0f, -25.0f, 25.0f,  60.0f, 113.0f, JointType::Knee},


float lastServoAnglesDeg[NUM_SERVOS] = {};

float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

int angleToPulse(float angleDeg)
{
  const float safeAngle = clampFloat(angleDeg, 0.0f, 180.0f);
  const long angleTimesTen = lroundf(safeAngle * 10.0f);
  return map(angleTimesTen, 0, 1800, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void commandAbsoluteServoAngle(uint8_t servoIndex, float angleDeg)
{
  const ServoCalibration &cal = CALIBRATIONS[servoIndex];
  const float safeAngle = clampFloat(angleDeg, cal.servoMinDeg, cal.servoMaxDeg);
  pwm.setPWM(cal.channel, 0, angleToPulse(safeAngle));
  lastServoAnglesDeg[servoIndex] = safeAngle;
}

float linkageRatio(JointType type)
{
  return type == JointType::Knee ? KNEE_LINKAGE_RATIO : 1.0f;
}
}

void initializeServoController()
{
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_PWM_FREQUENCY_HZ);
  delay(500);

  for (uint8_t i = 0; i < NUM_SERVOS; ++i)
  {
    lastServoAnglesDeg[i] = CALIBRATIONS[i].neutralDeg;
  }
}

void commandNeutralPose()
{
  for (uint8_t i = 0; i < NUM_SERVOS; ++i)
  {
    commandAbsoluteServoAngle(i, CALIBRATIONS[i].neutralDeg);
  }
}

void commandJointOffsetsRadians(const float jointOffsetsRad[NUM_SERVOS])
{
  for (uint8_t i = 0; i < NUM_SERVOS; ++i)
  {
    const ServoCalibration &cal = CALIBRATIONS[i];
    const float requestedJointDeg = jointOffsetsRad[i] * 180.0f / PI_F;
    const float safeJointDeg = clampFloat(
      requestedJointDeg,
      cal.jointMinDeg,
      cal.jointMaxDeg
    );

    const float actuatorOffsetDeg = safeJointDeg * linkageRatio(cal.jointType);
    const float absoluteServoDeg =
      cal.neutralDeg + cal.direction * actuatorOffsetDeg;

    commandAbsoluteServoAngle(i, absoluteServoDeg);
  }
}

void getLastServoAngles(float outputAnglesDeg[NUM_SERVOS])
{
  for (uint8_t i = 0; i < NUM_SERVOS; ++i)
  {
    outputAnglesDeg[i] = lastServoAnglesDeg[i];
  }
}
