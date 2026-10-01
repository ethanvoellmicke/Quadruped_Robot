/*
 Servos have their own calibration based on the set up, this file translates values from gait_controller and outputs the correct PWM signlas through the PCA to the servos for the current robot set up
 */


#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>

#include <math.h>

#include "servo_controller.h"


namespace
{

constexpr float PI_F = 3.14159265358979323846f;

//lowest PCA9685 pulse value that will be sent to a servo
constexpr int SERVO_MIN_PULSE = 150;

//highest PCA9685 pulse value that will be sent to a servo
constexpr int SERVO_MAX_PULSE = 600;

//standard hz for these servos
constexpr uint16_t SERVO_PWM_FREQUENCY_HZ = 50;

//knee servo has to move more becuase of the linkage
constexpr float KNEE_LINKAGE_RATIO = 1.08f;


//creating the PCA9685 drivr object at I2C address 0x40
Adafruit_PWMServoDriver pwm(0x40);


//keep track of which type of joint servo is controlling
enum class JointType : uint8_t
{
  Shoulder,
  Hip,
  Knee
};


//stores all of the calibration information needed for one servo
struct ServoCalibration
{
  //PCA channel this servo is connected to
  uint8_t channel;

  //the physical servo angle for the robots neutral pose
  float neutralDeg;

  //decides the direction the servo moves, + or -
  float direction;

  //minimum allowed joint offset
  float jointMinDeg;

  //maximum allowed joint offset
  float jointMaxDeg;

  //lowest physical angle this servo is allowed to reach
  float servoMinDeg;

  //highest physical angle this servo is allowed to reach
  float servoMaxDeg;

  //joint type, is it a shoulder, hip, or knee
  JointType jointType;
};


const ServoCalibration CALIBRATIONS[NUM_SERVOS] =

{
  //front left, shoulder, hip knee
  {0,  60.0f, 1.0f, -17.0f, 20.0f,  43.0f,  80.0f, JointType::Shoulder},
  {1, 78.0f, 1.0f, -36.0f, 24.0f, 54.0f, 114.0f, JointType::Hip},
  {2, 107.0f, 1.0f, -25.0f, 25.0f,  82.0f, 135.0f, JointType::Knee},

  //front right, shoulder, hip knee
  {3,  91.0f, 1.0f, -17.0f, 20.0f,  74.0f, 111.0f, JointType::Shoulder},
  {4,  92.0f, 1.0f, -36.0f, 24.0f,  56.0f, 116.0f, JointType::Hip},
  {5, 107.0f, 1.0f, -25.0f, 25.0f,  79.0f, 132.0f, JointType::Knee},

  //back left, shoulder, hip knee
  {6,  68.0f, 1.0f, -17.0f, 20.0f,  51.0f,  88.0f, JointType::Shoulder},
  {7,  98.0f, 1.0f, -36.0f, 24.0f,  74.0f, 134.0f, JointType::Hip},
  {8,  45.0f, 1.0f, -25.0f, 25.0f,  20.0f,  70.0f, JointType::Knee},

  //back right, shoulder, hip knee
  {9, 108.0f, 1.0f, -17.0f, 20.0f,  91.0f, 128.0f, JointType::Shoulder},
  {10, 98.0f, 1.0f, -36.0f, 24.0f,  62.0f, 122.0f, JointType::Hip},
  {11,135.0f, 1.0f, -25.0f, 25.0f, 110.0f, 160.0f, JointType::Knee}
};


//stores the last physical angle that was sent to each servo
float lastServoAnglesDeg[NUM_SERVOS] = {};


//clamping values
float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;

  if (value > maximum) return maximum;

  return value;
}


//converts a servo angle in degrees into a PCA pulse value
int angleToPulse(float angleDeg)
{
  //make sure angle is within 180deg
  const float safeAngle = clampFloat(angleDeg, 0.0f, 180.0f);

  //angle resolution
  const long angleTimesTen = lroundf(safeAngle * 10.0f);//multiply by 10 get get one decimal place in arduino

  //map 0-180 degrees to the allowed PCA pulse range
  return map(angleTimesTen, 0, 1800, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}


//sends an actual physical angle to one servo
void commandAbsoluteServoAngle(uint8_t servoIndex, float angleDeg)
{
  //get the calibration information for this servo
  const ServoCalibration &cal = CALIBRATIONS[servoIndex];

  //keep the requested angle inside this specific servos safe limits
  const float safeAngle = clampFloat(angleDeg, cal.servoMinDeg, cal.servoMaxDeg);

  //convert the angle to a pulse and send it to the servos PCA channel
  pwm.setPWM(cal.channel, 0, angleToPulse(safeAngle));

  //remember what angle was actually sent
  lastServoAnglesDeg[servoIndex] = safeAngle;
}


// returns the extra motion needed because of the joint linkage
float linkageRatio(JointType type)
{

  //knees use a different ratio(1.08) than hips and shoulders which are just 1
  return type == JointType::Knee ? KNEE_LINKAGE_RATIO : 1.0f;
}
}


//set up the PCA9685 when the robot starts
void initializeServoController()
{
  //start I2C communication
  Wire.begin();

  //initialize the PCA9685
  pwm.begin();

  //set the servo PWM frequency to 50 Hz
  pwm.setPWMFreq(SERVO_PWM_FREQUENCY_HZ);

  //give the PCA9685 time to initialize
  delay(500);


  //start the stored servo angles at their calibrated neutral values
  for (uint8_t i = 0; i < NUM_SERVOS; ++i)
  {
    lastServoAnglesDeg[i] = CALIBRATIONS[i].neutralDeg;
  }
}


//physically command all 12 servos to their calibrated neutral angles
void commandNeutralPose()
{
  for (uint8_t i = 0; i < NUM_SERVOS; ++i)
  {
    commandAbsoluteServoAngle(i, CALIBRATIONS[i].neutralDeg);
  }
}


//create the actual servo angles
void commandJointOffsetsRadians(const float jointOffsetsRad[NUM_SERVOS])
{
  //loop through all 12 servos
  for (uint8_t i = 0; i < NUM_SERVOS; ++i)
  {
    //get servo calibration info
    const ServoCalibration &cal = CALIBRATIONS[i];


    //convert the requested joint movement(from gait controller) into degrees.
    const float requestedJointDeg = jointOffsetsRad[i] * 180.0f / PI_F;


    //limit the requested joint movement before turning it into servo movement
    const float safeJointDeg = clampFloat(requestedJointDeg,cal.jointMinDeg,cal.jointMaxDeg
    );


    // apply the linkage ratio.
    const float actuatorOffsetDeg = safeJointDeg * linkageRatio(cal.jointType);

    const float absoluteServoDeg =
      cal.neutralDeg + cal.direction * actuatorOffsetDeg;

    //send final angle to the physical servo
    commandAbsoluteServoAngle(i, absoluteServoDeg);
  }
}


//copies the last commanded servo angles into an output array
void getLastServoAngles(float outputAnglesDeg[NUM_SERVOS])
{
  for (uint8_t i = 0; i < NUM_SERVOS; ++i)
  {
    outputAnglesDeg[i] = lastServoAnglesDeg[i];
  }
}
