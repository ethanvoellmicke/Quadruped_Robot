/*
Using inverse kinematics to convert the 2D foot position from foot_trajectory into the hip and knee joint angles
 */

#include <Arduino.h>
#include <math.h>

#include "leg_ik.h"

namespace
{
constexpr float UPPER_LEG_LENGTH = 0.110f;//physical length of the upper leg link
constexpr float LOWER_LEG_LENGTH = 0.1105f;//physical length of the lower leg link

//clamping values between min and max
float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;//under minumum = minimum
  if (value > maximum) return maximum;//over maximum = maximum
  return value;//inbetween min and max keep the same
}
}


LegAngles solveLegIK(float x, float z)
{
  const float requestedDistance = sqrtf(x * x + z * z);//straight line distance from hip joint to foot position(hypotenuse)
  const float maximumReach = UPPER_LEG_LENGTH + LOWER_LEG_LENGTH - 0.001f;//defining max reach
  const float minimumReach = fabsf(UPPER_LEG_LENGTH - LOWER_LEG_LENGTH) + 0.001f;//defining min reach(essentially zero here because of similar lengths)
  const float distance = clampFloat(requestedDistance, minimumReach, maximumReach);//clsmping distance, making sure its in the workspace of the leg
  
//using law of cosines to get the angle between leg links(knee)
//using (c^2-a^2-b^2)/2ab
  float cosineKnee =
    (distance * distance - UPPER_LEG_LENGTH * UPPER_LEG_LENGTH - LOWER_LEG_LENGTH * LOWER_LEG_LENGTH)
    / (2.0f * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);

  cosineKnee = clampFloat(cosineKnee, -1.0f, 1.0f);//because of rounding, a value of 1.00001 can occur from acos(), this prevents that

  const float knee = -acosf(cosineKnee);//use the negative configuration as there are usually 2 solutions
  const float angleToFoot = atan2f(z, x);//find direction from hip to foot

  //we know the direction the foot is in from the hip, must account for middle angle to know where upper leg should point
  const float internalAngle = atan2f( LOWER_LEG_LENGTH * sinf(fabsf(knee)), UPPER_LEG_LENGTH + LOWER_LEG_LENGTH * cosf(fabsf(knee)));
  //internal angle works by having lower*sin(knee) and dividing by upper+lower*cos(knee) where atan gets the angular offset


  const float hip = angleToFoot + internalAngle;//getting final hip angle
  return {hip, knee};//return values to gait controller
}
