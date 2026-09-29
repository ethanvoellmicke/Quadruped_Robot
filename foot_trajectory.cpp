/*
 Generates the X and Z end position of a single robot foot throughout the course of one step
 This file is passed onto the leg_ik file which generates joint motion through IK

 Foot trajectory is split into the swing and the stance
  swing is spent in air and stance is spent on ground 
  swing is roughly 25% of the cycle with stance the other 75%

  Furthermore, swing is split into the lift, the motion forward, and the drop
  Stance focusses on moving the the foot backwards in relation to the body/shoulder
 */

#include <Arduino.h>
#include <math.h>

#include "foot_trajectory.h"

namespace
{
constexpr float PI_F = 3.14159265358979323846f;

//clamping values inbetween 0-1, useful in these equations where abnormalities can create physical problems
float clamp01(float value)
{
  if (value < 0.0f) return 0.0f;//self explanitory
  if (value > 1.0f) return 1.0f;
  return value;
}


//this creates motion that moves qickly at the beginning than slows down near the end
//used in the lift part of the swing, so it can lift quickly off ground then ease into step height
float easeOutCubic(float t)
{
  t = clamp01(t);
  const float oneMinusT = 1.0f - t;
  return 1.0f - oneMinusT * oneMinusT * oneMinusT;//cubic ease-out equation
}

//opposite of ease out, this creates motion that starts slow then accelerates
//used in the dropping of the foot to the ground
//allows for smooth motion instead of instant drop while its in the swing

float easeInCubic(float t)
{
  t = clamp01(t);//clamping to the normal range
  return t * t * t;//cubic ease-in equation
}

//creates a smooth transition from 0-1
//uses f(t) = t^2(3-2t), creating a zero slop at both ends, so motion begins and ends smoothly
//this is used in the forward movement of the foot suring the swing
float smoothStep(float t)
{
  t = clamp01(t);
  return t * t * (3.0f - 2.0f * t);//the smoothstep equation
}
}

//calculates the desired position of one of the feet at each instant in time
FootPosition calculateStepTrajectory(
  float timeSeconds,
  float phaseOffset,
  float speed,
  float zGround,
  float xCenter,
  float stepLength,
  float stepHeight,
  float stanceDepth
)
{

  //calculate where leg is in gait cycle(2pi) using phase = speed * time * phaseoffset
  //negative sign added to speed when testing to flip direction
  float phase = fmodf(-speed * timeSeconds + phaseOffset, 2.0f * PI_F);

//convert negative to positive
  if (phase < 0.0f)
  {
    phase += 2.0f * PI_F;
  }

  //normalize phase, convert 0-2pi rad into 0-1
  const float cycleT = phase / (2.0f * PI_F);

 //calculate front and back x posiiongs
  const float xBack = xCenter - 0.5f * stepLength;
  const float xFront = xCenter + 0.5f * stepLength;

  FootPosition foot{};  //foot posiiton struxture that containts x and z
  
  //swing to stance ratio
  constexpr float SWING_PORTION = 0.25f;//first 25% of cycle is swing, rest is on the ground in the stance

  //swing phase, if cycle is progress is less than 25%, the foot is in swing portion/ in air
  if (cycleT < SWING_PORTION)
  {
    const float swingT = cycleT / SWING_PORTION;//halfway through swing
    constexpr float LIFT_END = 0.30f;//lift part ends at 30%, then forward beigings
    constexpr float DROP_START = 0.70f;//after 70% of swing, dropping starts


  //LIFT PART OF SWING
    if (swingT < LIFT_END)
    {
      const float t = swingT / LIFT_END; //lift section
      const float lift = easeOutCubic(t);//going through easeOutCubic function to smooth out
      foot.x = xBack;//x becomes fixed to back of stride while the z lifts leg vertically
      foot.z = zGround + stepHeight * lift; //raise the foot, based off ground and step height
    }

    //MOVE FORWARD PART OF SWING
    else if (swingT < DROP_START)
    {
      const float t = (swingT - LIFT_END) / (DROP_START - LIFT_END);//forward section
      const float forward = smoothStep(t);//going through smoothStep function
      foot.x = xBack + stepLength * forward;//now x moves forward from back through steplength
      foot.z = zGround + stepHeight;//z stays the same throughout 
    }

    //DROP PART OF SWING
    else

    {
      const float t = (swingT - DROP_START) / (1.0f - DROP_START); //drop section
      const float drop = easeInCubic(t);//smoothed out
      foot.x = xFront;//x stays fixed at front of gait
      foot.z = zGround + stepHeight * (1.0f - drop);//z drops to zground
    }
  }

  //STANCE PART AFTER THE SWING
  else
  {
    const float stanceT =
      (cycleT - SWING_PORTION) / (1.0f - SWING_PORTION); //0.3 - 1.0

    foot.x = xFront - stepLength * stanceT;//move linearly from xfront to xback
    foot.z = zGround - stanceDepth * sinf(PI_F * stanceT);//shallow downward arc pushing agaisnt ground
  }

  return foot;
}
