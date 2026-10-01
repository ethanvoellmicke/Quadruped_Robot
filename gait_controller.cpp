/*THE MAIN FILE
-controls overall movement of the robot legs

-Specifically, it controls the timing of different gaits 
-gives each leg a phase offset so they move in correct order
-finds desirect x and z positions thorughout each step(using foot_trajectory)
-uses leg_ik to convert desired fot posiiotn into servo angles
-Uses roll and pitch measurements to stabilize robot
-does some feed forward to shift robot away  from lifting leg


Overall file flow, 
-gait timing
-foot trajectory
-feed forward
-imu corrections
-inverse kinematics
-joint offets
-servo controller
-PCA/
-servo commands
 */

#include <Arduino.h>
#include <math.h>

#include "gait_controller.h"
#include "foot_trajectory.h"
#include "leg_ik.h"
#include "servo_controller.h"
#include "imu_controller.h"

namespace
{
constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 2.0f * PI_F;

constexpr uint16_t FRAME_PERIOD_MS = 20;//the time in between new servo commands
constexpr float TIME_STEP = static_cast<float>(FRAME_PERIOD_MS) / 1000.0f;//make it^ into seconds

constexpr float GAIT_SCALE = 0.40f;//40% of movement reaches servo
constexpr float NEUTRAL_FOOT_X = 0.040f;


//crawl timing and body shifting

//leg movement broken into 4 portions, first is the swing phase
constexpr float SWING_PORTION = 0.25f;//25% of cycle is swing

//prepare body before leg lifts, final 12% of cycle transfers weight away from leg
constexpr float PRE_LIFTOFF_PORTION = 0.12;//12%

//time used to reload leg after touching ground, probably wont do anything with this.
//constexpr float POST_TOUCHDOWN_PORTION = 0.04f;

//Swings body forward or backwards 4mm before legs lifts as preperation
constexpr float FEEDFORWARD_PITCH_SHIFT_M = 0.004f;//.008

//had to flip direction after testing
constexpr float FEEDFORWARD_PITCH_DIRECTION = -1.0f;

//can do feedforward with imu as well, currently 0 though
constexpr float FEEDFORWARD_ROLL_DEG = 0.0f;//2
constexpr float FEEDFORWARD_ROLL_RAD =
    FEEDFORWARD_ROLL_DEG * PI_F / 180.0f;

//flipped sign after testing
constexpr float FEEDFORWARD_ROLL_DIRECTION = -1.0f;

//when one leg lifts up, the oposing leg gets stronger correction to be safe as its doing more work now
constexpr float MAX_STANCE_NORMALIZATION = 1.50f;

//IMU TUNING

constexpr float IMU_DEADBAND_DEG = 0.75f;//deadband at .75degrees, anything under is effectively 0 degrees

constexpr float ROLL_KP = 1.0f;//proportional gain
constexpr float MAX_ROLL_CORRECTION_DEG = 12.0f;//no more than 12 degrees, to prevent over rotating

constexpr float PITCH_KP_METERS_PER_DEG = 0.00012f;//pitch correction changes z position of feet, convert degrees nto meters
constexpr float PITCH_KD_METERS_PER_DEG_PER_SEC = 0.00008f;//derivative gain, low

constexpr float PITCH_RATE_DEADBAND_DEG_PER_SEC = 2.0f;//ignoring small pitch RATES like gyro noise

constexpr float MAX_PITCH_D_CORRECTION_M = 0.0035f;//limit the correction the derivative part can do
constexpr float MAX_PITCH_CORRECTION_M = 0.010f;//limit total pitch correction(mm)

constexpr float ROLL_CORRECTION_SIGN = -1.0f;
constexpr float PITCH_CORRECTION_SIGN = 1.0f;

//Controller states

//robot starts in stand mode
GaitMode gaitMode = GaitMode::Stand;

//tracks how far through the gait the robot is
float gaitTime = 0.0f;

//controlling how fast the gait cycles
float gaitSpeed = 2.0f;

// normal Z position of the feet relative to the hips
float zGround = -0.22f;


float forwardCommand = 1.0f; // forward-  -1 reverse, 0 no translation, +1 forward
float turnCommand = 0.0f;// turn-  -1 turn one way, 0 straight, +1 turn the other way

//stores when the next gait update should happen
unsigned long nextUpdateMs = 0;

enum LegIndex : uint8_t//the names for each leg, the numbers are used for gait/cycle order
{
  FL = 0,
  FR = 1,
  BL = 2,
  BR = 3
};


//stores where a leg is in its gait and if it is on the ground
struct LegPhaseState
{
  float cycleT; // psition in the gait cycle from 0 to 1
  float contactWeight;//1 while physically in stance and 0 while physically swinging.
  float feedforwardInfluence;//used for moving robot away from lifting leg
};


struct LegCorrection //added correction for each leg
{
  float x;
  float z;
  float shoulder;
};


//clamping values(in general)
float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}


//a function to smooth out the motion from 0-1
float smoothStep(float t)
{
  t = clampFloat(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}


// deadband function, low values = 0
float applyDeadband(float value, float deadband)
{
  return fabsf(value) < deadband ? 0.0f : value;
}


//check whether a leg is on the left side
bool isLeftLeg(uint8_t legIndex)
{
  return legIndex == FL || legIndex == BL;
}


// set the timing offset for each leg
void fillPhaseOffsets(float phases[4])
{
  //trot mode- diagonal legs move together, off set diagonals by PI, or half the cycle
  if (gaitMode == GaitMode::Trot)
  {
    phases[FL] = 0.0f;
    phases[FR] = PI_F;
    phases[BL] = PI_F;
    phases[BR] = 0.0f;
    return;
  }

 //order of the legs in the cycle in general, FL, BR, FR, BL 
  phases[FL] = 0.0f;
  phases[FR] = 3.0f * PI_F / 2.0f;
  phases[BL] = PI_F;
  phases[BR] = PI_F / 2.0f;
}


//find where a leg currently is in its 0 to 1 gait cycle
float calculateCycleT(
    float sampleTime,
    float phaseOffset)
{
  float phase =
      fmodf( -gaitSpeed * sampleTime + phaseOffset,TWO_PI_F);

  // keep the phase positive
  if (phase < 0.0f)
  {
    phase += TWO_PI_F;
  }

  // convert 0 to 2pi into 0 to 1
  return phase / TWO_PI_F;
}


//is leg swining, standing, or loading
LegPhaseState calculateLegPhaseState(float sampleTime, float phaseOffset)
{
  LegPhaseState state{};

  state.cycleT = calculateCycleT(sampleTime, phaseOffset);


  //under 0.25 means the foot is in the swing part of the gait
  const bool isSwinging =
      state.cycleT < SWING_PORTION;

  // make sure no imu correction on swingning leg
  state.contactWeight = isSwinging ? 0.0f : 1.0f;

  // while swinging, use the full planned body shift
  if (isSwinging)
  {
    state.feedforwardInfluence = 1.0f;
    return state;
  }


//doing feedforward before the leg lift off

  //find when the pre-liftoff body shift should start
  const float preLiftoffEnd = SWING_PORTION + PRE_LIFTOFF_PORTION;

  if (state.cycleT < preLiftoffEnd)
  {
    //find how far through the pre-liftoff period the robot is 
    const float preparationProgress =
        (preLiftoffEnd - state.cycleT) /
        PRE_LIFTOFF_PORTION;

    //smoothly increase the body shift as the leg gets closer to liftoff
    state.feedforwardInfluence =  smoothStep(preparationProgress);
  }
  else
  {
    state.feedforwardInfluence = 0.0f;
  }

  return state;
}


// increase the correction on the remaining stance leg if the opposing leg is swinging
float groupNormalization(float firstWeight,float secondWeight)
{
  const float total = firstWeight + secondWeight;

  // dont apply correction if neither leg is on ground
  if (total < 0.05f)
  {
    return 0.0f;
  }

  return clampFloat(2.0f / total, 1.0f, MAX_STANCE_NORMALIZATION);
}

//Forward and Turning Commands

//combine forward movement and turning for the left legs
float calculateLeftMotionCommand()
{
  return clampFloat(
      forwardCommand + turnCommand,
      -1.0f,
      1.0f);
}


//combine forward movement and turning for the right legs
float calculateRightMotionCommand()
{
  return clampFloat(
      forwardCommand - turnCommand,
      -1.0f,
      1.0f);
}


// get the step length this leg should use
float calculateStepLengthForLeg(uint8_t legIndex)
{
  //left and right legs can use different commands while turning
  const float sideCommand = isLeftLeg(legIndex) ? calculateLeftMotionCommand() : calculateRightMotionCommand();

  return DEFAULT_STEP_LENGTH * sideCommand;
}

//Feedback Calculations

// using the IMU roll angle to calculate a shoulder correction
float calculateImuRollCorrection()
{
  //roll stabilization is currently turned off during trot
  if (gaitMode == GaitMode::Trot)
  {
    return 0.0f;
  }

  //read roll and ignore small errors
  const float measuredRollDeg = applyDeadband(getRoll(),IMU_DEADBAND_DEG);

 //larger roll error gives a bigger correction
  float correctionDeg = ROLL_CORRECTION_SIGN * measuredRollDeg * ROLL_KP;

  //keep the correction within a safe range
  correctionDeg = clampFloat(correctionDeg, -MAX_ROLL_CORRECTION_DEG, MAX_ROLL_CORRECTION_DEG);

 //convert the correction from degrees to radians
  return correctionDeg *
      PI_F / 180.0f;
}


//use pitch angle and pitch rate to calculate an up/down foot correction
float calculateImuPitchCorrection()
{
  // pitch stabilization is currently turned off during trot
  if (gaitMode == GaitMode::Trot)
  {
    return 0.0f;
  }

  float pitchDeg = getPitch();
  float pitchRateDegPerSec = getPitchRate();

  //ignore very small pitch errors
  if (fabsf(pitchDeg) < IMU_DEADBAND_DEG)
  {
    pitchDeg = 0.0f;
  }

  //ignore very small pitch speeds
  if (fabsf(pitchRateDegPerSec) < PITCH_RATE_DEADBAND_DEG_PER_SEC)
  {
    pitchRateDegPerSec = 0.0f;
  }

  // Proportion(P) to correct based on how far the robot is tilted
  const float proportionalCorrection =
      pitchDeg *
      PITCH_KP_METERS_PER_DEG;

  // Derivative(D) to correct based on how quickly the robot is tipping
  float derivativeCorrection = pitchRateDegPerSec * PITCH_KD_METERS_PER_DEG_PER_SEC;

//limit the derivative section
  derivativeCorrection = clampFloat(derivativeCorrection, -MAX_PITCH_D_CORRECTION_M, MAX_PITCH_D_CORRECTION_M);

  //combine P and D control
  const float correctionMeters = PITCH_CORRECTION_SIGN *(proportionalCorrection + derivativeCorrection);

   /limit the final correction
  return clampFloat( correctionMeters, -MAX_PITCH_CORRECTION_M, MAX_PITCH_CORRECTION_M);
}

//Feed Forward for stance

//work out the X, Z, and shoulder correction for each leg
void calculateLegCorrections( const LegPhaseState phaseStates[4], LegCorrection corrections[4])
{
  //start every leg with no correction
  for (uint8_t i = 0; i < 4; ++i)
  {
    corrections[i] = {0.0f, 0.0f, 0.0f};
  }

  //to stabilize
  //front swing, shift forward
  //rear swing, shift backward

  //find how much the front legs need to be shifted 
  const float frontfeedforwardInfluence = phaseStates[FL].feedforwardInfluence + phaseStates[FR].feedforwardInfluence;

  //same for rear legs
  const float rearfeedforwardInfluence = phaseStates[BL].feedforwardInfluence + phaseStates[BR].feedforwardInfluence;

  //calculate the forward/backward shift
  const float feedforwardX = FEEDFORWARD_PITCH_DIRECTION * FEEDFORWARD_PITCH_SHIFT_M * (frontfeedforwardInfluence - rearfeedforwardInfluence);

  //shift all four foot targets together
  for (uint8_t i = 0; i < 4; ++i)
  {
    corrections[i].x = feedforwardX;
  }

//lateral shift
//same as forward and back just left and right here

  //find the planned shift from the left legs
  const float leftfeedforwardInfluence = phaseStates[FL].feedforwardInfluence + phaseStates[BL].feedforwardInfluence;

  //find the planned shift from the right legs
  const float rightfeedforwardInfluence = phaseStates[FR].feedforwardInfluence + phaseStates[BR].feedforwardInfluence;

  //calculate planned side-to-side correction
  const float feedforwardRoll = FEEDFORWARD_ROLL_DIRECTION * FEEDFORWARD_ROLL_RAD * (leftfeedforwardInfluence - rightfeedforwardInfluence);

  //imu PITCH on stance legs

  //get pitch correction from the IMU controller
  const float imuPitch = calculateImuPitchCorrection();

  //front and rear feet move opposite ways to correct body pitch
  const float desiredFrontZ = -imuPitch;

  const float desiredRearZ = imuPitch;

  //adjust front correction based on which front feet are on the ground
  const float frontNormalization = groupNormalization(phaseStates[FL].contactWeight,phaseStates[FR].contactWeight);

  //same thing for the rear feet
  const float rearNormalization = groupNormalization( phaseStates[BL].contactWeight,phaseStates[BR].contactWeight);

  //give pitch correction only to feet that are actually in stance
  corrections[FL].z = desiredFrontZ * phaseStates[FL].contactWeight * frontNormalization;

  corrections[FR].z = desiredFrontZ * phaseStates[FR].contactWeight * frontNormalization;

  corrections[BL].z = desiredRearZ * phaseStates[BL].contactWeight * rearNormalization;

  corrections[BR].z = desiredRearZ * phaseStates[BR].contactWeight * rearNormalization;

//IMU ROLL on stance legs
  //get the roll correction from the IMU
  const float imuRoll = calculateImuRollCorrection();

  //combine planned roll shifting with IMU feedback
  const float finalRoll = feedforwardRoll + imuRoll;

  //adjust correction based on how many left legs are in stance
  const float leftNormalization = groupNormalization( phaseStates[FL].contactWeight, phaseStates[BL].contactWeight);

  //same for the right side
  const float rightNormalization = groupNormalization( phaseStates[FR].contactWeight, phaseStates[BR].contactWeight);

  //left and right shoulders move opposite ways to correct roll
  corrections[FL].shoulder = -finalRoll * phaseStates[FL].contactWeight * leftNormalization;

  corrections[BL].shoulder = -finalRoll * phaseStates[BL].contactWeight * leftNormalization;

  corrections[FR].shoulder = finalRoll * phaseStates[FR].contactWeight * rightNormalization;

  corrections[BR].shoulder = finalRoll * phaseStates[BR].contactWeight * rightNormalization;
}

//IK

//calculate the hip and knee movement for one leg
LegAngles calculateGaitDelta(
    float sampleTime,
    float phaseOffset,
    const LegAngles &neutral,
    bool rightSide,
    const LegCorrection &correction,
    bool gaitActive,
    float stepLength)
{
  FootPosition gaitFoot{};

  //if walking, get the foot position from the gait trajectory
  if (gaitActive)
  {
    gaitFoot =
        calculateStepTrajectory(
            sampleTime,
            phaseOffset,
            gaitSpeed,
            zGround,
            DEFAULT_FOOT_X_CENTER,
            stepLength);
  }
  else
  {
    //if standing, keep the foot at its normal position
    gaitFoot.x = NEUTRAL_FOOT_X;
    gaitFoot.z = zGround;
  }

  //planned translation is part of the nominal body motion.
  gaitFoot.x += correction.x;

  //use IK to find the hip and knee angles for the normal gait position
  const LegAngles gaitAngles =
      solveLegIK(
          gaitFoot.x,
          gaitFoot.z);

  //make a copy so the IMU correction can be added separately
  FootPosition correctedFoot =
      gaitFoot;

  //IMU correction is applied at full strength.
  correctedFoot.z += correction.z;

  //solve IK again after adding the IMU correction
  const LegAngles correctedAngles = solveLegIK( correctedFoot.x, correctedFoot.z);

  float hipDelta = 0.0f;
  float kneeDelta = 0.0f;

  //fnd how far the walking pose is from the normal standing pose
  if (gaitActive)
  {
    hipDelta =(gaitAngles.hip - neutral.hip) * GAIT_SCALE;

    kneeDelta = -(gaitAngles.knee - neutral.knee) * GAIT_SCALE;
  }

  //add the extra movement caused by IMU stabilization
  hipDelta += correctedAngles.hip - gaitAngles.hip;

  kneeDelta -= correctedAngles.knee - gaitAngles.knee;

  //right legs are mirrored, so their directions need to be flipped
  if (rightSide)
  {
    hipDelta = -hipDelta;
    kneeDelta = -kneeDelta;
  }

  return {hipDelta, kneeDelta};
}


//Main Pose 

// calculate one complete pose for all four legs and send it to the servos
void commandCurrentGaitPose()
{
  //get the timing offset for each leg
  float phases[4];
  fillPhaseOffsets(phases);

  LegPhaseState phaseStates[4];

  if (gaitMode == GaitMode::Stand)
  {
    //every foot is counted as fully loaded in Stand.
    for (uint8_t i = 0; i < 4; ++i)
    {
      phaseStates[i] = {0.0f, 1.0f, 0.0f};
    }
  }
  else
  {
    //find the current stance/swing state of each leg
    for (uint8_t i = 0; i < 4; ++i)
    {
      phaseStates[i] =
          calculateLegPhaseState(
              gaitTime,
              phases[i]);
    }
  }

  //clculate IMU and planned body corrections for each leg
  LegCorrection corrections[4];
  calculateLegCorrections( phaseStates, corrections);

  //Find the hip and knee angles for the normal standing position
  const LegAngles neutral = solveLegIK( NEUTRAL_FOOT_X, zGround);

  //crawl and trot use the gait trajectory; Stand does not
  const bool gaitActive =
      gaitMode != GaitMode::Stand;

  //find each leg's step length based on forward and turn commands
  const float flStepLength = calculateStepLengthForLeg(FL);
  const float frStepLength = calculateStepLengthForLeg(FR);
  const float blStepLength = calculateStepLengthForLeg(BL);
  const float brStepLength = calculateStepLengthForLeg(BR);

  //calculate hip and knee commands for each leg
  const LegAngles fl =
      calculateGaitDelta(
          gaitTime,
          phases[FL],
          neutral,
          false,
          corrections[FL],
          gaitActive,
          flStepLength);

  const LegAngles fr =
      calculateGaitDelta(
          gaitTime,
          phases[FR],
          neutral,
          true,
          corrections[FR],
          gaitActive,
          frStepLength);

  const LegAngles bl =
      calculateGaitDelta(
          gaitTime,
          phases[BL],
          neutral,
          false,
          corrections[BL],
          gaitActive,
          blStepLength);

  const LegAngles br =
      calculateGaitDelta(
          gaitTime,
          phases[BR],
          neutral,
          true,
          corrections[BR],
          gaitActive,
          brStepLength);

  //put all 12 joint commands in servo order:
  //FL, FR, BL, BR with shoulder, hip, knee for each
  const float jointOffsets[NUM_SERVOS] =
  {
    corrections[FL].shoulder, fl.hip, fl.knee,
    corrections[FR].shoulder, fr.hip, fr.knee,
    corrections[BL].shoulder, bl.hip, bl.knee,
    corrections[BR].shoulder, br.hip, br.knee
  };

  //send the final joint offsets to the servo controller
  commandJointOffsetsRadians(jointOffsets);

  static unsigned long lastStatusPrintMs = 0;

  //print useful information every 250 ms
  if (millis() - lastStatusPrintMs >= 250)
  {
    lastStatusPrintMs = millis();

    Serial.print("Pitch=");
    Serial.print(getPitch(), 2);

    Serial.print(" Rate=");
    Serial.print(getPitchRate(), 2);

    Serial.print(" Fwd=");
    Serial.print(forwardCommand, 2);

    Serial.print(" Turn=");
    Serial.print(turnCommand, 2);

    Serial.print(" Support FL/FR/BL/BR=");
    Serial.print(phaseStates[FL].contactWeight, 2);
    Serial.print("/");
    Serial.print(phaseStates[FR].contactWeight, 2);
    Serial.print("/");
    Serial.print(phaseStates[BL].contactWeight, 2);
    Serial.print("/");
    Serial.println(phaseStates[BR].contactWeight, 2);
  }
}
}


//set the gait controller to its starting state
void initializeGaitController()
{
  gaitMode = GaitMode::Stand;
  gaitTime = 0.0f;
  nextUpdateMs = millis();
  commandNeutralPose();
}


//update robots gait every 20 ms
void updateGaitController()
{
  const unsigned long now = millis();

  //return if it is not time for the next gait update yet
  if (static_cast<long>(now - nextUpdateMs) < 0)
  {
    return;
  }

  //calculate and send the current robot pose
  commandCurrentGaitPose();

  //only move forward through the gait cycle while walking
  if (gaitMode != GaitMode::Stand)
  {
    gaitTime += TIME_STEP;
  }

  //next update time
  nextUpdateMs += FRAME_PERIOD_MS;

  // If the program falls far behind, reset the timing instead of trying to quickly run several old frames
  if (static_cast<unsigned long>(now - nextUpdateMs) >
      FRAME_PERIOD_MS * 2UL)
  {
    nextUpdateMs =
        now + FRAME_PERIOD_MS;
  }
}


//cange between Stand, Crawl, and Trot
void setGaitMode(GaitMode mode)
{
  if (mode != gaitMode)
  {
    gaitMode = mode;

    //restart the gait timing whenever the gait changes
    gaitTime = 0.0f;
  }
}


// return the current gait mode
GaitMode getGaitMode()
{
  return gaitMode;
}


//set gait speed and keep it inside the allowed range
void setGaitSpeed(float speed)
{
  gaitSpeed =clampFloat(speed, 0.1f, 10.0f);
}


//return the current gait speed
float getGaitSpeed()
{
  return gaitSpeed;
}


// Set the normal foot height and keep it inside the allowed range
void setGroundHeight(float newZGround)
{
  zGround = clampFloat(newZGround, -0.235f, -0.08f);
}


//return the current foot height
float getGroundHeight()
{
  return zGround;
}


//set forward/reverse movement from -1 to +1
void setForwardCommand(float value)
{
  forwardCommand = clampFloat(value, -1.0f, 1.0f);
}


//return the current forward command
float getForwardCommand()
{
  return forwardCommand;
}


// set turning from -1 to +1
void setTurnCommand(float value)
{
  turnCommand = clampFloat(value, -1.0f, 1.0f);
}


//return the current turn command
float getTurnCommand()
{
  return turnCommand;
}
