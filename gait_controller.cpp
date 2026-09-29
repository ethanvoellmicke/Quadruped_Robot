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

constexpr uint16_t FRAME_PERIOD_MS = 20;
constexpr float TIME_STEP =
    static_cast<float>(FRAME_PERIOD_MS) / 1000.0f;

constexpr float GAIT_SCALE = 0.40f;
constexpr float NEUTRAL_FOOT_X = 0.040f;

// -----------------------------------------------------
// Crawl timing and feedforward
// -----------------------------------------------------

constexpr float SWING_PORTION = 0.25f;

// Begin unloading a leg before liftoff.
// 0.12 means the final 12% of the cycle is used to
// transfer weight away from that leg.
constexpr float PRE_LIFTOFF_PORTION = 0.12;

// Gradually reload the foot after touchdown.
constexpr float POST_TOUCHDOWN_PORTION = 0.04f;

// Planned fore/aft body shift before and during swing.
constexpr float FEEDFORWARD_PITCH_SHIFT_M = 0.004f;//.008

// Keep the direction that helped your rear-leg stability.
// Flip this sign only if the body shifts the wrong way.
constexpr float FEEDFORWARD_PITCH_DIRECTION = -1.0f;

// Optional planned side shift through the shoulders.
// Start at zero because your IMU roll response already works well.
constexpr float FEEDFORWARD_ROLL_DEG = 0.0f;//2
constexpr float FEEDFORWARD_ROLL_RAD =
    FEEDFORWARD_ROLL_DEG * PI_F / 180.0f;

// Flip only if a future nonzero feedforward roll moves toward
// the swing leg instead of away from it.
constexpr float FEEDFORWARD_ROLL_DIRECTION = -1.0f;

// When one of two legs on an end/side is swinging, modestly
// increase the correction assigned to the remaining stance leg.
constexpr float MAX_STANCE_NORMALIZATION = 1.50f;

// -----------------------------------------------------
// IMU stabilization tuning
// -----------------------------------------------------

constexpr float IMU_DEADBAND_DEG = 0.75f;

constexpr float ROLL_KP = 1.0f;
constexpr float MAX_ROLL_CORRECTION_DEG = 12.0f;

constexpr float PITCH_KP_METERS_PER_DEG = 0.00012f;
constexpr float PITCH_KD_METERS_PER_DEG_PER_SEC = 0.00008f;

constexpr float PITCH_RATE_DEADBAND_DEG_PER_SEC = 2.0f;

constexpr float MAX_PITCH_D_CORRECTION_M = 0.0035f;
constexpr float MAX_PITCH_CORRECTION_M = 0.010f;

constexpr float ROLL_CORRECTION_SIGN = -1.0f;
constexpr float PITCH_CORRECTION_SIGN = 1.0f;

// -----------------------------------------------------
// Controller state
// -----------------------------------------------------

GaitMode gaitMode = GaitMode::Stand;

float gaitTime = 0.0f;
float gaitSpeed = 2.0f;
float zGround = -0.22f;

// Normalized motion commands.
// forward: -1 reverse, 0 no translation, +1 forward
// turn:    -1 turn one way, 0 straight, +1 turn the other way
float forwardCommand = 1.0f;
float turnCommand = 0.0f;

unsigned long nextUpdateMs = 0;

enum LegIndex : uint8_t
{
  FL = 0,
  FR = 1,
  BL = 2,
  BR = 3
};

struct LegPhaseState
{
  float cycleT;

  // 1 while physically in stance, 0 while physically swinging.
  float contactWeight;

  // Ramps up before liftoff and stays high during swing.
  // Used only for feedforward body shifting.
  float feedforwardInfluence;
};

struct LegCorrection
{
  float x;
  float z;
  float shoulder;
};

// -----------------------------------------------------
// Helpers
// -----------------------------------------------------

float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float smoothStep(float t)
{
  t = clampFloat(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

float applyDeadband(float value, float deadband)
{
  return fabsf(value) < deadband ? 0.0f : value;
}

bool isLeftLeg(uint8_t legIndex)
{
  return legIndex == FL || legIndex == BL;
}

void fillPhaseOffsets(float phases[4])
{
  if (gaitMode == GaitMode::Trot)
  {
    phases[FL] = 0.0f;
    phases[FR] = PI_F;
    phases[BL] = PI_F;
    phases[BR] = 0.0f;
    return;
  }

  // With the negative time direction inside foot_trajectory.cpp,
  // these offsets produce:
  // FL -> BR -> BL -> FR.
  phases[FL] = 0.0f;
  phases[FR] = 3.0f * PI_F / 2.0f;
  phases[BL] = PI_F;
  phases[BR] = PI_F / 2.0f;
}

float calculateCycleT(
    float sampleTime,
    float phaseOffset)
{
  float phase =
      fmodf(
          -gaitSpeed * sampleTime + phaseOffset,
          TWO_PI_F);

  if (phase < 0.0f)
  {
    phase += TWO_PI_F;
  }

  return phase / TWO_PI_F;
}

LegPhaseState calculateLegPhaseState(
    float sampleTime,
    float phaseOffset)
{
  LegPhaseState state{};

  state.cycleT =
      calculateCycleT(
          sampleTime,
          phaseOffset);

  /*
   * Your cycle runs backward:
   *
   * 1.00 -> touchdown
   * decreasing through stance
   * 0.25 -> liftoff
   * 0.25 to 0.00 -> swing
   */

  const bool isSwinging =
      state.cycleT < SWING_PORTION;

  // Only the physically swinging leg loses IMU correction.
  state.contactWeight =
      isSwinging ? 0.0f : 1.0f;

  if (isSwinging)
  {
    state.feedforwardInfluence = 1.0f;
    return state;
  }

  /*
   * Begin feedforward before liftoff, but do not remove
   * stance-leg IMU control during this preparation period.
   */
  const float preLiftoffEnd =
      SWING_PORTION +
      PRE_LIFTOFF_PORTION;

  if (state.cycleT < preLiftoffEnd)
  {
    const float preparationProgress =
        (preLiftoffEnd - state.cycleT) /
        PRE_LIFTOFF_PORTION;

    state.feedforwardInfluence =
        smoothStep(preparationProgress);
  }
  else
  {
    state.feedforwardInfluence = 0.0f;
  }

  return state;
}

float groupNormalization(
    float firstWeight,
    float secondWeight)
{
  const float total =
      firstWeight + secondWeight;

  if (total < 0.05f)
  {
    return 0.0f;
  }

  return clampFloat(
      2.0f / total,
      1.0f,
      MAX_STANCE_NORMALIZATION);
}

// -----------------------------------------------------
// Forward / turning command mixing
// -----------------------------------------------------

float calculateLeftMotionCommand()
{
  return clampFloat(
      forwardCommand + turnCommand,
      -1.0f,
      1.0f);
}

float calculateRightMotionCommand()
{
  return clampFloat(
      forwardCommand - turnCommand,
      -1.0f,
      1.0f);
}

float calculateStepLengthForLeg(uint8_t legIndex)
{
  const float sideCommand =
      isLeftLeg(legIndex)
          ? calculateLeftMotionCommand()
          : calculateRightMotionCommand();

  return DEFAULT_STEP_LENGTH * sideCommand;
}

// -----------------------------------------------------
// Feedback calculations
// -----------------------------------------------------

float calculateImuRollCorrection()
{
  if (gaitMode == GaitMode::Trot)
  {
    return 0.0f;
  }

  const float measuredRollDeg =
      applyDeadband(
          getRoll(),
          IMU_DEADBAND_DEG);

  float correctionDeg =
      ROLL_CORRECTION_SIGN *
      measuredRollDeg *
      ROLL_KP;

  correctionDeg =
      clampFloat(
          correctionDeg,
          -MAX_ROLL_CORRECTION_DEG,
          MAX_ROLL_CORRECTION_DEG);

  return correctionDeg *
      PI_F / 180.0f;
}

float calculateImuPitchCorrection()
{
  if (gaitMode == GaitMode::Trot)
  {
    return 0.0f;
  }

  float pitchDeg = getPitch();
  float pitchRateDegPerSec = getPitchRate();

  if (fabsf(pitchDeg) < IMU_DEADBAND_DEG)
  {
    pitchDeg = 0.0f;
  }

  if (fabsf(pitchRateDegPerSec) <
      PITCH_RATE_DEADBAND_DEG_PER_SEC)
  {
    pitchRateDegPerSec = 0.0f;
  }

  const float proportionalCorrection =
      pitchDeg *
      PITCH_KP_METERS_PER_DEG;

  float derivativeCorrection =
      pitchRateDegPerSec *
      PITCH_KD_METERS_PER_DEG_PER_SEC;

  derivativeCorrection =
      clampFloat(
          derivativeCorrection,
          -MAX_PITCH_D_CORRECTION_M,
          MAX_PITCH_D_CORRECTION_M);

  const float correctionMeters =
      PITCH_CORRECTION_SIGN *
      (
        proportionalCorrection +
        derivativeCorrection
      );

  return clampFloat(
      correctionMeters,
      -MAX_PITCH_CORRECTION_M,
      MAX_PITCH_CORRECTION_M);
}

// -----------------------------------------------------
// Feedforward and stance-only correction distribution
// -----------------------------------------------------

void calculateLegCorrections(
    const LegPhaseState phaseStates[4],
    LegCorrection corrections[4])
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    corrections[i] = {0.0f, 0.0f, 0.0f};
  }

  // -------- Feedforward fore/aft shift --------
  //
  // Front swing -> shift body backward.
  // Rear swing  -> shift body forward.
  //
  // feedforwardInfluence begins increasing before liftoff,
  // so the body is already moving before the foot leaves.
  const float frontfeedforwardInfluence =
      phaseStates[FL].feedforwardInfluence +
      phaseStates[FR].feedforwardInfluence;

  const float rearfeedforwardInfluence =
      phaseStates[BL].feedforwardInfluence +
      phaseStates[BR].feedforwardInfluence;

  const float feedforwardX =
      FEEDFORWARD_PITCH_DIRECTION *
      FEEDFORWARD_PITCH_SHIFT_M *
      (frontfeedforwardInfluence - rearfeedforwardInfluence);

  for (uint8_t i = 0; i < 4; ++i)
  {
    corrections[i].x = feedforwardX;
  }

  // -------- Feedforward lateral shift --------
  //
  // Left swing -> shift body right.
  // Right swing -> shift body left.
  const float leftfeedforwardInfluence =
      phaseStates[FL].feedforwardInfluence +
      phaseStates[BL].feedforwardInfluence;

  const float rightfeedforwardInfluence =
      phaseStates[FR].feedforwardInfluence +
      phaseStates[BR].feedforwardInfluence;

  const float feedforwardRoll =
      FEEDFORWARD_ROLL_DIRECTION *
      FEEDFORWARD_ROLL_RAD *
      (leftfeedforwardInfluence - rightfeedforwardInfluence);

  // -------- IMU pitch: stance legs only --------
  const float imuPitch =
      calculateImuPitchCorrection();

  const float desiredFrontZ =
      -imuPitch;

  const float desiredRearZ =
      imuPitch;

  const float frontNormalization =
      groupNormalization(
          phaseStates[FL].contactWeight,
          phaseStates[FR].contactWeight);

  const float rearNormalization =
      groupNormalization(
          phaseStates[BL].contactWeight,
          phaseStates[BR].contactWeight);

  corrections[FL].z =
      desiredFrontZ *
      phaseStates[FL].contactWeight *
      frontNormalization;

  corrections[FR].z =
      desiredFrontZ *
      phaseStates[FR].contactWeight *
      frontNormalization;

  corrections[BL].z =
      desiredRearZ *
      phaseStates[BL].contactWeight *
      rearNormalization;

  corrections[BR].z =
      desiredRearZ *
      phaseStates[BR].contactWeight *
      rearNormalization;

  // -------- IMU roll: stance legs only --------
  const float imuRoll =
      calculateImuRollCorrection();

  const float finalRoll =
      feedforwardRoll + imuRoll;

  const float leftNormalization =
      groupNormalization(
          phaseStates[FL].contactWeight,
          phaseStates[BL].contactWeight);

  const float rightNormalization =
      groupNormalization(
          phaseStates[FR].contactWeight,
          phaseStates[BR].contactWeight);

  corrections[FL].shoulder =
      -finalRoll *
      phaseStates[FL].contactWeight *
      leftNormalization;

  corrections[BL].shoulder =
      -finalRoll *
      phaseStates[BL].contactWeight *
      leftNormalization;

  corrections[FR].shoulder =
      finalRoll *
      phaseStates[FR].contactWeight *
      rightNormalization;

  corrections[BR].shoulder =
      finalRoll *
      phaseStates[BR].contactWeight *
      rightNormalization;
}

// -----------------------------------------------------
// IK
// -----------------------------------------------------

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
    gaitFoot.x = NEUTRAL_FOOT_X;
    gaitFoot.z = zGround;
  }

  // Planned translation is part of the nominal body motion.
  gaitFoot.x += correction.x;

  const LegAngles gaitAngles =
      solveLegIK(
          gaitFoot.x,
          gaitFoot.z);

  FootPosition correctedFoot =
      gaitFoot;

  // IMU correction is applied at full strength.
  correctedFoot.z += correction.z;

  const LegAngles correctedAngles =
      solveLegIK(
          correctedFoot.x,
          correctedFoot.z);

  float hipDelta = 0.0f;
  float kneeDelta = 0.0f;

  if (gaitActive)
  {
    hipDelta =
        (gaitAngles.hip - neutral.hip) *
        GAIT_SCALE;

    kneeDelta =
        -(gaitAngles.knee - neutral.knee) *
        GAIT_SCALE;
  }

  hipDelta +=
      correctedAngles.hip -
      gaitAngles.hip;

  kneeDelta -=
      correctedAngles.knee -
      gaitAngles.knee;

  if (rightSide)
  {
    hipDelta = -hipDelta;
    kneeDelta = -kneeDelta;
  }

  return {hipDelta, kneeDelta};
}

// -----------------------------------------------------
// Main pose command
// -----------------------------------------------------

void commandCurrentGaitPose()
{
  float phases[4];
  fillPhaseOffsets(phases);

  LegPhaseState phaseStates[4];

  if (gaitMode == GaitMode::Stand)
  {
    // Every foot is considered fully loaded in Stand.
    for (uint8_t i = 0; i < 4; ++i)
    {
      phaseStates[i] = {0.0f, 1.0f, 0.0f};
    }
  }
  else
  {
    for (uint8_t i = 0; i < 4; ++i)
    {
      phaseStates[i] =
          calculateLegPhaseState(
              gaitTime,
              phases[i]);
    }
  }

  LegCorrection corrections[4];
  calculateLegCorrections(
      phaseStates,
      corrections);

  const LegAngles neutral =
      solveLegIK(
          NEUTRAL_FOOT_X,
          zGround);

  const bool gaitActive =
      gaitMode != GaitMode::Stand;

  const float flStepLength = calculateStepLengthForLeg(FL);
  const float frStepLength = calculateStepLengthForLeg(FR);
  const float blStepLength = calculateStepLengthForLeg(BL);
  const float brStepLength = calculateStepLengthForLeg(BR);

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

  const float jointOffsets[NUM_SERVOS] =
  {
    corrections[FL].shoulder, fl.hip, fl.knee,
    corrections[FR].shoulder, fr.hip, fr.knee,
    corrections[BL].shoulder, bl.hip, bl.knee,
    corrections[BR].shoulder, br.hip, br.knee
  };

  commandJointOffsetsRadians(jointOffsets);

  static unsigned long lastStatusPrintMs = 0;

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

void initializeGaitController()
{
  gaitMode = GaitMode::Stand;
  gaitTime = 0.0f;
  nextUpdateMs = millis();
  commandNeutralPose();
}

void updateGaitController()
{
  const unsigned long now = millis();

  if (static_cast<long>(now - nextUpdateMs) < 0)
  {
    return;
  }

  commandCurrentGaitPose();

  if (gaitMode != GaitMode::Stand)
  {
    gaitTime += TIME_STEP;
  }

  nextUpdateMs += FRAME_PERIOD_MS;

  if (static_cast<unsigned long>(now - nextUpdateMs) >
      FRAME_PERIOD_MS * 2UL)
  {
    nextUpdateMs =
        now + FRAME_PERIOD_MS;
  }
}

void setGaitMode(GaitMode mode)
{
  if (mode != gaitMode)
  {
    gaitMode = mode;
    gaitTime = 0.0f;
  }
}

GaitMode getGaitMode()
{
  return gaitMode;
}

void setGaitSpeed(float speed)
{
  gaitSpeed =
      clampFloat(
          speed,
          0.1f,
          10.0f);
}

float getGaitSpeed()
{
  return gaitSpeed;
}

void setGroundHeight(float newZGround)
{
  zGround =
      clampFloat(
          newZGround,
          -0.235f,
          -0.08f);
}

float getGroundHeight()
{
  return zGround;
}

void setForwardCommand(float value)
{
  forwardCommand = clampFloat(value, -1.0f, 1.0f);
}

float getForwardCommand()
{
  return forwardCommand;
}

void setTurnCommand(float value)
{
  turnCommand = clampFloat(value, -1.0f, 1.0f);
}

float getTurnCommand()
{
  return turnCommand;
}
