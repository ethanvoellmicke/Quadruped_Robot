#ifndef FOOT_TRAJECTORY_H
#define FOOT_TRAJECTORY_H

#include "gait_types.h"

constexpr float DEFAULT_FOOT_X_CENTER = 0.06f;
constexpr float DEFAULT_STEP_LENGTH = 0.12f;
constexpr float DEFAULT_STEP_HEIGHT = 0.02f;
constexpr float DEFAULT_STANCE_DEPTH = 0.004f;

FootPosition calculateStepTrajectory(
  float timeSeconds,
  float phaseOffset,
  float speed,
  float zGround,
  float xCenter = DEFAULT_FOOT_X_CENTER,
  float stepLength = DEFAULT_STEP_LENGTH,
  float stepHeight = DEFAULT_STEP_HEIGHT,
  float stanceDepth = DEFAULT_STANCE_DEPTH
);

#endif
