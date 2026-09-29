#ifndef GAIT_TYPES_H
#define GAIT_TYPES_H

#include <Arduino.h>

struct FootPosition
{
  float x;
  float z;
};

struct LegAngles
{
  float hip;
  float knee;
};

enum class GaitMode : uint8_t
{
  Stand = 0,
  Crawl = 1,
  Trot = 2
};

#endif
