#ifndef GAIT_CONTROLLER_H
#define GAIT_CONTROLLER_H

#include <Arduino.h>
#include "gait_types.h"

void initializeGaitController();
void updateGaitController();

void setGaitMode(GaitMode mode);
GaitMode getGaitMode();

void setGaitSpeed(float speed);
float getGaitSpeed();

void setGroundHeight(float zGround);
float getGroundHeight();

void setForwardCommand(float value);
float getForwardCommand();

void setTurnCommand(float value);
float getTurnCommand();

#endif
