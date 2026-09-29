

#ifndef IMU_CONTROLLER_H
#define IMU_CONTROLLER_H

void initializeIMU();
void updateIMU();

float getRoll();
float getPitch();

float getRollRate();
float getPitchRate();

#endif
