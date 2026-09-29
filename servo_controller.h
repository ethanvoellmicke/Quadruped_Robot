#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>

constexpr uint8_t NUM_SERVOS = 12;

void initializeServoController();
void commandNeutralPose();
void commandJointOffsetsRadians(const float jointOffsetsRad[NUM_SERVOS]);
void getLastServoAngles(float outputAnglesDeg[NUM_SERVOS]);

#endif
