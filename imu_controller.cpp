#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#include "imu_controller.h"

namespace
{
constexpr float PI_F = 3.14159265358979323846f;
constexpr float RAD_TO_DEG_F = 180.0f / PI_F;

constexpr float COMPLEMENTARY_ALPHA = 0.98f;//98% gyro based orientation, 2% accelerometer
constexpr float OUTPUT_FILTER_ALPHA = 0.60f;//low pass filter, in middle of smooth and fast signal reading
constexpr float RATE_FILTER_ALPHA = 0.25f;//how much filtered velo moves to last filtered measurement, smooth
constexpr float ANGLE_DEADBAND_DEG = 0.15f;//anything under this value is = 0, keeps out noise
constexpr uint16_t CALIBRATION_SAMPLES = 300;//calibration samples to establish level
constexpr float MAX_DT_SECONDS = 0.05f;//maximum time for gyro integration, prevents huge dt values, as gyrorate*dt gets oritentation

Adafruit_MPU6050 mpu;

float rollDeg = 0.0f;
float pitchDeg = 0.0f;

float filteredRollDeg = 0.0f;
float filteredPitchDeg = 0.0f;

float rollZeroDeg = 0.0f;
float pitchZeroDeg = 0.0f;

float gyroXBias = 0.0f;
float gyroYBias = 0.0f;

float rollRateDegPerSec = 0.0f;
float pitchRateDegPerSec = 0.0f;

float filteredRollRateDegPerSec = 0.0f;
float filteredPitchRateDegPerSec = 0.0f;

float rollRateHistory[3] = {0.0f, 0.0f, 0.0f};
float pitchRateHistory[3] = {0.0f, 0.0f, 0.0f};
uint8_t rateHistoryIndex = 0;

unsigned long previousMicros = 0;
unsigned long lastPrintMs = 0;

float normalizeAngle180(float angleDeg)
{
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

//gets rid of very small measurements
float applyDeadband(float value, float deadband)
{
  return fabsf(value) < deadband ? 0.0f : value;
  //if magnitude is smaller than the deadband, return 0
}

//take three measurements and return the middle value, prevents outliers
float medianOfThree(float a, float b, float c)
{
  if (a > b) { const float t = a; a = b; b = t; }
  if (b > c) { const float t = b; b = c; c = t; }
  if (a > b) { const float t = a; a = b; b = t; }
  return b;
}

//measuring acceleration, using gravity as reference
void calculateAccelerometerAngles( const sensors_event_t &accel,float &rollOutputDeg,float &pitchOutputDeg)
{
  //roll comes from the y and z components of gravity
  const float rawRollDeg =
      atan2f(accel.acceleration.y, accel.acceleration.z) *
      RAD_TO_DEG_F;

   //pitch comes from the X compared to the combined Y/Z direction
  const float rawPitchDeg =
      atan2f(
          -accel.acceleration.x,
          sqrtf(
              accel.acceleration.y * accel.acceleration.y +
              accel.acceleration.z * accel.acceleration.z)) *
      RAD_TO_DEG_F;

  rollOutputDeg =
      normalizeAngle180(rawRollDeg - 180.0f);

  pitchOutputDeg = rawPitchDeg;
}

void resetFilters()
{
  rollDeg = 0.0f;
  pitchDeg = 0.0f;
  filteredRollDeg = 0.0f;
  filteredPitchDeg = 0.0f;
  rollRateDegPerSec = 0.0f;
  pitchRateDegPerSec = 0.0f;
  filteredRollRateDegPerSec = 0.0f;
  filteredPitchRateDegPerSec = 0.0f;

  for (uint8_t i = 0; i < 3; ++i)
  {
    rollRateHistory[i] = 0.0f;
    pitchRateHistory[i] = 0.0f;
  }

  rateHistoryIndex = 0;
  previousMicros = micros();
}

void calibrateIMU()
{
  Serial.println("Keep robot still and level: calibrating IMU...");

  float rollSum = 0.0f;
  float pitchSum = 0.0f;
  float gyroXSum = 0.0f;
  float gyroYSum = 0.0f;

  for (uint16_t sample = 0; sample < CALIBRATION_SAMPLES; ++sample)
  {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(&accel, &gyro, &temperature);

    float accelRollDeg = 0.0f;
    float accelPitchDeg = 0.0f;

    calculateAccelerometerAngles(
        accel,
        accelRollDeg,
        accelPitchDeg);

    rollSum += accelRollDeg;
    pitchSum += accelPitchDeg;

    gyroXSum += gyro.gyro.x * RAD_TO_DEG_F;
    gyroYSum += gyro.gyro.y * RAD_TO_DEG_F;

    delay(5);
  }

  rollZeroDeg =
      rollSum / static_cast<float>(CALIBRATION_SAMPLES);

  pitchZeroDeg =
      pitchSum / static_cast<float>(CALIBRATION_SAMPLES);

  gyroXBias =
      gyroXSum / static_cast<float>(CALIBRATION_SAMPLES);

  gyroYBias =
      gyroYSum / static_cast<float>(CALIBRATION_SAMPLES);

  resetFilters();

  Serial.print("Roll zero: ");
  Serial.println(rollZeroDeg, 3);

  Serial.print("Pitch zero: ");
  Serial.println(pitchZeroDeg, 3);

  Serial.print("Gyro X bias: ");
  Serial.println(gyroXBias, 3);

  Serial.print("Gyro Y bias: ");
  Serial.println(gyroYBias, 3);

  Serial.println("IMU calibration complete");
}
}

void initializeIMU()
{
  if (!mpu.begin())
  {
    Serial.println("MPU6050 NOT FOUND");
    while (true) delay(100);
  }

  Serial.println("MPU6050 connected");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  calibrateIMU();
}

void updateIMU()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temperature;

  mpu.getEvent(&accel, &gyro, &temperature);

  const unsigned long nowMicros = micros();

  float dtSeconds =
      static_cast<float>(nowMicros - previousMicros) /
      1000000.0f;

  previousMicros = nowMicros;

  if (dtSeconds <= 0.0f) return;
  if (dtSeconds > MAX_DT_SECONDS) dtSeconds = MAX_DT_SECONDS;

  float accelRollDeg = 0.0f;
  float accelPitchDeg = 0.0f;

  calculateAccelerometerAngles(
      accel,
      accelRollDeg,
      accelPitchDeg);

  accelRollDeg =
      normalizeAngle180(accelRollDeg - rollZeroDeg);

  accelPitchDeg -= pitchZeroDeg;

  rollRateDegPerSec =
      gyro.gyro.x * RAD_TO_DEG_F -
      gyroXBias;

  pitchRateDegPerSec =
      gyro.gyro.y * RAD_TO_DEG_F -
      gyroYBias;

  rollRateHistory[rateHistoryIndex] =
      rollRateDegPerSec;

  pitchRateHistory[rateHistoryIndex] =
      pitchRateDegPerSec;

  rateHistoryIndex =
      (rateHistoryIndex + 1U) % 3U;

  const float medianRollRate =
      medianOfThree(
          rollRateHistory[0],
          rollRateHistory[1],
          rollRateHistory[2]);

  const float medianPitchRate =
      medianOfThree(
          pitchRateHistory[0],
          pitchRateHistory[1],
          pitchRateHistory[2]);

  filteredRollRateDegPerSec +=
      RATE_FILTER_ALPHA *
      (medianRollRate - filteredRollRateDegPerSec);

  filteredPitchRateDegPerSec +=
      RATE_FILTER_ALPHA *
      (medianPitchRate - filteredPitchRateDegPerSec);

  const float gyroRollPrediction =
      rollDeg +
      rollRateDegPerSec * dtSeconds;

  const float gyroPitchPrediction =
      pitchDeg +
      pitchRateDegPerSec * dtSeconds;

  rollDeg =
      COMPLEMENTARY_ALPHA * gyroRollPrediction +
      (1.0f - COMPLEMENTARY_ALPHA) * accelRollDeg;

  pitchDeg =
      COMPLEMENTARY_ALPHA * gyroPitchPrediction +
      (1.0f - COMPLEMENTARY_ALPHA) * accelPitchDeg;

  filteredRollDeg +=
      OUTPUT_FILTER_ALPHA *
      (rollDeg - filteredRollDeg);

  filteredPitchDeg +=
      OUTPUT_FILTER_ALPHA *
      (pitchDeg - filteredPitchDeg);

  filteredRollDeg =
      applyDeadband(filteredRollDeg, ANGLE_DEADBAND_DEG);

  filteredPitchDeg =
      applyDeadband(filteredPitchDeg, ANGLE_DEADBAND_DEG);

  const unsigned long nowMs = millis();

  if (nowMs - lastPrintMs >= 200)
  {
    lastPrintMs = nowMs;

    Serial.print("Roll: ");
    Serial.print(filteredRollDeg, 2);

    Serial.print("   Pitch: ");
    Serial.print(filteredPitchDeg, 2);

    Serial.print("   Pitch rate: ");
    Serial.println(filteredPitchRateDegPerSec, 2);
  }
}

float getRoll()
{
  return filteredRollDeg;
}

float getPitch()
{
  return filteredPitchDeg;
}

float getRollRate()
{
  return filteredRollRateDegPerSec;
}

float getPitchRate()
{
  return filteredPitchRateDegPerSec;
}
