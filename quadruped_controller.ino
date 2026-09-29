
#include "servo_controller.h"
#include "gait_controller.h"
#include "command_receiver.h"
#include "imu_controller.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Quadruped local gait controller starting");

  initializeServoController();
  initializeGaitController();
  initializeCommandReceiver();
  initializeIMU();

  Serial.println(
    "Ready. Commands: stand, crawl, trot, speed <value>, height <value>, status"
  );
}

void loop()
{
  // Keep gait updates frequent so communication and IMU reads
  // do not interrupt the 20 ms motion scheduler.
  updateGaitController();

  updateCommandReceiver();

  updateGaitController();

  updateIMU();

  updateGaitController();
}
