////////////////////////////////////////////////////////////////////////////////////////////////
// includes
#include "include/joint_defs.h"

#include "src/mcu_util/mcu_util.h"
#include "src/state_estimation/state_estimation.h"
#include "src/motor_control/motor_control.h"
#include "src/locomotion/locomotion.h"

////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  initTeensy();
  initActuators();
  
  homeLeggedRobot();
  openDataFile();
  standUp();
}

void loop() {
  while (!stop_signal && gait_cycles < 10) {
    handleODriveCANMsg();
    // parseJetsonSerial();
    parseTeensySerial();
    // updateTrajectory();
    updateStates();
    updateLocomotion();
    updateMotorCommands();
    sendTelemetry();
  }

  stopActuators();    // only use if the legs are actually non-backdrviable; if not, the robot will drop when the motors turn off
  closeDataFile();
  delay(10000000);
}