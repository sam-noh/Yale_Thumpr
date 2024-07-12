////////////////////////////////////////////////////////////////////////////////////////////////
// includes
#include "include/joint_defs.h"

#include "src/mcu_util/mcu_util.h"
#include "src/state_estimation/state_estimation.h"
#include "src/motor_control/motor_control.h"
#include "src/locomotion/locomotion.h"

////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  z_body_nominal = 300;             // body height in mm
  leg_swing_percent = 0.3;          // swing leg stroke as a percentage of the leg stroke at contact; [0, 1.0]; smaller value means more retraction
  int number_of_steps = 2;          // number of touchdown steps
  uint32_t dt_btwn_steps_ms = 500;  // milliseconds to wait between touchdowns

  initTeensy();
  initActuators();
  
  homeLeggedRobot();
  openDataFile();
  standUp();

  Serial.println("Standing test");

  elapsedMillis timer_1;
  while(timer_1 < 500) {
    updateFunctions();
  }

  for (auto i = 0; i < number_of_steps; ++i) {
    // touchdown
    SERIAL_USB.println("touchdown");
    actuation_phase = ActuationPhases::kTouchDown;
    digitalWrite(LED, LOW);
    updateTouchdown(gait_phase, kVelLegMaxContact);
    while (!isInContact[gait_phase * 2] || !isInContact[gait_phase * 2 + 1]) {
      updateFunctions();
      updateTouchdownTorque(gait_phase);
      updateStanceBodyTorque(gait_phase);
    }

    // swing
    SERIAL_USB.println("swing phase");
    actuation_phase = ActuationPhases::kRetractLeg;
    updateRetract();
    resetBodyLegContactState(gait_phase);
    while (!motors[gait_phase * 2].states_.holding && !motors[gait_phase * 2 + 1].states_.holding) {
      updateFunctions();
    }

    elapsedMillis timer_2;
    while(timer_2 < dt_btwn_steps_ms) {
      updateFunctions();
    }
  }

  stopActuators();
  closeDataFile();
}

void loop() {
}

void updateFunctions() {
  handleODriveCANMsg();
  updateStates();
  updateMotorCommands();
  sendTelemetry();

  if (stop_signal) {
    stopActuators();    // only use if the legs are actually non-backdrviable; if not, the robot will drop when the motors turn off
    closeDataFile();
    delay(10000000);
  }
}
