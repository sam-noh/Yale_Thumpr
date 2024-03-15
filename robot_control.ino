////////////////////////////////////////////////////////////////////////////////////////////////
// includes
#include "include\joint_defs.h"

#include "src\mcu_util\mcu_util.h"
#include "src\state_estimation\state_estimation.h"
#include "src\motor_control\motor_control.h"
#include "src\locomotion\locomotion.h"

////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  z_body_nominal = 230;
  leg_swing_percent = 0.6;

  initTeensy();
  initActuators();
  
  homeLeggedRobot();
  zeroIMUReading();
  standUp();

  Serial.println("Standing test");

  elapsedMillis timer_1;
  while(timer_1 < 500) {
    updateFunctions();
  }

  for (auto i = 0; i < 2; ++i) {
    // touchdown
    updateMotorsForTouchdown();
    while (!inContact[gait_phase * 2] && !inContact[gait_phase * 2 + 1]) {
      updateFunctions();
    }

    // swing
    updateMotorsForSwing();
    while (!motors[gait_phase * 2].states_.holding && !motors[gait_phase * 2 + 1].states_.holding) {
      updateFunctions();
    }

    elapsedMillis timer_2;
    while(timer_2 < 400) {
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
  parseTeensySerial();
  updateStates();
  updateMotorCommands();
  sendTelemetry();

  if (stop_signal) {
    stopActuators();    // only use if the legs are actually non-backdrviable; if not, the robot will drop when the motors turn off
    closeDataFile();
    delay(10000000);
  }
}

void updateMotorsForTouchdown() {
  actuation_phase = ActuationPhases::kTouchDown;

  for (auto i = 0; i < 2; ++i) {
    motors[gait_phase * 2 + i].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kTorqueControl;
    motors[gait_phase * 2 + i].states_.velocity_limit = kVelLegMaxContact;
    motors[gait_phase * 2 + i].states_.tau_d = touchdown_torque[gait_phase * 2 + i][1];
  }
}

void updateMotorsForSwing() {
  actuation_phase = ActuationPhases::kRetractLeg;

  for (auto i = 0; i < 2; ++i) {
    q_leg_contact[i] = motors[gait_phase*2 + i].states_.q;      // remember the leg motor position at ground contact
    float q_leg_retract = q_leg_contact[i]*leg_swing_percent;   // nominal swing leg setpoint; NOT necessarily equal to the actual setpoint

    if (fabs(q[gait_phase * 4 + i * 2] - q[gait_phase * 4 + i * 2 + 1]) > kDqUnevenTerrain) { // if the previous stance legs are standing on uneven ground
      float q_max = max(q[gait_phase * 4 + i * 2], q[gait_phase * 4 + i * 2 + 1]);            // calculate the additional leg stroke due to uneven terrain
      float dq = q_max - motors[gait_phase * 2 + i].states_.q;
      
      motors[gait_phase * 2 + i].states_.q_d = max(kQLegMin, q_leg_retract - dq); // retract more by dq to ensure proper ground clearance

    } else {  
      motors[gait_phase * 2 + i].states_.q_d = q_leg_retract;  // if even terrain, use the nominal swing leg setpoint
    }

    q_leg_swing[i] = motors[gait_phase * 2 + i].states_.q_d;
    motors[gait_phase * 2 + i].states_.holding = false;
    motors[gait_phase * 2 + i].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
    motors[gait_phase * 2 + i].states_.trap_traj_vel_limit = kVelLegTrajSwing;
    motors[gait_phase * 2 + i].states_.trap_traj_accel_limit = kAccelLegTrajSwing;
    motors[gait_phase * 2 + i].states_.trap_traj_decel_limit = kDecelLegTrajSwing;
    inContact[gait_phase * 2 + i] = false;
  }
}
