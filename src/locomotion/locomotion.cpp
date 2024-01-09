#include "locomotion.h"
#include "..\..\include\joint_defs.h"
#include "..\mcu_util\mcu_util.h"
#include "..\motor_control\motor_control.h"
#include "..\state_estimation\state_estimation.h"

std::vector<std::vector<float>> touchdown_torque = {
  {0.05, 0.04},
  {0.05, 0.04},
  {0.05, 0.04},
  {0.05, 0.04}
};

// gait variables
std::vector<float> cmd_vector = {0, 0, 0};              // command vector: {forward-back, right-left, yaw angle}
uint8_t gait_phase = GaitPhases::kLateralSwing;         // current gait phase/swing body; (0 medial swing/lateral stance) -> (1 medial stance/lateral swing); double support is omitted
uint8_t actuation_phase = ActuationPhases::kRetractLeg; // current actuation phase of the swing legs; 0 retract -> 1 translate -> 2 touchdown
uint32_t gait_cycles = 0;                               // number of completed gait cycles
std::vector<int> inContact = {0,0,0,0};                 // true if the motor's current exceeds the threshold during touchdown; stays true until legs lift
bool isBlocking = false;                                // true if any motion primitive outside of the standard gait cycle is in progress
bool corrected = false;                                 // true if a motion primitive has been completed during the gait cycle

// nominal leg trajectory parameters; can be updated by a high-level planner
// exact trajectory is determined by the motor controller's trapezoidal trajectory generation: acceleration, deceleration, max velocity
float q_leg_stance = 250;         // nominal (lower) leg stroke in stance (tested 250)
float q_leg_swing = 10;           // nominal (lower) leg stroke at max clearance in swing (tested 35 and 70)
float q_trans = kQTransMax - 15;  // nominal translation position in swing
float q_yaw = kQYawMax - 5;       // nominal yaw rotation per step

float dq_leg_trans = 30;                   // remaining swing leg retraction at which forward translation begins (tested 50)
float q_trans_transition = 50;              // translational joint position at which swing leg step down begins; zero position is at midpoint; regardless of gait_phase, negative position indicates before crossing the midpoint and positive position indicates after crossing the midpoint
float z_body_nominal = q_leg_stance;       // nominal body height over local terrain; currently taken as avg of stance leg motors joint position

// motor setpoints that parameterize leg motion
// 1. leg stroke at max clearance in swing; can change to speed up locomotion or clear taller obstacles
// 2. translation position in swing; can change each step based on higher level planning
// 3. leg stroke in stance; can change each step based on proprioceptive sensing (terrain slope change)
std::vector<std::vector<float>> gait_setpoints = {
  {q_leg_swing + kLegOffset, -q_trans, q_leg_stance + kLegOffset},
  {q_leg_swing, q_trans, q_leg_stance}
};

std::vector<float> q_leg_contacts = {0, 0, 0, 0};

void homeLeggedRobot() {
  snprintf(sent_data, sizeof(sent_data), "Homing all legs...\n\n");
  transmitMsg();

  // leg actuator homing
  for (uint8_t i = 0; i < kNumOfLegs/2; i++) {
    ODrive_CAN.SetLimits(i, kVelLegMaxContact, kCurrentLegMaxHoming);
    motors[i].setControlMode(ODriveTeensyCAN::ControlMode_t::kVelocityControl);
    motors[i].sendCommand(ODriveTeensyCAN::ControlMode_t::kVelocityControl, kQdotLegHoming);
  }

  bool moving[4] = {true, true, true, true};
  uint32_t t_current = millis();

  // wait a minimum time to allow motors to start moving
  while (moving[0] || moving[1] || moving[2] || moving[3]) {
    handleODriveCANMsg();
    updateStates();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }

    for (uint8_t i = 0; i < kNumOfLegs/2; i++ ) {
      // if the motor has slowed down, it's at the joint limit
      if (moving[i] && millis() - t_current > 250 && fabs(motors[i].states_.q_dot) < fabs(kQdotLegHomingStop)) {
        moving[i] = false;
        
        motors[i].sendCommand(ODriveTeensyCAN::ControlMode_t::kVelocityControl, 0);
        motors[i].setControlMode(ODriveTeensyCAN::ControlMode_t::kPositionControl);
        ODrive_CAN.SetLimits(i, kVelLegMax, kCurrentLegMax);
        
        motors[i].states_.homed = true;
        motors[i].states_.pos_home = motors[i].states_.pos_abs;
        motors[i].states_.pos_rel = motors[i].states_.pos_abs - motors[i].states_.pos_home;
        encoders[i*2].write(0);
        encoders[i*2 + 1].write(0);
        
        snprintf(sent_data, sizeof(sent_data), "Actuator %d homed.\n******************************************************\n", i + 1);
        transmitMsg();
      }
    }
  }
  snprintf(sent_data, sizeof(sent_data), "Finished homing the legs.\n\n");
  transmitMsg();

  // yaw actuator homing
  snprintf(sent_data, sizeof(sent_data), "Homing the yaw mechanism...\n");
  transmitMsg();
  motors[MotorID::kMotorYaw].setControlMode(ODriveTeensyCAN::ControlMode_t::kPositionControl);
  motors[MotorID::kMotorYaw].sendCommand(ODriveTeensyCAN::ControlMode_t::kPositionControl, kQYawHome);  // yaw mechanism's range of motion is < 1 rev so the absolute encoder can be used directly

  t_current = millis();
  while (millis() - t_current < 500 || fabs(motors[MotorID::kMotorYaw].states_.q_dot) > fabs(kQdotYawHomingStop)) {
    handleODriveCANMsg();
    updateStates();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
  }

  motors[MotorID::kMotorYaw].states_.homed = true;
  motors[MotorID::kMotorYaw].states_.pos_home = motors[MotorID::kMotorYaw].states_.pos_abs;
  motors[MotorID::kMotorYaw].states_.pos_rel = motors[MotorID::kMotorYaw].states_.pos_abs - motors[MotorID::kMotorYaw].states_.pos_home;

  snprintf(sent_data, sizeof(sent_data), "Finished homing the yaw mechanism.\n\n");
  transmitMsg();

  // translation actuator homing
  snprintf(sent_data, sizeof(sent_data), "Homing the translation mechanism...\n");
  transmitMsg();

  motors[MotorID::kMotorTranslate].setControlMode(ODriveTeensyCAN::ControlMode_t::kVelocityControl);
  ODrive_CAN.SetLimits(MotorID::kMotorTranslate, kVelTransMax, kCurrentTransMaxHoming);
  motors[MotorID::kMotorTranslate].sendCommand(ODriveTeensyCAN::ControlMode_t::kVelocityControl, kQdotTransHoming);

  bool moving2 = true;
  t_current = millis();

  while (moving2) {
    handleODriveCANMsg();
    updateStates();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
    
    if (millis() - t_current > 500 && fabs(motors[MotorID::kMotorTranslate].states_.q_dot) < fabs(kQdotTransHomingStop)) {
      moving2 = false;

      motors[MotorID::kMotorTranslate].sendCommand(ODriveTeensyCAN::ControlMode_t::kVelocityControl, 0);
      motors[MotorID::kMotorTranslate].setControlMode(ODriveTeensyCAN::ControlMode_t::kPositionControl);
      ODrive_CAN.SetLimits(MotorID::kMotorTranslate, kVelTransMax, kCurrentTransMax);
      motors[MotorID::kMotorTranslate].sendCommand(ODriveTeensyCAN::ControlMode_t::kPositionControl,
                                                   motors[MotorID::kMotorTranslate].states_.pos_abs * motors[MotorID::kMotorTranslate].params_.T + kQTransHomingOffset); // move to the zero position
    }
  }

  t_current = millis();
  while (millis() - t_current < 500 || fabs(motors[MotorID::kMotorTranslate].states_.q_dot) > fabs(kQdotTransHomingStop)) {
    handleODriveCANMsg();
    updateStates();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
  }

  motors[MotorID::kMotorTranslate].states_.homed = true;
  motors[MotorID::kMotorTranslate].states_.pos_home = motors[MotorID::kMotorTranslate].states_.pos_abs;
  motors[MotorID::kMotorTranslate].states_.pos_rel = motors[MotorID::kMotorTranslate].states_.pos_abs - motors[MotorID::kMotorTranslate].states_.pos_home;

  snprintf(sent_data, sizeof(sent_data), "Finished homing the translation mechanism.\n\n");
  transmitMsg();

  snprintf(sent_data, sizeof(sent_data), "Homing finished.\n---------------------------------------------\n\n");
  transmitMsg();
}

void standUp() {
  uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;
  motors[stance * 2].states_.q_d = gait_setpoints[stance][2];
  motors[stance * 2 + 1].states_.q_d = gait_setpoints[stance][2];
  ODrive_CAN.SetTrajVelLimit(stance * 2, kVelLegTrajStandup);
  ODrive_CAN.SetTrajVelLimit(stance * 2 + 1, kVelLegTrajStandup);
  inContact[stance * 2] = true;
  inContact[stance * 2 + 1] = true;

  // update states while standing up
  while (!motors[stance * 2].states_.holding && !motors[stance * 2 + 1].states_.holding) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    // transmitData();
  }
  q_leg_contacts[0] = motors[0].states_.q;
  q_leg_contacts[1] = motors[1].states_.q;
  SERIAL_USB.println("Starting");
}

void updateGait() {
  uint32_t t_current = millis();
  if (t_current - t_last_setpoint_update >= k_dtSetpointUpdate) {
    t_last_setpoint_update = t_current;

    // calculate stability metrics here
    regulateBodyPose(); // if this function executes motions, updateSetpoint() is bypassed during the process
    updateSetpoints();  // update setpoints for the motors in the swing phase
  }
}

// currently takes joystick commands for heading
void updateCmdVector() {
  cmd_vector[0] = two_axis_joystick.y_norm;
  cmd_vector[1] = 0.1 * two_axis_joystick.x_norm; // scaled down for finer heading control
  cmd_vector[2] = pow(-1, gait_phase) * atan2(-cmd_vector[1], cmd_vector[0]) * 180 / PI;     // check yaw direction

  if (cmd_vector[2] > 90) {
    cmd_vector[2] -= 180;
  } else if (cmd_vector[2] < -90) {
    cmd_vector[2] += 180;
  }
}

// % fix this so that it only sets the setpoint once per cycle
// % isBlocking doesn't prevent the if statements from being run repeatedly
void regulateBodyPose() {
  // nominal body height control
  if (!corrected && actuation_phase == 2 && fabs(z_body_local - z_body_nominal) > kdzMax && fabs(rpy_medial[0]) < kTiltNominal && fabs(rpy_medial[1]) < kTiltNominal) { // if swing legs have made contact and body height is outside allowed range
    updateContactState();
    if (inContact[gait_phase * 2] && inContact[gait_phase * 2 + 1]) {
      float dz_body_local = z_body_local - z_body_nominal;

      // adjust velocity, acceleration, deceleration limits for trapezoidal trajectory during this maneuver
      for (uint8_t i = 0; i < kNumOfLegs/2; i++) {
        ODrive_CAN.SetTrajVelLimit(i, kVelLegTrajTilt);
        ODrive_CAN.SetTrajAccelLimits(i, kAccelLegTrajSwing, kDecelLegTrajTilt);
      }

      // move all leg motors by dz_body_local
      for (uint8_t i = 0; i < kNumOfLegs/2; i++) {
        motors[i].states_.q_d = motors[i].states_.q - dz_body_local;
      }
      isBlocking = true;
      corrected = true;
    }

    // nominal body tilt control
  } else if (!corrected && fabs(rpy_medial[gait_phase]) > kTiltNominal && actuation_phase == 1 && fabs(motors[MotorID::kMotorTranslate].states_.q) < 1 && fabs(motors[MotorID::kMotorTranslate].states_.q_dot) < 0.5) {  // if in translation and body is tilted
    uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;
    float dq = stanceWidth[gait_phase] * tan(rpy_medial[gait_phase] * PI / 180);
    if (dq > kDqLegMaxTilt) {
      dq = kDqLegMaxTilt;
    }
    if (dq < -kDqLegMaxTilt) {
      dq = -kDqLegMaxTilt;
    }
    
    motors[stance * 2].states_.q_d = motors[stance * 2].states_.q_d - dq / 2;
    motors[stance * 2 + 1].states_.q_d = motors[stance * 2 + 1].states_.q_d + dq / 2;
    ODrive_CAN.SetTrajVelLimit(stance * 2, kVelLegTrajTilt);
    ODrive_CAN.SetTrajVelLimit(stance * 2 + 1, kVelLegTrajTilt);
    ODrive_CAN.SetTrajAccelLimits(stance * 2, kAccelLegTrajSwing, kDecelLegTrajTilt);
    ODrive_CAN.SetTrajAccelLimits(stance * 2 + 1, kAccelLegTrajSwing, kDecelLegTrajTilt);
    isBlocking = true;
    corrected = true;

  } else if (isBlocking && fabs(bno08x_imu.gyro[0]) < kGyroStable && fabs(bno08x_imu.gyro[1]) < kGyroStable &&
             fabs(motors[0].states_.q_dot) < kQdotContact && fabs(motors[1].states_.q_dot) < kQdotContact && fabs(motors[2].states_.q_dot) < kQdotContact && fabs(motors[3].states_.q_dot) < kQdotContact) {
    isBlocking = false;
    for(uint8_t i = 0; i < 4; i++) {  // update the leg stroke at contact
      q_leg_contacts[i] = motors[i].states_.q;
    }
  }
}

// returns true if ready for transition to the next actuation phase
bool isReadyForTransition(uint8_t phase) {
  if (phase == ActuationPhases::kRetractLeg) {                                        // if currently retracting leg
    float q_leg_transition_1 = motors[gait_phase*2].states_.q_d + dq_leg_trans;       // leg stroke above which translation can begin
    float q_leg_transition_2 = motors[gait_phase*2 + 1].states_.q_d + dq_leg_trans;
    return motors[gait_phase*2].states_.q < q_leg_transition_1 && motors[gait_phase*2 + 1].states_.q < q_leg_transition_2;

  } else if (phase == ActuationPhases::kTranslateForward) {  // if currently translating forward
    return fabs(cmd_vector[0]) > 0.01 && pow(-1, gait_phase + 1) * q[JointID::kJointTranslate] > q_trans_transition; // check if there is a forward command and if the translational joint has reached the transition point

  } else if (phase == ActuationPhases::kTouchDown) {                    // if currently touching down
    updateContactState();                                               // check if the swing legs make ground contact
    return inContact[gait_phase * 2] && inContact[gait_phase * 2 + 1];  // true if both swing legs have made contact

  } else {
    return false;
  }
}

// currently implemented for position control locomotion
void updateSetpoints() {
  // update setpoints at gait phase transitions here
  if (!isBlocking && isReadyForTransition(actuation_phase)) {

    if (actuation_phase == ActuationPhases::kRetractLeg) {              // if currently retracting leg
      if (fabs(cmd_vector[0]) < 0.01 || fabs(rpy_medial[gait_phase]) > kTiltNominal) {                                    // if no forward command OR if body is tilted, stop at the neutral body translation
        motors[MotorID::kMotorTranslate].states_.q_d = 0;                                                                 // move to the neutral point and stop
        motors[MotorID::kMotorYaw].states_.q_d = 0;
      } else {                                                                                                            // else, actuate the locomotion mechanism according to cmd_vector
        motors[MotorID::kMotorTranslate].states_.q_d =
            gait_setpoints[gait_phase][ActuationPhases::kTranslateForward] * cmd_vector[0];                               // forward translation scaled by x-component of cmd_vector
        motors[MotorID::kMotorYaw].states_.q_d =
            min(motors[MotorID::kMotorYaw].states_.q_max, max(motors[MotorID::kMotorYaw].states_.q_min, cmd_vector[2]));  // limit the max turn per step
      }

    } else if (actuation_phase == ActuationPhases::kTranslateForward) { // if currently translating forward
      // change to torque control for leg touchdown
      motors[gait_phase * 2].setControlMode(ODriveTeensyCAN::ControlMode_t::kTorqueControl);
      motors[gait_phase * 2 + 1].setControlMode(ODriveTeensyCAN::ControlMode_t::kTorqueControl);

      // limit velocity in torque control during touchdown
      ODrive_CAN.SetLimits(gait_phase*2, kVelLegMaxContact, kCurrentLegMax);
      ODrive_CAN.SetLimits(gait_phase*2 + 1, kVelLegMaxContact, kCurrentLegMax);

      // set torque command for leg touchdown
      motors[gait_phase * 2].states_.tau_d = touchdown_torque[gait_phase * 2][0];
      motors[gait_phase * 2 + 1].states_.tau_d = touchdown_torque[gait_phase * 2 + 1][0];

      // reapply locomotion mechanism setpoints in the case of resuming locomotion from standstill
      motors[MotorID::kMotorTranslate].states_.q_d =
            gait_setpoints[gait_phase][ActuationPhases::kTranslateForward] * cmd_vector[0];                               // forward translation scaled by x-component of cmd_vector
      motors[MotorID::kMotorYaw].states_.q_d =
          min(motors[MotorID::kMotorYaw].states_.q_max, max(motors[MotorID::kMotorYaw].states_.q_min, cmd_vector[2]));    // limit the max turn per step
      
    } else if (actuation_phase == ActuationPhases::kTouchDown) {        // if currently touching down
      isBlocking = false;
      corrected = false;
      
      // remember the leg strokes at ground contact
      q_leg_contacts[gait_phase*2] = motors[gait_phase*2].states_.q;
      q_leg_contacts[gait_phase*2 + 1] = motors[gait_phase*2 + 1].states_.q;

      // zero out torque command
      motors[gait_phase*2].sendCommand(ODriveTeensyCAN::ControlMode_t::kTorqueControl, 0);
      motors[gait_phase*2 + 1].sendCommand(ODriveTeensyCAN::ControlMode_t::kTorqueControl, 0);

      // put the legs into position control now
      motors[gait_phase * 2].setControlMode(ODriveTeensyCAN::ControlMode_t::kPositionControl);
      motors[gait_phase * 2 + 1].setControlMode(ODriveTeensyCAN::ControlMode_t::kPositionControl);

      // set the desired position to the current position
      motors[gait_phase * 2].states_.q_d = motors[gait_phase * 2].states_.q;
      motors[gait_phase * 2 + 1].states_.q_d = motors[gait_phase * 2 + 1].states_.q;

      // advance the gait phase
      gait_phase = (gait_phase + 1) % kNumOfGaitPhases;
      if (gait_phase == 0) {
        gait_cycles++; // if the gait phase is back to 0, increment the number of completed gait cycles
      }

      // update and/or reset swing/stance setpoints based on last contact conditions
      for (uint8_t i = 0; i < 2; i++) {
        if (fabs(q[gait_phase * 4 + i * 2] - q[gait_phase * 4 + i * 2 + 1]) > kDqUnevenTerrain) { // if the previous stance legs are standing on uneven ground
          float q_max = max(q[gait_phase * 4 + i * 2], q[gait_phase * 4 + i * 2 + 1]);            // calculate the additional leg stroke due to uneven terrain
          float dq = q_max - motors[gait_phase * 2 + i].states_.q_d;
          motors[gait_phase * 2 + i].states_.q_d = max(kQLegMin, gait_setpoints[gait_phase][ActuationPhases::kRetractLeg] - dq); // retract more by dq to ensure proper ground clearance

        } else {  
          motors[gait_phase * 2 + i].states_.q_d = gait_setpoints[gait_phase][ActuationPhases::kRetractLeg];  // if even terrain, use the nominal swing leg setpoint
        }
        
        // update the motor limits for leg retraction
        ODrive_CAN.SetLimits(gait_phase*2 + i, kVelLegMax, kCurrentLegMax);
        ODrive_CAN.SetTrajVelLimit(gait_phase*2 + i, kVelLegTrajSwing);
        ODrive_CAN.SetTrajAccelLimits(gait_phase*2 + i, kAccelLegTrajSwing, kDecelLegTrajSwing);
        inContact[gait_phase * 2 + i] = false;
      }
    }
    actuation_phase = (actuation_phase + 1) % kNumOfActuationPhases;

    // update setpoints that do not involve a gait phase transition here
  } else if (!isBlocking) {
    if (actuation_phase == ActuationPhases::kRetractLeg) {              // if currently retracting leg
      
    } else if (actuation_phase == ActuationPhases::kTranslateForward) { // if currently translating forward
      
      // if the forward command stops, stop at the current translation position
      if (fabs(cmd_vector[0]) < 0.01) {
        motors[MotorID::kMotorTranslate].states_.q_d = motors[MotorID::kMotorTranslate].states_.q;
        motors[MotorID::kMotorYaw].states_.q_d = motors[MotorID::kMotorYaw].states_.q;

      // else if there's body tilt and hasn't been corrected yet, stop at the neutral translation position
      } else if (!corrected && fabs(rpy_medial[gait_phase]) > kTiltNominal) {
        motors[MotorID::kMotorTranslate].states_.q_d = 0;
        motors[MotorID::kMotorYaw].states_.q_d = 0;

      // else, continue moving according to the current command
      } else {
        motors[MotorID::kMotorTranslate].states_.q_d = gait_setpoints[gait_phase][ActuationPhases::kTranslateForward] * cmd_vector[0];                        // forward translation scaled by x-component of cmd_vector
        motors[MotorID::kMotorYaw].states_.q_d = min(motors[MotorID::kMotorYaw].states_.q_max, max(motors[MotorID::kMotorYaw].states_.q_min, cmd_vector[2])); // limit the max turn per step  
      }
      
    } else if (actuation_phase == ActuationPhases::kTouchDown) {        // if currently touching down
      // for (uint8_t i = 0; i < 2; i++) {     // for each swing leg motor
      //   if (!inContact[gait_phase * 2 + i] && // if the motor is not in contact
      //       motors[gait_phase * 2 + i].states_.q - gait_setpoints[gait_phase][ActuationPhases::kRetractLeg] > kDqStartContact) { // AND the leg stroke is past the initial displacement
              
      //     motors[gait_phase * 2 + i].states_.tau_d = touchdown_torque[gait_phase * 2 + i][1];        // apply the lower torque command
      //   }
      // }
    }
  }
}

// estimates the contact state of each swing leg motors
void updateContactState() {
  for (uint8_t i = 0; i < 2; i++) {     // for each swing leg motor
    inContact[gait_phase * 2 + i] = motors[gait_phase * 2 + i].states_.q - gait_setpoints[gait_phase][0] > kDqStartContact  // if the leg stroke is past the initial displacement
                                    && fabs(motors[gait_phase * 2 + i].states_.q_dot) < kQdotContact;                       // AND the motor "joint velocity" is below a threshold
  }
}



