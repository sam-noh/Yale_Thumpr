#include "locomotion.h"
#include "../../include/joint_defs.h"
#include "../mcu_util/mcu_util.h"
#include "../motor_control/motor_control.h"
#include "../state_estimation/state_estimation.h"

std::vector<std::vector<float>> touchdown_torque = {
  {0.18, 0.14}, // 0.18 and 0.12
  {0.18, 0.14},
  {0.18, 0.14},
  {0.18, 0.14}
};

// gait variables
std::vector<float> cmd_vector = {0, 0};                 // command vector: {forward-back, yaw angle}
uint8_t gait_phase = GaitPhases::kLateralSwing;         // current gait phase/swing body; (0 medial swing/lateral stance) -> (1 medial stance/lateral swing); double support is omitted
uint8_t actuation_phase = ActuationPhases::kRetractLeg; // current actuation phase of the swing legs; 0 retract -> 1 translate -> 2 touchdown
uint32_t gait_cycles = 0;                               // number of completed gait cycles
bool isBlocking = false;                                // true if any motion primitive outside of the standard gait cycle is in progress
bool isCorrected = false;                               // true if a motion primitive has been completed during the gait cycle

// nominal leg trajectory parameters; can be updated by a high-level planner
// exact trajectory is determined by the motor controller's trapezoidal trajectory generation: acceleration, deceleration, max velocity
float z_body_nominal = 180;               // nominal body height over local terrain in mm; currently taken as avg of stance leg motors joint position
float leg_swing_percent = 0.9;            // swing leg stroke as a percentage of its stroke at last stance phase
float dz_body_local = 0;                  // amount of body height change applied; used as an offset for the leg stroke in contact estimation

// actuation phase transition parameters
// these are currently fixed and not exposed for easier teleop
float swing_percent_at_translate = 0.5;   // percentage of swing leg retraction after which translation begins; small values can cause swing legs to collide with rough terrains
float trans_percent_at_touchdown = 0.4;   // percentage of translatonal displacement from midpoint after which leg touchdown begins; small values can result in leg touchdown before the translation completes, resulting in some backward motion after stance switch
float yaw_percent_at_touchdown = 0.6;     // percentage of yaw command from midpoint after which leg touchdown begins; small values can result in leg touchdown before the turning completes, resulting in some backward motion after stance switch

std::vector<float> q_leg_contact = {kQLegMax, kQLegMax};    // position of the swing leg actuators when they were last in contact
std::vector<float> q_leg_swing = {kQLegMin, kQLegMin};      // position setpoint of swing leg actuators during leg retraction

void homeLeggedRobot() {
  snprintf(sent_data, sizeof(sent_data), "Homing all legs...\n\n");
  writeToSerial();

  // leg actuator homing
  for (uint8_t axis_id = 0; axis_id < kNumOfLegs/2; ++axis_id) {
    motors[axis_id].states_.current_limit = kCurrentLegMaxHoming;
    motors[axis_id].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kVelocityControl;
    motors[axis_id].states_.q_dot_d = kQdotLegHoming;
  }

  bool moving[4] = {true, true, true, true};
  uint32_t t_current = millis();

  // wait a minimum time to allow motors to start moving
  while (moving[0] || moving[1] || moving[2] || moving[3]) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();

    for (uint8_t axis_id = 0; axis_id < kNumOfLegs/2; ++axis_id) {
      // if the motor has slowed down, it's at the joint limit
      if (moving[axis_id] && millis() - t_current > 1000 && fabs(motors[axis_id].states_.q_dot) < fabs(kQdotLegHomingStop)) {
        moving[axis_id] = false;
        
        motors[axis_id].states_.q_dot_d = 0;
        motors[axis_id].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
        motors[axis_id].states_.trap_traj_vel_limit = kVelLegTrajStandup;
        motors[axis_id].states_.current_limit = kCurrentLegMax;
                
        motors[axis_id].states_.homed = true;
        motors[axis_id].states_.pos_home = motors[axis_id].states_.pos_abs;
        motors[axis_id].states_.pos_rel = motors[axis_id].states_.pos_abs - motors[axis_id].states_.pos_home;
        encoders[axis_id*2].write(0);
        encoders[axis_id*2 + 1].write(0);
        
        snprintf(sent_data, sizeof(sent_data), "Actuator %d homed.\n******************************************************\n", axis_id + 1);
        writeToSerial();
      }
    }
    
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
  }

  snprintf(sent_data, sizeof(sent_data), "Finished homing the legs.\n\n");
  writeToSerial();

  // lift the body slightly to allow locomotion mechanism homing
  // depends on which body starts in stance
  snprintf(sent_data, sizeof(sent_data), "Lifting body off the ground...\n\n");
  writeToSerial();

  uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;
  motors[stance*2].states_.q_d = 40;
  motors[stance*2 + 1].states_.q_d = 40;

  t_current = millis();
  while (millis() - t_current < 500 || (fabs(motors[stance*2].states_.q_dot) > fabs(kQdotLegHomingStop)
                                        && fabs(motors[stance*2 + 1].states_.q_dot) > fabs(kQdotLegHomingStop))) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
  }

  // yaw actuator homing
  snprintf(sent_data, sizeof(sent_data), "Homing the yaw mechanism...\n");
  writeToSerial();

  motors[MotorID::kMotorYaw].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
  motors[MotorID::kMotorYaw].states_.q_d = 0;   // yaw mechanism's range of motion is < 1 rev so the absolute encoder can be used directly
                                                // if the ODrive firmware changes, check the API to ensure this is the correct command for absolute position control

  t_current = millis();
  while (millis() - t_current < 500 || fabs(motors[MotorID::kMotorYaw].states_.q_dot) > fabs(kQdotYawHomingStop)) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
  }

  motors[MotorID::kMotorYaw].states_.homed = true;
  motors[MotorID::kMotorYaw].states_.pos_home = motors[MotorID::kMotorYaw].states_.pos_abs;
  motors[MotorID::kMotorYaw].states_.pos_rel = motors[MotorID::kMotorYaw].states_.pos_abs - motors[MotorID::kMotorYaw].states_.pos_home;

  snprintf(sent_data, sizeof(sent_data), "Finished homing the yaw mechanism.\n\n");
  writeToSerial();

  // translation actuator homing
  snprintf(sent_data, sizeof(sent_data), "Homing the translation mechanism...\n");
  writeToSerial();

  motors[MotorID::kMotorTranslate].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kVelocityControl;
  motors[MotorID::kMotorTranslate].states_.current_limit = kCurrentTransMaxHoming;
  motors[MotorID::kMotorTranslate].states_.q_dot_d = kQdotTransHoming;

  bool moving2 = true;
  t_current = millis();

  while (moving2) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
    
    if (millis() - t_current > 500 && fabs(motors[MotorID::kMotorTranslate].states_.q_dot) < fabs(kQdotTransHomingStop)) {
      moving2 = false;

      snprintf(sent_data, sizeof(sent_data), "Reached translation joint limit.\n");
      writeToSerial();

      motors[MotorID::kMotorTranslate].states_.q_dot_d = 0;
      motors[MotorID::kMotorTranslate].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
      motors[MotorID::kMotorTranslate].states_.current_limit = kCurrentTransMax;
      motors[MotorID::kMotorTranslate].states_.q_d = motors[MotorID::kMotorTranslate].params_.direction*
                                                     motors[MotorID::kMotorTranslate].states_.pos_abs*
                                                     motors[MotorID::kMotorTranslate].params_.T + kQTransHomingOffset; // move to the zero position
    }
  }

  t_current = millis();
  while (millis() - t_current < 500 || fabs(motors[MotorID::kMotorTranslate].states_.q_dot) > fabs(kQdotTransHomingStop)) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    if (stop_signal) {
      stopActuators();
      while (1) {}
    }
  }

  motors[MotorID::kMotorTranslate].states_.homed = true;
  motors[MotorID::kMotorTranslate].states_.pos_home = motors[MotorID::kMotorTranslate].states_.pos_abs;
  motors[MotorID::kMotorTranslate].states_.pos_rel = motors[MotorID::kMotorTranslate].states_.pos_abs - motors[MotorID::kMotorTranslate].states_.pos_home;
  motors[MotorID::kMotorTranslate].states_.q_d = 0;

  snprintf(sent_data, sizeof(sent_data), "Finished homing the translation mechanism.\n\n");
  writeToSerial();

  snprintf(sent_data, sizeof(sent_data), "Homing finished.\n---------------------------------------------\n\n");
  writeToSerial();
}

void standUp() {
  snprintf(sent_data, sizeof(sent_data), "Standing up...\n");
  writeToSerial();
  
  uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;

  for (uint8_t axis_id = stance*2; axis_id < stance*2 + 2; ++axis_id) {
    motors[axis_id].states_.q_d = z_body_nominal;
    motors[axis_id].states_.holding = false;
    motors[axis_id].states_.trap_traj_vel_limit = kVelLegTrajStandup;
    isInContact[axis_id] = true;
  }

  // update states while standing up
  while (!motors[stance * 2].states_.holding && !motors[stance * 2 + 1].states_.holding) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
  }

  for (uint8_t axis_id = stance*2; axis_id < stance*2 + 2; ++axis_id) {
    motors[axis_id].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kTorqueControl;
    motors[axis_id].states_.tau_d = 0;                          // zero out torque command
    motors[axis_id].sendCommand(ODriveTeensyCAN::ControlMode_t::kTorqueControl, motors[axis_id].states_.tau_d);
  }
  
  snprintf(sent_data, sizeof(sent_data), "Starting\n");
  writeToSerial();
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

// update desired motion vector, body height, trajectories, etc. based on higher-level input/planner
void updateTrajectory() {
  // apply deadzone to left-right joystick input to prevent undesired turning during translation
  float input_y_filtered = max(abs(input_y) - kGUIJoystickYDeadZone, 0);
  int dir_y = (input_y > 0) - (input_y < 0);
  input_y_filtered *= dir_y/kGUIJoystickYDeadZone;

  // apply deadzone to front-back joystick input to prevent undesired translation during in-place turning
  float input_x_filtered = max(abs(input_x) - kGUIJoystickXDeadZone, 0);
  int dir_x = (input_x > 0) - (input_x < 0);
  input_x_filtered *= dir_x/kGUIJoystickXDeadZone;

  cmd_vector[0] = input_x_filtered;                                     // use deadzone/scaled input
  cmd_vector[1] = input_y_filtered;                                     // use deadzone/scaled input

  leg_swing_percent = max(min(input_swing, kLegSwingPercentMax), kLegSwingPercentMin);  // bound the leg swing percentage with min/max
  z_body_nominal = (k_zBodyMax - k_zBodyMin)*input_height + k_zBodyMin;                 // for now, nominal body height is equal to the leg actuator setpoint in stance
  
}

// executes body orientation and height regulation that blocks normal gait cycle
void regulateBodyPose() {

  if (!isBlocking) {    // only check body pose if currently not performing a regulation maneuver

    // nominal body height control
    if (actuation_phase == ActuationPhases::kTouchDown                                    // if currently touching down
        && isInContact[gait_phase * 2] && isInContact[gait_phase * 2 + 1]                 // AND the swing legs are now also on the ground
        && fabs(z_body_local - z_body_nominal) > kdzMax                                   // AND the body height is not within nominal range
        && fabs(rpy_lateral[0]) < kTiltNominal && fabs(rpy_lateral[1]) < kTiltNominal) {  // AND the body tilt is within nominal range

      // perform the body height regulation maneuver
      dz_body_local = z_body_local - z_body_nominal;

      for (uint8_t axis_id = 0; axis_id < kNumOfLegs/2; ++axis_id) {
        // put leg motors in position control for the maneuver
        motors[axis_id].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;

        // adjust velocity, acceleration, deceleration limits for trapezoidal trajectory during this maneuver
        motors[axis_id].states_.trap_traj_vel_limit = kVelLegTrajStandup;
        motors[axis_id].states_.trap_traj_accel_limit = kAccelLegTrajStandup;
        motors[axis_id].states_.trap_traj_decel_limit = kDecelLegTrajStandup;
        
        // move all leg motors by dz_body_local
        motors[axis_id].states_.q_d = motors[axis_id].states_.q - dz_body_local;
      }

      isBlocking = true;
      isCorrected = true;

    // nominal body tilt control
    } else if (actuation_phase == ActuationPhases::kLocomote                      // if currently translating or turning
               && fabs(rpy_lateral[gait_phase]) > kTiltNominal                    // AND the body tilt is not within nominal range
               && fabs(motors[MotorID::kMotorTranslate].states_.q) < 20) {        // AND the translational joint is near the midpoint

      uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;
      float dq = stance_width[gait_phase] * tan(rpy_lateral[gait_phase] * PI / 180);
      if (dq > kDqLegMaxTilt) {
        dq = kDqLegMaxTilt;
      }
      if (dq < -kDqLegMaxTilt) {
        dq = -kDqLegMaxTilt;
      }

      for (uint8_t i = 0; i < 2; ++i) {
        // put leg motors in position control for the maneuver
        motors[stance * 2 + i].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;

        // adjust velocity, acceleration, deceleration limits for trapezoidal trajectory during this maneuver
        motors[stance * 2 + i].states_.trap_traj_vel_limit = kVelLegTrajTilt;
        motors[stance * 2 + i].states_.trap_traj_accel_limit = kAccelLegTrajTilt;
        motors[stance * 2 + i].states_.trap_traj_decel_limit = kDecelLegTrajTilt;

        // move opposite leg motors
        motors[stance * 2 + i].states_.q_d = motors[stance * 2 + i].states_.q + pow(-1, i)*dq / 2;
      }
      
      isBlocking = true;
      isCorrected = true;
     }


  } else if (isBlocking                                                                                         // if the body pose regulation is currently happening
             && fabs(omega_lateral[0]) < kOmegaStable && fabs(omega_lateral[1]) < kOmegaStable                  // AND the body angular velocities are below a threshold
             && fabs(motors[0].states_.q_dot) < kQdotStable && fabs(motors[1].states_.q_dot) < kQdotStable      // AND all leg motor velocities are below a threshold
             && fabs(motors[2].states_.q_dot) < kQdotStable && fabs(motors[3].states_.q_dot) < kQdotStable) {

    isBlocking = false;
    
    for(uint8_t axis_id = 0; axis_id < kNumOfLegs/2; ++axis_id) {
      motors[axis_id].states_.trap_traj_vel_limit = kVelLegTrajSwing;
      motors[axis_id].states_.trap_traj_accel_limit = kAccelLegTrajSwing;
      motors[axis_id].states_.trap_traj_decel_limit = kDecelLegTrajSwing;
    }

  }
}

// returns true if ready for transition to the next actuation phase
bool isReadyForTransition(uint8_t phase) {
  
  if (phase == ActuationPhases::kRetractLeg) {  // if currently retracting leg
    float q_leg_transition_1 = q_leg_swing[0] + swing_percent_at_translate*(q_leg_contact[0] - q_leg_swing[0]);
    float q_leg_transition_2 = q_leg_swing[1] + swing_percent_at_translate*(q_leg_contact[1] - q_leg_swing[1]);

    return motors[gait_phase*2].states_.q < q_leg_transition_1 && motors[gait_phase*2 + 1].states_.q < q_leg_transition_2;

  } else if (phase == ActuationPhases::kLocomote) { // if currently translating or turning
    bool isTranslated = false, isTurned = false;

    // if there is a translation command
    if (fabs(cmd_vector[0]) > EPS) {
      int dir = (cmd_vector[0] > 0) - (cmd_vector[0] < 0);                                              // direction of translation command
      float q_trans_transition = fabs(trans_percent_at_touchdown*kQTransMax*cmd_vector[0]);             // this value is always positive since it represents forward motion, regardless of direction
      isTranslated = dir * pow(-1, gait_phase + 1) * q[JointID::kJointTranslate] > q_trans_transition;  // the translational joint has reached the transition point
                                                                                                        // assume that any concurrent yaw motion will be completed in time

    // if there is an in-place turn command
    } else if (fabs(cmd_vector[1]) > EPS) {
      int dir = (cmd_vector[1] > 0) - (cmd_vector[1] < 0);                              // direction of yaw command
      float q_yaw_transition = fabs(yaw_percent_at_touchdown*kQYawMax*cmd_vector[1]);   // this value is always positive since it represents forward motion, regardless of direction
      isTurned = dir * pow(-1, gait_phase) * q[JointID::kJointYaw] > q_yaw_transition;  // the translational joint has reached the transition point
    }

    return isTranslated || isTurned;

  } else if (phase == ActuationPhases::kTouchDown) {  // if currently touching down
        
    return isInContact[gait_phase * 2] && isInContact[gait_phase * 2 + 1]
           && ((fabs(z_body_local - z_body_nominal) < kdzMax)
           ||
           (fabs(z_body_local - z_body_nominal) > kdzMax && !(fabs(rpy_lateral[0]) < kTiltNominal && fabs(rpy_lateral[1]) < kTiltNominal)));  // true if both swing legs have made contact and body height regulation is not needed

  } else {
    return false;
  }
}

// touchdown: torque control
// translate: position control
// leg retract: position control
void updateSetpoints() {
  // update setpoints at gait phase transitions here
  if (!isBlocking && isReadyForTransition(actuation_phase)) {

    if (actuation_phase == ActuationPhases::kRetractLeg) {  // if currently retracting leg

      if (fabs(cmd_vector[0]) < EPS && fabs(cmd_vector[1]) < EPS) {   // if no translation or yaw command
        holdLocomotionMechanism();

      } else {  // else, move according to the current command
        moveLocomotionMechanism();
      }

    } else if (actuation_phase == ActuationPhases::kLocomote) { // if currently translating or turning
      updateLegMotorsForTouchdown();
      moveLocomotionMechanism();      // reapply locomotion mechanism setpoints in the case of resuming locomotion from standstill
      
    } else if (actuation_phase == ActuationPhases::kTouchDown) {        // if currently touching down
      // clear these flags/variables for the next gait cycle
      isBlocking = false;
      isCorrected = false;
      dz_body_local = 0;

      updateLegMotorsForStance();

      // advance the gait phase
      gait_phase = (gait_phase + 1) % kNumOfGaitPhases;
      if (gait_phase == 0) {
        gait_cycles++; // if the gait phase is back to 0, increment the number of completed gait cycles
      }

      updateSwingLegSetpoints();
      updateLegMotorsForSwing();
      resetSwingLegContactState();
    }

    actuation_phase = (actuation_phase + 1) % kNumOfActuationPhases;

    // update setpoints that do not involve a gait phase transition here
  } else if (!isBlocking) {
    if (actuation_phase == ActuationPhases::kRetractLeg) {          // if currently retracting leg
      
    } else if (actuation_phase == ActuationPhases::kLocomote) {     // if currently translating or turning
      
      if (fabs(cmd_vector[0]) < EPS && fabs(cmd_vector[1]) < EPS) {   // if no translation or yaw command
        holdLocomotionMechanism();

      } else {  // else, move according to the current command
        moveLocomotionMechanism();
      }
      
    } else if (actuation_phase == ActuationPhases::kTouchDown) {    // if currently touching down
      updateSwingLegTorque();
      updateLegMotorsForStance();
    }
  }
}

void updateLegMotorsForTouchdown() {
  for (uint8_t axis_id = gait_phase*2; axis_id < gait_phase*2 + 2; ++axis_id) {
    motors[axis_id].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kTorqueControl; // change to torque control for leg touchdown
    motors[axis_id].states_.velocity_limit = kVelLegMaxContact;                         // limit velocity in torque control during touchdown
    motors[axis_id].states_.tau_d = touchdown_torque[axis_id][0];                       // set torque command for leg touchdown
  }
}

void updateSwingLegTorque() {
  for (uint8_t i = 0; i < 2; ++i) {
    if (!isInContact[gait_phase*2 + i]                                                          // if the motor is not in contact
        && (motors[gait_phase*2 + i].states_.q + dz_body_local - q_leg_swing[i]) > kDqLegRamp) {  // AND the leg stroke is past the initial displacement

          motors[gait_phase*2 + i].states_.tau_d = touchdown_torque[gait_phase*2 + i][1];        // apply the lower torque command
        }
  }
}

// update and/or reset swing/stance setpoints based on last contact conditions
void updateSwingLegSetpoints() {
  for (uint8_t i = 0; i < 2; i++) {

    q_leg_contact[i] = motors[gait_phase*2 + i].states_.q;      // remember the leg motor position at ground contact
    float q_leg_retract = q_leg_contact[i]*leg_swing_percent;   // nominal swing leg setpoint; NOT necessarily equal to the actual setpoint

    // TODO: change this logic to account for robot kinematics (body tilt)
    if (fabs(q[gait_phase * 4 + i * 2] - q[gait_phase * 4 + i * 2 + 1]) > kDqUnevenTerrain) { // if the previous stance legs are standing on uneven ground
      float q_max = max(q[gait_phase * 4 + i * 2], q[gait_phase * 4 + i * 2 + 1]);            // calculate the additional leg stroke due to uneven terrain
      float dq = q_max - motors[gait_phase * 2 + i].states_.q;
      
      motors[gait_phase * 2 + i].states_.q_d = max(kQLegMin + 5, q_leg_retract - dq); // retract more by dq to ensure proper ground clearance

    } else {  
      motors[gait_phase * 2 + i].states_.q_d = q_leg_retract;  // if even terrain, use the nominal swing leg setpoint
    }

    // remember the swing leg position setpoint
    q_leg_swing[i] = motors[gait_phase * 2 + i].states_.q_d;
  }
}

// update the motor control mode and limits for swing phase
void updateLegMotorsForSwing() {
  for (uint8_t axis_id = gait_phase*2; axis_id < gait_phase*2 + 2; ++axis_id) {
    motors[axis_id].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
    motors[axis_id].states_.holding = false;
    motors[axis_id].states_.velocity_limit = kVelLegMax;
    motors[axis_id].states_.trap_traj_vel_limit = kVelLegTrajSwing;
    motors[axis_id].states_.trap_traj_accel_limit = kAccelLegTrajSwing;
    motors[axis_id].states_.trap_traj_decel_limit = kDecelLegTrajSwing;
  }
}

void updateLegMotorsForStance() {
  for (uint8_t axis_id = gait_phase*2; axis_id < gait_phase*2 + 2; ++axis_id) {
    if(isInContact[axis_id]) {
      motors[axis_id].states_.q_d = motors[axis_id].states_.q;    // set desired position for telemetry purposes and possible control changes
      motors[axis_id].states_.tau_d = 0;                          // zero out torque command
      motors[axis_id].sendCommand(ODriveTeensyCAN::ControlMode_t::kTorqueControl, motors[axis_id].states_.tau_d);
    }
  }
}

void moveLocomotionMechanism() {
  float q_trans = pow(-1, gait_phase + 1) * kQTransMax*cmd_vector[0];
  float q_yaw = pow(-1, gait_phase)*kQYawMax*cmd_vector[1];
  motors[MotorID::kMotorTranslate].states_.q_d = q_trans;
  motors[MotorID::kMotorYaw].states_.q_d = q_yaw;
}

void holdLocomotionMechanism() {
  motors[MotorID::kMotorTranslate].states_.q_d = motors[MotorID::kMotorTranslate].states_.q;  // stop at current translation and yaw
  motors[MotorID::kMotorYaw].states_.q_d = motors[MotorID::kMotorYaw].states_.q;
}