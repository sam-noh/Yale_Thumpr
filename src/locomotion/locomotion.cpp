#include "locomotion.h"
#include "../../include/joint_defs.h"
#include "../mcu_util/mcu_util.h"
#include "../motor_control/motor_control.h"
#include "../state_estimation/state_estimation.h"

std::vector<std::vector<float>> torque_profile_touchdown = {
  {0.55, 0.25, 0.12}, // 0.18 and 0.12
  {0.55, 0.25, 0.12},
  {0.55, 0.25, 0.10},
  {0.55, 0.25, 0.12}
};

// gait variables
std::vector<float> cmd_vector = {1, 0};                 // command vector: {forward-back, yaw angle}
uint8_t gait_phase = GaitPhases::kLateralSwing;         // current gait phase/swing body; (0 medial swing/lateral stance) -> (1 medial stance/lateral swing); double support is omitted
uint8_t actuation_phase = ActuationPhases::kRetractLeg; // current actuation phase of the swing legs; 0 retract -> 1 translate -> 2 touchdown
uint32_t gait_cycles = 0;                               // number of completed gait cycles
uint8_t motion_primitive = ReactiveBehaviors::kNone;    // current motion primitive
bool isBlocking = false;                                // true if a blocking motion primitive is in progress
bool isScheduled = false;                               // true if a motion primitive is scheduled for execution

// nominal leg trajectory parameters; can be updated by a high-level planner
// exact trajectory is determined by the motor controller's trapezoidal trajectory generation: acceleration, deceleration, max velocity
float z_body_nominal = 180;               // nominal body height over local terrain in mm; currently taken as avg of stance leg motors joint position
float leg_swing_percent = 0.9;            // swing leg stroke as a percentage of its stroke at last stance phase

// actuation phase transition parameters
// these are currently fixed and not exposed for easier teleop
float swing_percent_at_translate = leg_swing_percent;     // 0.5; percentage of swing leg retraction after which translation begins; values closer to 1 can cause swing legs to collide with rough terrains
float trans_percent_at_touchdown = 0.4;                   // 0.3; percentage of translatonal displacement from midpoint after which leg touchdown begins; values closer to 0 can result in leg touchdown before the translation completes, resulting in some backward motion after stance switch
float yaw_percent_at_touchdown = 0.9;                     // percentage of yaw command from midpoint after which leg touchdown begins; values closer to 0 can result in leg touchdown before the turning completes, resulting in some backward motion after stance switch

std::vector<float> q_leg_contact = {kQLegMax, kQLegMax, kQLegMax, kQLegMax};  // position of the swing leg actuators when they were last in contact
std::vector<float> q_leg_swing = {kQLegMin, kQLegMin, kQLegMin, kQLegMin};    // position setpoint of swing leg actuators during leg retraction
std::deque<int> idx_motor_mp;                                           // motors in touchdown as part of a motion primitive
uint32_t t_start_contact = 0;                                                 // time at which leg contact begins

uint8_t counts_steady_x = 0;                              // number of times a steay x command was received
uint8_t counts_steady_y = 0;                              // number of times a steay y command was received
uint8_t counts_steady_height = 0;                         // number of times a steady height command was received

float input_x_prev = 0;                                   // previous input_x_filtered
float input_y_prev = 0;                                   // previous input_y_filtered
float input_height_prev = 0.4;                            // previous input_height

void homeLeggedRobot() {
  snprintf(sent_data, sizeof(sent_data), "Homing all legs...\n\n");
  writeToSerial();

  // leg actuator homing
  for (uint8_t idx_motor = 0; idx_motor < kNumOfLegs/2; ++idx_motor) {
    motors[idx_motor].states_.current_limit = kCurrentLegMaxHoming;
    motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kVelocityControl;
    motors[idx_motor].states_.q_dot_d = kQdotLegHoming;
  }

  bool moving[4] = {true, true, true, true};
  uint32_t t_current = millis();

  // wait a minimum time to allow motors to start moving
  while (moving[0] || moving[1] || moving[2] || moving[3]) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();

    for (uint8_t idx_motor = 0; idx_motor < kNumOfLegs/2; ++idx_motor) {
      // if the motor has slowed down, it's at the joint limit
      if (moving[idx_motor] && millis() - t_current > 1000 && fabs(motors[idx_motor].states_.q_dot) < fabs(kQdotLegHomingStop)) {
        moving[idx_motor] = false;
        
        motors[idx_motor].states_.q_dot_d = 0;
        motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
        motors[idx_motor].states_.trap_traj_vel_limit = kVelLegTrajStandup;
        motors[idx_motor].states_.current_limit = kCurrentLegMax;
                
        motors[idx_motor].states_.homed = true;
        motors[idx_motor].states_.pos_home = motors[idx_motor].states_.pos_abs;
        motors[idx_motor].states_.pos_rel = motors[idx_motor].states_.pos_abs - motors[idx_motor].states_.pos_home;
        encoders[idx_motor*2].write(0);
        encoders[idx_motor*2 + 1].write(0);
        
        snprintf(sent_data, sizeof(sent_data), "Actuator %d homed.\n******************************************************\n", idx_motor + 1);
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

  for (uint8_t axis_id = stance*2; axis_id < stance*2 + 2; ++axis_id) {
    motors[axis_id].states_.q_d = kQLegMotorLift;
    motors[axis_id].states_.holding = false;
    motors[axis_id].states_.trap_traj_vel_limit = kVelLegTrajStandup;
    motors[axis_id].states_.trap_traj_accel_limit = kAccelLegTrajStandup;
    motors[axis_id].states_.trap_traj_decel_limit = kDecelLegTrajStandup;
    isInContact[axis_id] = true;
  }
  
  motors[gait_phase*2].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kTorqueControl;
  motors[gait_phase*2 + 1].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kTorqueControl;

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
    motors[axis_id].states_.trap_traj_accel_limit = kAccelLegTrajStandup;
    motors[axis_id].states_.trap_traj_decel_limit = kDecelLegTrajStandup;
  }

  // update states while standing up
  while (!motors[stance * 2].states_.holding && !motors[stance * 2 + 1].states_.holding) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    sendTelemetry();
  }

  // run necessary function calls and gait initialization here
  updateStanceTorque(stance);
  
  snprintf(sent_data, sizeof(sent_data), "Starting\n");
  writeToSerial();
}

void updateLocomotion() {
  uint32_t t_current = millis();
  if (t_current - t_last_setpoint_update >= k_dtSetpointUpdate) {
    t_last_setpoint_update = t_current;

    // calculate stability metrics here
    regulateBodyPose(); // if this function executes motions, updateSetpoint() is bypassed during the process
    updateGaitSetpoints();  // update setpoints for the motors in the swing phase
    checkStopCondition();
  }
}

// update desired motion vector, body height, trajectories, etc. based on higher-level input/planner
void updateTrajectory() {
  // apply deadzone to front-back joystick input to prevent undesired translation during in-place turning
  float input_x_filtered = max(abs(input_x) - kGUIJoystickXDeadZone, 0);
  int dir_x = (input_x > 0) - (input_x < 0);
  input_x_filtered *= dir_x/(1-kGUIJoystickXDeadZone);

  // apply deadzone to left-right joystick input to prevent undesired turning during translation
  float input_y_filtered = max(abs(input_y) - kGUIJoystickYDeadZone, 0);
  int dir_y = (input_y > 0) - (input_y < 0);
  input_y_filtered *= dir_y/(1-kGUIJoystickYDeadZone);

  if (millis() - t_last_Jetson > k_dtTrajectoryUpdate) {
    // use a counter to check that the commands have settled to a steady value
    if (fabs(input_x_filtered - input_x_prev) < EPS) {
      ++counts_steady_x;
    } else {
      input_x_prev = input_x_filtered;
      counts_steady_x = 0;
    }

    if (fabs(input_y_filtered - input_y_prev) < EPS) {
      ++counts_steady_y;
    } else {
      input_y_prev = input_y_filtered;
      counts_steady_y = 0;
    }

    if (fabs(input_height - input_height_prev) < EPS) {
      ++counts_steady_height;
    } else {
      input_height_prev = input_height;
      counts_steady_height = 0;
    }

    if (counts_steady_x > kMinCountsSteadyCmd) counts_steady_x = kMinCountsSteadyCmd + 1;
    if (counts_steady_y > kMinCountsSteadyCmd) counts_steady_y = kMinCountsSteadyCmd + 1;
    if (counts_steady_height > kMinCountsSteadyCmd) counts_steady_height = kMinCountsSteadyCmd + 1;
  }
  
  if (counts_steady_x > kMinCountsSteadyCmd) cmd_vector[0] = input_x_filtered;
  if (counts_steady_y > kMinCountsSteadyCmd) cmd_vector[1] = input_y_filtered;
  if (counts_steady_height > kMinCountsSteadyCmd) z_body_nominal = (kZBodyMax - kZBodyMin)*input_height + kZBodyMin;  // for now, nominal body height is equal to the leg actuator setpoint in stance

  leg_swing_percent = max(min(input_swing, kLegSwingPercentMax), kLegSwingPercentMin);  // bound the leg swing percentage with min/max
  swing_percent_at_translate = leg_swing_percent;                                       // set translation transition percentage equal to swing retraction percentage
                                                                                        // the idea is: if large retraction is needed, then translation should also occur later

  // update translation motor velocity based on step length and body height
  if (fabs(cmd_vector[0]) < kStepShort || z_body_nominal > kZBodyTall) {
    motors[MotorID::kMotorTranslate].states_.trap_traj_vel_limit = kVelTransTrajSlow;
  } else {
    motors[MotorID::kMotorTranslate].states_.trap_traj_vel_limit = kVelTransTraj;
  }
  
}

// executes blocking and non-blocking motions that regulate body pose
// refer to ReactiveBehaviors
void regulateBodyPose() {

  uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;
  float z_error = z_body_local - z_body_nominal;
  float dq_tilt = stance_width[gait_phase] * tan(rpy_lateral[gait_phase] * DEG2RAD);

  int dir_tipover[2] = {0, 0};  // direction of tipover; e.g. if pitch angle is negative, positive pitch velocity is stabilizing and negative pitch velocity is tipping
  for(auto i = 0; i < 2; ++i) {
    dir_tipover[i] = (rpy_lateral[i] > 0) - (rpy_lateral[i] < 0);
  }

  // check if the body is tipping over; two velocity ranges
  bool isTippingRoll = ((dir_tipover[0]*omega_filters[0].filtered_value) > kOmegaSoftMax_1 && fabs(rpy_lateral[0]) > kThetaSoftMax_1)
                      || ((dir_tipover[0]*omega_filters[0].filtered_value) > kOmegaSoftMax_2 && fabs(rpy_lateral[0]) > kThetaSoftMax_2);

  bool isTippingPitch = ((dir_tipover[1]*omega_filters[1].filtered_value) > kOmegaSoftMax_1 && fabs(rpy_lateral[1]) > kThetaSoftMax_1)
                       || ((dir_tipover[1]*omega_filters[1].filtered_value) > kOmegaSoftMax_2 && fabs(rpy_lateral[1]) > kThetaSoftMax_2);

  // slip recovery
  // executes a blocking maneuver where legs on the tipping side touch down
  // if only roll or pitch is detected, one leg pair will touch down
  // if both are detected, two leg pairs will touch down
  // at the end of this maneuver, if only one body's legs are in stance, regular gait cycle resumes
  // if the swing body also has a leg pair on the ground, that leg pair is commanded in position control
  // while the stance leg pairs are driven in torque control to regulate the Euler angle corresponding to the swing body
  // this will override an ongoing non-blocking motion primitives such as single-stance pose regulation

  if (!isBlocking                           // if currently there's no blocking motion primitive
      && !isScheduled                       // AND no motion primitive is scheduled
      && (isTippingRoll || isTippingPitch)  // AND the body is tipping over
     ){
                                                                                                                                          
    // stop the locomotion mechanism
    holdLocomotionMechanism();

    // determine which legs to touchdown
    if (isTippingRoll) {
      dir_tipover[0] > 0 ? idx_motor_mp.push_back(MotorID::kMotorLateralRight) : idx_motor_mp.push_back(MotorID::kMotorLateralLeft);
    }

    if (isTippingPitch) {
      dir_tipover[1] > 0 ? idx_motor_mp.push_back(MotorID::kMotorMedialFront) : idx_motor_mp.push_back(MotorID::kMotorMedialRear);
    }

    // for each leg pair touching down
    for (std::deque<int>::iterator idx_motor = idx_motor_mp.begin(); idx_motor != idx_motor_mp.end(); ++idx_motor) {
      updateMotorTorque(*idx_motor, torque_profile_touchdown[*idx_motor][0], kVelLegFootSlip);  // apply the torque command
      resetLegMotorContactState(*idx_motor);                                                    // clear the contact states on the new touchdown legs
      q_leg_swing[*idx_motor] = motors[*idx_motor].states_.q;                                   // update q_leg_swing for the touchdown legs, which is used in updateLegMotorContactState()
    }
    t_start_contact = millis(); // update the time variable for contact detection

    motion_primitive = ReactiveBehaviors::kSwingTorque;
    isBlocking = true;
    actuation_phase = ActuationPhases::kTouchDown;
  }
  
  // single-stance pose regulation (tilt and height)
  if (!motion_primitive                                                                        // if currently there's no motion primitive
      && actuation_phase == ActuationPhases::kLocomote                                        // AND in single stance
      && (fabs(z_error) > kZErrorSoftMax || fabs(rpy_lateral[gait_phase]) > kThetaNominal)    // AND body pose error is large
     ){

    std::vector<float> dq_stance{0, 0};

    // regulate body tilt
    if (fabs(rpy_lateral[gait_phase]) > kThetaNominal) {
      dq_stance[0] += dq_tilt / 2;
      dq_stance[1] -= dq_tilt / 2;
    }

    // regulate body height only during lateral body swing for better stability
    if (fabs(z_error) > kZErrorSoftMax
       && gait_phase == GaitPhases::kLateralSwing) {
      dq_stance[0] -= z_error;
      dq_stance[1] -= z_error;
    }

    updateBodyLegsPosition(stance, dq_stance);   // move stance legs
    idx_motor_mp.push_back(stance*2);
    idx_motor_mp.push_back(stance*2 + 1);

    // adjust swing legs for ground clearance
    float dq_leg_max_clearance = min(dq_stance[0], dq_stance[1]);   // greater of the two leg retractions or lesser of the two leg extensions
    if (dq_leg_max_clearance < -20) {                               // enforce minimum displacement to prevent frequent swing leg movements/jitters; only move swing legs upward
      std::vector<float> dq_swing{dq_leg_max_clearance, dq_leg_max_clearance};
      updateBodyLegsPosition(gait_phase, dq_swing);
      q_leg_swing[gait_phase*2] = motors[gait_phase*2].states_.q_d;
      q_leg_swing[gait_phase*2 + 1] = motors[gait_phase*2 + 1].states_.q_d;
    }

    motion_primitive = ReactiveBehaviors::kStancePosition;

  // double-stance pose regulation (height only)
  } else if (!motion_primitive                                                       // if currently there's no motion primitive
             && actuation_phase == ActuationPhases::kTouchDown                      // AND in double stance
             && isInContact[gait_phase * 2] && isInContact[gait_phase * 2 + 1]
             && fabs(z_error) > kZErrorHardMax                                      // AND height error is large
            ){

      std::vector<float> dq_stance{-z_error, -z_error};
      updateBodyLegsPosition(GaitPhases::kLateralSwing, dq_stance);
      updateBodyLegsPosition(GaitPhases::kMedialSwing, dq_stance);
      for (auto i = 0; i < 4; ++i) { idx_motor_mp.push_back(i); }

      motion_primitive = ReactiveBehaviors::kStancePosition;
      isBlocking = true;
  }
  
  // check if an ongoing motion primitive is completed and call the callback function
  if (motion_primitive) {

    bool done = false;

    // if stance legs are in position control
    if (motion_primitive == ReactiveBehaviors::kStancePosition) {

      for (std::deque<int>::iterator idx_motor = idx_motor_mp.begin(); idx_motor != idx_motor_mp.end(); ++idx_motor) {
        done = done && motors[*idx_motor].states_.holding;
      }

      if (done) {
        updateStanceBodyTorque(stance);

        idx_motor_mp.clear();
        motion_primitive = ReactiveBehaviors::kNone;
        isBlocking = false;
      }

    // if swing legs are in torque control
    } else if (motion_primitive == ReactiveBehaviors::kSwingTorque) {

      // check the contact state of all legs for this motion primitive
      for (std::deque<int>::iterator idx_motor = idx_motor_mp.begin(); idx_motor != idx_motor_mp.end(); ++idx_motor) {
        updateLegMotorContactState(*idx_motor);
        updateStanceTorque(*idx_motor);
        done = done && isInContact[*idx_motor];
      }
      
      if (done) {
        idx_motor_mp.clear();
        motion_primitive = ReactiveBehaviors::kNone;
        isBlocking = false;   // the blocking portion is over, but slip recovery is not finished until tilt correct is completed
        isScheduled = true;   // this is cleared when the next motion primmitive is completed
      }
    }
  }
}

// returns true if ready for transition to the next actuation phase
bool isReadyForTransition(uint8_t phase) {
  
  if (phase == ActuationPhases::kRetractLeg) {  // if currently retracting leg
    float q_leg_transition_1 = q_leg_swing[gait_phase*2] + swing_percent_at_translate*(q_leg_contact[gait_phase*2] - q_leg_swing[gait_phase*2]);
    float q_leg_transition_2 = q_leg_swing[gait_phase*2 + 1] + swing_percent_at_translate*(q_leg_contact[gait_phase*2 + 1] - q_leg_swing[gait_phase*2 + 1]);

    return motors[gait_phase*2].states_.q < q_leg_transition_1 && motors[gait_phase*2 + 1].states_.q < q_leg_transition_2;

  } else if (phase == ActuationPhases::kLocomote) { // if currently translating or turning
    bool isTranslated = false, isTurned = false;

    // if there is a translation command
    if (fabs(cmd_vector[0]) > EPS) {
      int dir = (cmd_vector[0] > 0) - (cmd_vector[0] < 0);                                              // direction of translation command
      float q_trans_transition = fabs(trans_percent_at_touchdown*motors[MotorID::kMotorTranslate].states_.q_d);             // this value is always positive since it represents forward motion, regardless of direction
      isTranslated = dir * pow(-1, gait_phase + 1) * q[JointID::kJointTranslate] > q_trans_transition;  // the translational joint has reached the transition point
                                                                                                        // don't check yaw; assume that any concurrent yaw motion will be completed in time

    // if there is an in-place turn command
    } else if (fabs(cmd_vector[1]) > EPS) {
      // int dir = (cmd_vector[1] > 0) - (cmd_vector[1] < 0);                              // direction of yaw command
      // float q_yaw_transition = fabs(yaw_percent_at_touchdown*motors[MotorID::kMotorYaw].states_.q_d);   // this value is always positive since it represents forward motion, regardless of direction
      // isTurned = dir * pow(-1, gait_phase) * q[JointID::kJointYaw] > q_yaw_transition;  // the yaw joint has reached the transition point
      isTurned = fabs(motors[MotorID::kMotorYaw].states_.q - motors[MotorID::kMotorYaw].states_.q_d) < 3;
    }

    return isTranslated || isTurned;

  } else if (phase == ActuationPhases::kTouchDown) {  // if currently touching down

    float z_error = z_body_local - z_body_nominal;
        
    return isInContact[gait_phase * 2] && isInContact[gait_phase * 2 + 1]
           && ((fabs(z_error) < kZErrorHardMax)
           ||
           (fabs(z_error) > kZErrorHardMax && !(fabs(rpy_lateral[0]) < kThetaNominal && fabs(rpy_lateral[1]) < kThetaNominal)));  // true if both swing legs have made contact and body height regulation is not needed; deleting the body height condition results the motion primitive often being skipped

  } else {
    return false;
  }
}

// touchdown: torque control
// translate: position control
// leg retract: position control
void updateGaitSetpoints() {
  // update setpoints at gait phase transitions here
  if (!isBlocking && isReadyForTransition(actuation_phase)) {

    if (actuation_phase == ActuationPhases::kRetractLeg) {  // if currently retracting leg

      if (fabs(cmd_vector[0]) < EPS && fabs(cmd_vector[1]) < EPS) {   // if no translation or yaw command
        holdLocomotionMechanism();

      } else {  // else, move according to the current command
        moveLocomotionMechanism();
      }

    } else if (actuation_phase == ActuationPhases::kLocomote) { // if currently translating or turning
      updateTouchdown(gait_phase, kVelLegMaxContact);
      moveLocomotionMechanism();      // reapply locomotion mechanism setpoints in the case of resuming locomotion from standstill
      
    } else if (actuation_phase == ActuationPhases::kTouchDown) {        // if currently touching down
      updateStanceTorque(gait_phase);

      // advance the gait phase
      gait_phase = (gait_phase + 1) % kNumOfGaitPhases;
      if (gait_phase == 0) {
        gait_cycles++; // if the gait phase is back to 0, increment the number of completed gait cycles
      }

      updateRetract();
      resetBodyLegContactState(gait_phase);
    }

    actuation_phase = (actuation_phase + 1) % kNumOfActuationPhases;

    // update setpoints that do not involve an actuation phase transition here
  } else if (!isBlocking) {
    if (actuation_phase == ActuationPhases::kRetractLeg) {          // if currently retracting leg
      
    } else if (actuation_phase == ActuationPhases::kLocomote) {     // if currently translating or turning
      
      if (fabs(cmd_vector[0]) < EPS && fabs(cmd_vector[1]) < EPS) {   // if no translation or yaw command
        holdLocomotionMechanism();

      } else {  // else, move according to the current command
        moveLocomotionMechanism();
      }
      
    } else if (actuation_phase == ActuationPhases::kTouchDown) {    // if currently touching down
      updateTouchdownTorque(gait_phase);
      updateStanceTorque(gait_phase);
    }
  }
}

void checkStopCondition() {
  if (fabs(rpy_lateral[0]) > kThetaHardMax || fabs(rpy_lateral[1]) > kThetaHardMax) {
    stop_signal = true;
  }
}

void updateTouchdown(uint8_t idx_body, float vel_limit) {
  for (uint8_t idx_motor = idx_body*2; idx_motor < idx_body*2 + 2; ++idx_motor) {
    updateMotorTorque(idx_motor, torque_profile_touchdown[idx_motor][1], vel_limit);
    if (!isNotStuck(idx_motor)) {
      updateMotorTorque(idx_motor, torque_profile_touchdown[idx_motor][0], vel_limit);
    }
  }
  t_start_contact = millis();
}

void updateTouchdownTorque(uint8_t idx_body) {
  for (uint8_t idx_motor = idx_body*2; idx_motor < idx_body*2 + 2; ++idx_motor) {

    // if the motor is not in contact
    if (!isInContact[idx_motor]) {
      if ((motors[idx_motor].states_.q  - q_leg_swing[idx_motor]) > kDqLegMotorStartup  // if past the startup displacement
          && isNotStuck(idx_motor)) {                                           // AND the legs have moved away from the joint limit
        motors[idx_motor].states_.tau_d = torque_profile_touchdown[idx_motor][2];       // lower the leg torque
      }
    }
  }
}

// updates the given stance leg motor torque to zero, leveraging the non-backdrivability
void updateStanceTorque(uint8_t idx_motor) {
  if(isInContact[idx_motor]) {                                    // cannot be in stance without being in contact
    updateMotorTorque(idx_motor, 0);
    motors[idx_motor].states_.q_d = motors[idx_motor].states_.q;  // set desired position for telemetry purposes and possible control changes
    q_leg_contact[idx_motor] = motors[idx_motor].states_.q;       // remember the leg motor position at ground contact
  }
}

// updates the given stance body leg motors' torque to zero
void updateStanceBodyTorque(uint8_t idx_body) {
  for (uint8_t idx_motor = idx_body*2; idx_motor < idx_body*2 + 2; ++idx_motor) {
    updateStanceTorque(idx_motor);
  }
}

// determine swing leg setpoints based on contact conditions and update the motor control mode and limits for swing phase
void updateRetract() {
  for (uint8_t idx_motor = gait_phase*2; idx_motor < gait_phase*2 + 2; ++idx_motor) {

    q_leg_contact[idx_motor] = motors[idx_motor].states_.q;             // remember the leg motor position at ground contact
    float q_leg_retract = q_leg_contact[idx_motor]*leg_swing_percent;   // nominal swing leg setpoint; NOT necessarily equal to the actual setpoint
    if (q_leg_contact[idx_motor]*(1 - leg_swing_percent) < kDqLegSwingMin) q_leg_retract = q_leg_contact[idx_motor] - kDqLegSwingMin; // enforce minimum required swing retraction

    // TODO: change this logic to account for robot kinematics (body tilt)
    if (fabs(q[idx_motor*2] - q[idx_motor*2 + 1]) > kDqLegUnevenTerrain) { // if the previous stance legs are standing on uneven ground
      float q_max = max(q[idx_motor*2], q[idx_motor*2 + 1]);            // calculate the additional leg stroke due to uneven terrain
      float dq = q_max - motors[idx_motor].states_.q;
      
      motors[idx_motor].states_.q_d = max(kQLegMin + 5, q_leg_retract - dq); // retract more by dq to ensure proper ground clearance

    } else {  
      motors[idx_motor].states_.q_d = q_leg_retract;  // if even terrain, use the nominal swing leg setpoint
    }

    // remember the swing leg position setpoint
    q_leg_swing[idx_motor] = motors[idx_motor].states_.q_d;

    // update the motor control mode and limits for swing phase
    motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
    motors[idx_motor].states_.holding = false;
    motors[idx_motor].states_.velocity_limit = kVelLegMax;
    motors[idx_motor].states_.trap_traj_vel_limit = kVelLegTrajSwing;
    motors[idx_motor].states_.trap_traj_accel_limit = kAccelLegTrajSwing;
    motors[idx_motor].states_.trap_traj_decel_limit = kDecelLegTrajSwing;
  }
}

void updateBodyLegsPosition(uint8_t idx_body, std::vector<float> dq) {
  for (uint8_t idx_motor = idx_body*2; idx_motor < idx_body*2 + 2; ++idx_motor) {
    uint8_t i = idx_motor - idx_body*2;
    motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;
    motors[idx_motor].states_.holding = false;
    motors[idx_motor].states_.trap_traj_vel_limit = kVelLegTrajStandup;
    motors[idx_motor].states_.trap_traj_accel_limit = kAccelLegTrajStandup;
    motors[idx_motor].states_.trap_traj_decel_limit = kDecelLegTrajStandup;
    motors[idx_motor].states_.q_d = motors[idx_motor].states_.q + dq[i];

    // stay away from leg joint limits
    if (motors[idx_motor].states_.q_d < motors[idx_motor].states_.q_min) {
      motors[idx_motor].states_.q_d = motors[idx_motor].states_.q_min + kDqJointLimit;
    } else if (motors[idx_motor].states_.q_d > motors[idx_motor].states_.q_max) {
      motors[idx_motor].states_.q_d = motors[idx_motor].states_.q_max - kDqJointLimit;
    }
  }
}

void moveLocomotionMechanism() {
  float q_trans = pow(-1, gait_phase + 1) * kQTransMax*cmd_vector[0];
  float q_yaw = pow(-1, gait_phase)*kQYawMax*cmd_vector[1];
  if (fabs(kQTransMax - fabs(q_trans)) < kDqTransEndYawLimit) q_yaw = pow(-1, gait_phase)*(kQYawMax - kDqYawLimit)*cmd_vector[1];
  motors[MotorID::kMotorTranslate].states_.q_d = q_trans;
  motors[MotorID::kMotorYaw].states_.q_d = q_yaw;
}

void holdLocomotionMechanism() {
  motors[MotorID::kMotorTranslate].states_.q_d = motors[MotorID::kMotorTranslate].states_.q;  // stop at current translation and yaw
  motors[MotorID::kMotorYaw].states_.q_d = 0;
}