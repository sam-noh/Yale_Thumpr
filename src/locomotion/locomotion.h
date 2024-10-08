#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include "Arduino.h"
#include <vector>
#include <deque>
#include <functional>
#include <algorithm>

#define USE_TIPOVER_RECOVERY
#define USE_TELEOP

const int kNumOfGaitPhases = 2;             // number of gait phases in a gait cycle
const int kNumOfActuationPhases = 3;        // number of actuation phases in swing

// enum for swing phase body
enum GaitPhases {
    kMedialSwing = 0,          // medial body swing phase & lateral body stance phase
    kLateralSwing = 1          // lateral body swing phase & medial body stance phase
};

// enum for actuation phase
// transitions are handled by updateGaitSetpoints()
// can be overridden by motion primitives in regulateBodyPose()
enum ActuationPhases {
    kRetractLeg = 0,    // legs retract for swing phase
    kLocomote = 1,      // body translates and/or turns based on command; legs may be still retracting
    kTouchDown = 2      // legs touch down; body may be still translating or turning
};

// enum for reactive behaviors
enum ReactiveBehaviors {
    kNone = 0,              // no ongoing reactive behavior
    kStancePosition = 1,    // stance legs in position control; currently used in single and double stance pose regulation
    kStanceTorque = 2,      // stance legs in torque control; currently used for the stance legs during single-side tilt correct; when kSwingPosition and kStanceTorque are both active, motion_primitive is set to kSwingPosition
    kSwingPosition = 3,     // swing legs in position control; currently used for swing leg clearance and single-side tilt correct
    kSwingTorque = 4,       // swing legs in torque control; currently used for slip recovery (when slipping, all legs are assumed to be in swing and their contact states are checked again)
    kTransPosition = 5,     // translation motor in position control; currently not used
    kTransTorque = 6        // translation motor in torque control; currently not used
};

// homing sequence parameters
const float kQLegMotorLift = 60;        // leg motor position setpoint for homing the translation joint

// higher-level command related parameters
const uint8_t kMinCountsSteadyCmd = 5;     // minimum number of steady input values before the command changes; used to filter out input values changing or dropping out

// leg touchdown parameters
const float kDqLegMotorStartup = 20;         // leg touchdown displacement after which a lower torque is applied
const uint32_t kDtTouchdown = 2000;          // minimum time duration for the touchdown phase

// kRetractLeg parameters
const float kDqLegUnevenTerrain = 100;  // leg pair stroke difference in mm greater than which the terrain is assumed to be uneven
const float kDqLegSwingMin = 60;        // minimum required leg retraction during swing

// swing leg control parameters
const float kMinDqClearance = -20;      // minimum required leg retraction for swing leg adjustment during single-stance pose regulation

// body height regulation parameters
const float kZBodyMin = 100;            // minimum allowable value for z_body_local
const float kZBodyMax = 550;            // maximum allowable value for z_body_local

// translational inertia related parameters
const float kZBodyTall = 300;           // body height in mm above which translational velocity is reduced
const float kStepShort = 0.4;           // cmd_vector[0] below which translational velocity is reduced; to be updated to joint space value

// gait cycle parameter limits
const float kLegSwingPercentMax = 0.9;
const float kLegSwingPercentMin = 0.2;

// energy stabiliy margin parameters
const float kDqTransCentered = 20;      // displacement from the translational joint midpoint in mm below which the two bodies are considered to be centered
const float kZBodyStepScaleMin = 250;
const float kZBodyStepScaleMax = 400;
const float kTerrainPitchMin = 5;       // minimum terrain pitch in degrees over which step length is scaled
const float kTerrainPitchMax = 30;      // maximum terrain pitch in degrees used for scaling the step length over terrain slopes

// motion primitive parameters
const float kZErrorSoftMax = 20;        // body height deviation in mm above which non-blocking regulation is executed
const float kZErrorHardMax = 60;        // body height deviation in mm above which blocking regulation is executed
const float kThetaNominal = 1;          // body Euler angle above which non-blocking regulation is executed
const float kThetaSoftMax_1 = 4;        // body Euler angle above which slip recovery is executed; used with kOmegaSoftMax_1
const float kThetaSoftMax_2 = 6;        // body Euler angle above which slip recovery is executed; used with kOmegaSoftMax_2
const float kThetaSoftMax_3 = 8;      // body Euler angle above which slip recovery is executed
const float kOmegaSoftMax_1 = 25;       // body angular velocity above which slip recovery is executed at kThetaSoftMax_1 or greater tilt
const float kOmegaSoftMax_2 = 12;       // body angular velocity above which slip recovery is executed at kThetaSoftMax_2 or greater tilt
const float kOmegaSoftMax_3 = 7;        // body angular velocity above which slip recovery is executed at kThetaSoftMax_3 or greater tilt
const float kThetaHardMax = 40;         // body Euler angle above which robot is stopped

extern std::vector<float> q_trans_traj;     // translation joint trajectory setpoints
extern int32_t idx_q_trans_traj;           // index of the current translation joint trajectory setpoint

// motor torque setpoints during leg touchdown; determined heuristically
// the first torque is the minimum necessary to initiate motion
// the second torque command is the minimum necessary to maintain motion
// after the motor moves "kDqLegMotorStartContact" mm, the second command is sent
extern std::vector<std::vector<float>> torque_profile_touchdown;

// gait variables
extern std::vector<float> cmd_vector;   // command vector: {forward-back, yaw angle}
                                        // if the vector is zero, the robot will stop at the next stance phase; to be extended to 3D
                                        // follows IMU "coordinate frame"; x-positive: forward, y-positive: left, z-positive: up
                                        // currently take values from the normalized joystick inputs after deadzone and max value compensation (0 to 1)

extern uint8_t gait_phase;              // current gait phase; (0 medial swing/lateral stance) -> (1 medial stance/lateral swing); double support is omitted
extern uint8_t actuation_phase;         // current actuation phase of the swing legs; 0 retract -> 1 translate -> 2 touchdown
extern uint32_t gait_cycles;            // number of completed gait cycles
extern uint8_t motion_primitive;        // current motion primitive
extern bool isBlocking;                 // true if any motion primitive outside of the standard gait cycle is in progress
extern bool isScheduled;                // true if a motion primitive is scheduled for execution
extern bool isScheduledTrans;           // true if a motion primitive is scheduled for the translation joint

// nominal leg trajectory parameters; can be updated by a high-level planner
// exact trajectory is determined by the motor controller's trapezoidal trajectory generation: acceleration, deceleration, max velocity
extern float z_body_nominal;                // nominal body height over local terrain; currently taken as avg of stance leg motors joint position
extern float leg_swing_percent;             // swing leg stroke as a percentage of its stroke at last stance phase
extern std::vector<float> q_trans_limit;    // [q_trans_min, q_trans_max]; the two values will change signs and values according to the current gait phase, terrain slope and body tilt
extern float q_trans_prev;                  // translation joint position at last ground contact; used for phase transition check

extern std::vector<float> q_leg_contact;            // position of the swing leg actuators when they were last in contact
extern std::vector<float> q_leg_swing;              // position setpoint of swing leg actuators during leg retraction
extern std::deque<int> idx_motor_mp;          // motors in touchdown as part of a motion primitive
extern uint32_t t_start_contact;                    // time at which leg contact begins

extern uint8_t counts_steady_x;             // number of times a steay x command was received
extern uint8_t counts_steady_y;             // number of times a steay y command was received
extern uint8_t counts_steady_height;        // number of times a steady height command was received

extern float input_x_prev;                  // previous input_x_filtered
extern float input_y_prev;                  // previous input_y_filtered
extern float input_height_prev;             // previous input_height

void homeLeggedRobot();

void standUp();

void updateLocomotion();

void updateTrajectory();

void regulateBodyPose();

bool isReadyForTransition(uint8_t phase);

void updateGaitSetpoints();

void checkStopCondition();

void updateTouchdown(uint8_t idx_body, float vel_limit);

void updateTouchdownTorque(uint8_t idx_body);

// updates the specified stance body's leg motors for zero torque stance, leveraging the non-backdrivable legs
void updateStanceTorque(uint8_t idx_body);

// updates the given stance body leg motors' torque to zero
void updateStanceBodyTorque(uint8_t idx_body);

// determine swing leg setpoints based on contact conditions and update the motor control mode and limits for swing phase
void updateRetract();

// limit leg motor torque during retraction if at least one leg is near the joint limit
// to prevent the leg from locking up due to belt tension
void limitRetractionTorque(uint8_t idx_body);

void updateLegPosition(uint8_t idx_motor, float dq, float vel_lim);

void updateBodyLegsPosition(uint8_t idx_body, std::vector<float> dq, float vel_lim);

void moveLocomotionMechanism();

void holdLocomotionMechanism();

#endif