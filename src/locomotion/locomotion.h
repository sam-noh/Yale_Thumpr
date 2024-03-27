#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include "Arduino.h"
#include <vector>

const int kNumOfGaitPhases = 2;             // number of gait phases in a gait cycle
const int kNumOfActuationPhases = 3;        // number of actuation phases in swing

enum GaitPhases {
    kMedialSwing = 0,          // medial body swing phase & lateral body stance phase
    kLateralSwing = 1          // lateral body swing phase & medial body stance phase
};

enum ActuationPhases {
    kRetractLeg = 0,
    kLocomote = 1,
    kTouchDown = 2
};

// touchdown torque profile parameters
const float kDqLegStartup = 10;         // leg touchdown displacement after which a lower torque is applied
const float kDqLegRamp = 60;            // leg touchdown displacement after which an even lower torque is applied

// kRetractLeg parameters
const float kDqUnevenTerrain = 50;      // leg pair stroke difference in mm greater than which the terrain is assumed to be uneven

// body height regulation parameters
const float k_zBodyMin = 100;           // minimum allowable value for z_body_local
const float k_zBodyMax = 300;           // maximum allowable value for z_body_local

// gait cycle parameter limits
const float kLegSwingPercentMax = 0.9;
const float kLegSwingPercentMin = 0.2;

// blocking motion primitive parameters
const float kDqSwingLegClearance = 10;  // swing leg vertical clearance margin when performing body height or tilt regulation; assumes flat terrain; adjust accordingly
const float kQTransCentered = 20;       // distance from translational joint midpoint within which non-blocking motion primitives are allowed; used to ensure stable support boundary
const float k_zErrorSoftMax = 30;       // body height deviation in mm above which non-blocking regulation is executed
const float k_zErrorHardMax = 60;       // body height deviation in mm above which blocking regulation is executed
const float kTiltNominal = 3;           // acceptable body tilt from zero in degrees
const float kDqLegMaxTilt = 100;        // max total leg displacements per tilt correction
const float kQdotStable = 15;           // leg motor velocity in mm/s below which the blocking of normal gait behavior ends

// motor torque setpoints during leg touchdown; determined heuristically
// the first torque is the minimum necessary to initiate motion
// the second torque command is the minimum necessary to maintain motion
// after the motor moves "kDqStartContactLowImpulse" mm, the second command is sent
extern std::vector<std::vector<float>> touchdown_torque;

// gait variables
extern std::vector<float> cmd_vector;   // command vector: {forward-back, yaw angle}
                                        // if the vector is zero, the robot will stop at the next stance phase; to be extended to 3D
                                        // follows IMU "coordinate frame"; x-positive: forward, y-positive: left, z-positive: up
                                        // currently take values from the normalized joystick inputs after deadzone and max value compensation (0 to 1)

extern uint8_t gait_phase;              // current gait phase; (0 medial swing/lateral stance) -> (1 medial stance/lateral swing); double support is omitted
extern uint8_t actuation_phase;         // current actuation phase of the swing legs; 0 retract -> 1 translate -> 2 touchdown
extern uint32_t gait_cycles;            // number of completed gait cycles
extern bool isBlocking;                 // true if any motion primitive outside of the standard gait cycle is in progress
extern bool isCorrected;                // true if a motion primitive has been completed during the gait cycle

// nominal leg trajectory parameters; can be updated by a high-level planner
// exact trajectory is determined by the motor controller's trapezoidal trajectory generation: acceleration, deceleration, max velocity
extern float z_body_nominal;                // nominal body height over local terrain; currently taken as avg of stance leg motors joint position
extern float leg_swing_percent;             // swing leg stroke as a percentage of its stroke at last stance phase

extern std::vector<float> q_leg_contact;    // position of the swing leg actuators when they were last in contact
extern std::vector<float> q_leg_swing;      // position setpoint of swing leg actuators during leg retraction

void homeLeggedRobot();

void standUp();

void updateGait();

void updateTrajectory();

void regulateBodyPose();

bool isReadyForTransition(uint8_t phase);

void updateSetpoints();

void updateMotorsTouchdown();

void updateTouchdownTorque();

// updates the specified stance body's leg motors for zero torque stance, leveraging the non-backdrivable legs
void updateMotorsStance(uint8_t stance);

// determine swing leg setpoints based on contact conditions and update the motor control mode and limits for swing phase
void updateMotorsSwing();

void moveLocomotionMechanism();

void holdLocomotionMechanism();

#endif