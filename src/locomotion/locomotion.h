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
    kTranslateForward = 1,
    kTouchDown = 2
};

// kRetractLeg parameters
const float kDqUnevenTerrain = 20;      // leg pair stroke difference in mm greater than which the terrain is assumed to be uneven

// kTouchDown/contact detection parameters
const float kDqStartContact = 20;       // leg displacment in mm past which contact detection begins
const float kQdotContact = 0.5;         // joint velocity in mm/s below which contact is likely

// blocking motion primitive parameters
const float kdzMax = 40;                // maximum allowed body height deviation in mm
const float kTiltNominal = 3;           // acceptable body tilt from zero in degrees
const float kDqLegMaxTilt = 100;        // max total leg displacements per tilt correction
const float kGyroStable = 0.04;         // calibrated gyro output value below which the robot body is considered to be not oscillating due to dynamics

// motor torque setpoints during leg touchdown; determined heuristically
// the first torque is the minimum necessary to initiate motion
// the second torque command is the minimum necessary to maintain motion
// after the motor moves "kDqStartContact" mm, the second command is sent
extern std::vector<std::vector<float>> touchdown_torque;

// gait variables
extern std::vector<float> cmd_vector;   // command vector: {forward-back, right-left, yaw angle}
                                        // if the vector is zero, the robot will stop at the next stance phase; to be extended to 3D
                                        // follows IMU "coordinate frame"; x-positive: forward, y-positive: left, z-positive: up
                                        // currently take values from the normalized joystick inputs after deadzone and max value compensation (0 to 1)

extern uint8_t gait_phase;              // current gait phase; (0 medial swing/lateral stance) -> (1 medial stance/lateral swing); double support is omitted
extern uint8_t actuation_phase;         // current actuation phase of the swing legs; 0 retract -> 1 translate -> 2 touchdown
extern uint32_t gait_cycles;            // number of completed gait cycles
extern std::vector<int> inContact;      // true if the motor's current exceeds the threshold during touchdown; stays true until legs lift
extern bool isBlocking;                 // true if any motion primitive outside of the standard gait cycle is in progress
extern bool corrected;                  // true if a motion primitive has been completed during the gait cycle

// nominal leg trajectory parameters; can be updated by a high-level planner
// exact trajectory is determined by the motor controller's trapezoidal trajectory generation: acceleration, deceleration, max velocity
extern float q_leg_stance;              // nominal (lower) leg stroke in stance (tested 250)
extern float q_leg_swing;               // nominal (lower) leg stroke at max clearance in swing (tested 35 and 70)
extern float q_trans;                   // nominal translation position in swing
extern float q_yaw;                     // nominal yaw rotation per step

extern float dq_leg_trans;              // remaining swing leg retraction at which forward translation begins (tested 50)
extern float q_trans_transition;         // translation position at which swing leg step down begins; negative value is before the midpoint regardless of gait_phase
extern float z_body_nominal;            // nominal body height over local terrain; currently taken as avg of stance leg motors joint position

// motor setpoints that parameterize leg motion
// 1. leg stroke at max clearance in swing; can change to speed up locomotion or clear taller obstacles
// 2. translation position in swing; can change each step based on higher level planning
// 3. leg stroke in stance; can change each step based on proprioceptive sensing (terrain slope change)
extern std::vector<std::vector<float>> gait_setpoints;

extern std::vector<float> q_leg_contacts;

void homeLeggedRobot();

void standUp();

void updateGait();

void updateCmdVector();

void regulateBodyPose();

bool isReadyForTransition(uint8_t phase);

void updateSetpoints();

void updateContactState();

#endif