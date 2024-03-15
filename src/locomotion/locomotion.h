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
const float kDqUnevenTerrain = 50;      // leg pair stroke difference in mm greater than which the terrain is assumed to be uneven

// body height regulation parameters
const float k_zBodyMin = 100;           // minimum allowable value for z_body_local
const float k_zBodyMax = 300;           // maximum allowable value for z_body_local

// blocking motion primitive parameters
const float kdzMax = 60;                // maximum allowed body height deviation in mm
const float kTiltNominal = 3;           // acceptable body tilt from zero in degrees
const float kDqLegMaxTilt = 100;        // max total leg displacements per tilt correction
const float kOmegaStable = 5;           // lateral body angular velocity value in deg/s below which the blocking of normal gait behavior ends
const float kQdotStable = 15;           // leg motor velocity in mm/s below which the blocking of normal gait behavior ends

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
extern bool isCorrected;                // true if a motion primitive has been completed during the gait cycle

// nominal leg trajectory parameters; can be updated by a high-level planner
// exact trajectory is determined by the motor controller's trapezoidal trajectory generation: acceleration, deceleration, max velocity
extern float leg_swing_percent;             // swing leg stroke as a percentage of its stroke at last stance phase
extern float q_trans;                       // nominal translation position in swing

extern float swing_percent_at_translate;    // remaining swing leg retraction at which forward translation begins (tested 50)
extern float q_trans_transition;            // translation position at which swing leg step down begins; negative value is before the midpoint regardless of gait_phase
extern float z_body_nominal;                // nominal body height over local terrain; currently taken as avg of stance leg motors joint position
extern float dz_body_local;                 // amount of body height change applied

extern std::vector<float> q_leg_contact;    // position of the swing leg actuators when they were last in contact
extern std::vector<float> q_leg_swing;      // position setpoint of swing leg actuators during leg retraction

void homeLeggedRobot();

void standUp();

void updateGait();

void updateTrajectory();

void regulateBodyPose();

bool isReadyForTransition(uint8_t phase);

void updateSetpoints();

#endif