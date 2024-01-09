#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <vector>
#include "..\actuator\actuator.h"

//////////////////////////////////////////////////////////////////////////////////////
// constants

// ODrive Pro CAN parameters
const int kODriveCANBaudRate = 1000000;     // baud rate for ODrive CAN2.0
const int kAxisIDLength = 6;                // number of bits of the CAN message's axis ID
const int kCmdIDLength = 5;                 // number of bits of the CAN message's command ID

// motor speed constants
const float k_KV_to_KT = 8.27;              // conversion factor from speed constant to torque constant
const float k_KV_D5312S = 330;              // speed constant of the leg motor
const float k_KV_GL60 = 25;                 // speed constant of the trans/yaw motor

// positive motor direction
const int kDirectionLeg = -1;               // positive direction for leg motors
const int kDirectionTrans = 1;              // positive direction for translation motor
const int kDirectionYaw = 1;                // positive direction for yaw motor

// motor homing parameters in joint space
// should be determined heuristically
const float kQdotLegHoming = -50;             // leg motor homing velocity in mm/s
const float kQdotLegHomingStop = -5;         // leg motor homing stop condition velocity in mm/s

const float kQdotTransHoming = 80;            // translation motor homing velocity in mm/s
const float kQdotTransHomingStop = 5;         // translation motor homing stop condition velocity in mm/s
const float kQTransHomingOffset = -110;       // distance from the joint limit to the translation zero position in mm

const float kQYawHome = 47.7;                  // UPDATE VALUE AFTER YAW MOTOR REASSEMBLY; yaw home position in degrees
const float kQdotYawHomingStop = 0.2;          // yaw motor homing stop condition velocity in deg/s

// motor velocity (1) absolute limits and (2) trapezoidal trajectory limits
const float kVelLegMax = 25;                    // leg motor velocity limit in turns/s
const float kVelLegMaxContact = 15;             // leg motor velocity limit during touchdown in turns/s
const float kVelTransMax = 5;                   // translation motor velocity limit in turns/s

const float kVelLegTrajSwing = kVelLegMax;      // leg motor trapezoidal trajectory velocity in swing phase in turns/s
const float kVelLegTrajStandup = 15;            // leg motor trapezoidal trajectory velocity when standing up in turns/s
const float kVelLegTrajTilt = 6;                // leg motor trapezoidal trajectory velocity when tilt correcting in turns/s (tested 10)
const float kVelLegTrajRetract = 2;             // leg motor trapezoidal trajectory velocity when stance switching in turns/s

// motor trapezoidal trajectory parameters
const float kAccelLegTrajSwing = 100;           // leg motor trapezoidal trajectory acceleration when retracting legs in turns/s^2
const float kDecelLegTrajSwing = 80;            // leg motor trapezoidal trajectory deceleration when retracting legs in turns/s^2
const float kDecelLegTrajTilt = 10;             // leg motor trapezoidal trajectory deceleration when tilt correcting in turns/s^2

// motor current limits
const float kCurrentLegMax = 40;                // leg motor current limit in ampere
const float kCurrentLegMaxHoming = 8;          // leg motor current limit while homing in ampere
const float kCurrentTransMax = 4;               // translation motor current limit in ampere
const float kCurrentTransMaxHoming = 0.75;      // translation motor current limit while homing in ampere

// motor position control parameters
const float kQErrorMax = 5;                     // maximum allowable position error in mm

//////////////////////////////////////////////////////////////////////////////////////
// global variables

// ODrive CAN object and actuator objects
extern ODriveTeensyCAN ODrive_CAN;
extern std::vector<Actuator> motors;

//////////////////////////////////////////////////////////////////////////////////////
// global functions

// reads the next CAN msg from an ODrive CAN node
// calls member variables of Actuator objects
uint32_t handleODriveCANMsg();

// extracts and returns the axis/node ID from the ODrive's CAN message
uint8_t getAxisID(uint32_t msg_id);

// extracts and returns the command ID from the ODrive's CAN message
uint8_t getCmdID(uint32_t msg_id);

// blocking function that reads the heartbeat msg data
// used only in the initODrive() function
void waitForHeartbeat(uint8_t axis);

void readHeartbeat(uint8_t axis_id, CAN_message_t msg);

// enters closed-loop control
void initActuators();

// calls the Estop() function on all axes
void stopActuators();

// updates motor commands on all axes
void updateMotorCommands();

#endif