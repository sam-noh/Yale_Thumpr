#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <vector>
#include "..\actuator\actuator.h"

//////////////////////////////////////////////////////////////////////////////////////
// constants

// ODrive Pro CAN parameters
const int kODriveCANBaudRate = 1000000;             // baud rate for ODrive CAN2.0
const int kAxisIDLength = 6;                        // number of bits of the CAN message's axis ID
const int kCmdIDLength = 5;                         // number of bits of the CAN message's command ID
const uint16_t kODriveCANWatchdogTimeout = 200;     // number of milliseconds to wait for a CAN message until safety timeout

// motor speed constants
const float k_KV_to_KT = 8.27;              // conversion factor from speed constant to torque constant
const float k_KV_D5312S = 330;              // speed constant of the leg motor
const float k_KV_GL60 = 25;                 // speed constant of the trans/yaw motor

// positive motor direction
const int kDirectionLeg = 1;                // positive direction for leg motors
const int kDirectionTrans = -1;             // positive direction for translation motor
const int kDirectionYaw = 1;                // positive direction for yaw motor

// motor homing parameters in joint space
// should be determined heuristically
const float kQdotLegHoming = -90;               // leg motor homing velocity in mm/s
const float kQdotLegHomingStop = -6;            // leg motor homing stop condition velocity in mm/s

const float kQdotTransHoming = 60;              // translation motor homing velocity in mm/s
const float kQdotTransHomingStop = 25;          // translation motor homing stop condition velocity in mm/s
const float kQTransHomingOffset = -120;         // distance from the joint limit to the translation zero position in mm

const float kQdotYawHomingStop = 0.5;           // yaw motor homing stop condition velocity in deg/s

// motor (1) controller velocity limits and (2) trapezoidal trajectory velocity limits
const float kVelLegMax = 80;                    // leg motor controller velocity limit in turns/s
const float kVelLegMaxContact = 25;             // leg motor controller velocity limit during touchdown in turns/s
const float kVelTransMax = 22;                  // translation motor controller velocity limit in turns/s
const float kVelYawMax = 5.8;                   // yaw motor controller velocity limit in turns/s

const float kVelLegTrajSwing = 35;              // leg motor trapezoidal trajectory velocity in swing phase in turns/s
const float kVelLegTrajStandup = 20;            // leg motor trapezoidal trajectory velocity when standing up in turns/s
const float kVelLegTrajTilt = 10;               // leg motor trapezoidal trajectory velocity when tilt correcting in turns/s (tested 10)
const float kVelTransTraj = 12;                 // translation motor trapezoidal trajectory velocity
const float kVelYawTraj = 0.4;                  // yaw motor trapezoidal trajectory velocity

// motor trapezoidal trajectory accel/deceleratioin parameters
const float kAccelLegTrajSwing = 180;           // leg motor trapezoidal trajectory acceleration when retracting legs in turns/s^2
const float kAccelLegTrajStandup = 80;          // leg motor trapezoidal trajectory acceleration when changing height in turns/s^2
const float kAccelLegTrajTilt = 80;             // leg motor trapezoidal trajectory acceleration when tilt correcting in turns/s^2
const float kAccelTransTraj = 15;               // translation motor trapezoidal trajectory acceleration;
const float kAccelYawTraj = 10;                 // yaw motor trapezoidal trajectory acceleration

const float kDecelLegTrajSwing = 100;           // leg motor trapezoidal trajectory deceleration when retracting legs in turns/s^2
const float kDecelLegTrajStandup = 60;          // leg motor trapezoidal trajectory deceleration when changing height in turns/s^2;
const float kDecelLegTrajTilt = 30;             // leg motor trapezoidal trajectory deceleration when tilt correcting in turns/s^2;
const float kDecelTransTraj = 15;               // translation motor trapezoidal trajectory acceleration;
const float kDecelYawTraj = 5;                  // yaw motor trapezoidal trajectory acceleration

// motor current limits
const float kCurrentLegMax = 60;                // leg motor current limit in ampere
const float kCurrentLegMaxHoming = 10;          // leg motor current limit while homing in ampere
const float kCurrentTransMax = 8;               // translation motor current limit in ampere
const float kCurrentYawMax = 5;                 // yaw motor current limit in ampere
const float kCurrentTransMaxHoming = 6;         // translation motor current limit while homing in ampere

// motor temperature limits
const float kTemperatureLegMotorMax = 60;       // leg motor temperature limit in celsius

// motor position control parameters
const float kQErrorMax = 3;                     // maximum allowable position error in mm

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

// updates motor controller limits on all axes
void updateMotorLimits();

// updates motor commands on all axes
void updateMotorCommands();

#endif