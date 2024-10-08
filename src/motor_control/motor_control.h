#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <vector>
#include "../actuator/actuator.h"

//////////////////////////////////////////////////////////////////////////////////////
// constants

// enum for motor index
enum MotorID {
    kMotorMedialFront = 0,
    kMotorMedialRear = 1,
    kMotorLateralRight = 2,
    kMotorLateralLeft = 3,
    kMotorTranslate = 4,
    kMotorYaw = 5
};

// battery parameters
const float kMinBatteryVoltage = 22.3;              // min battery voltage

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
const float kQdotLegHomingStop = -10;           // leg motor homing stop condition velocity in mm/s
const float kDqLegLift = 40;                    // stance body lift in mm to home the locomotion mechanism

const float kQdotTransHoming = 200;             // translation motor homing velocity in mm/s
const float kQdotTransHomingStop = 25;          // translation motor homing stop condition velocity in mm/s
const float kQTransHomingOffset = -120;         // distance from the joint limit to the translation zero position in mm

const float kQdotYawHomingStop = 8;             // yaw motor homing stop condition velocity in deg/s

// motor (1) controller velocity limits and (2) trapezoidal trajectory velocity limits
const float kVelLegMax = 100;                   // leg motor controller velocity limit in turns/s
const float kVelLegMaxContact = 25;             // leg motor controller velocity limit during touchdown in turns/s
const float kVelLegFootSlip = 80;               // leg motor controller velocity limit during unexpected foot slip in turns/s
const float kVelTransMax = 22;                  // translation motor controller velocity limit in turns/s
const float kVelYawMax = 5.8;                   // yaw motor controller velocity limit in turns/s

const float kVelLegTrajSwing = 40;              // 45; leg motor trapezoidal trajectory velocity in swing phase in turns/s; 35
const float kVelLegTrajStandup = 25;            // leg motor trapezoidal trajectory velocity when standing up in turns/s
const float kVelLegTrajSlow = 6;               // leg motor trapezoidal trajectory velocity when tilt correcting in turns/s
const float kVelTransTraj = 16;                 // translation motor trapezoidal trajectory velocity
const float kVelTransTrajSlow = 8;              // translation motor trapezoidal trajectory velocity when the body height is high
const float kVelYawTraj = 0.4;                  // yaw motor trapezoidal trajectory velocity

// motor trapezoidal trajectory accel/deceleratioin parameters
const float kAccelLegTrajSwing = 250;           // 300; leg motor trapezoidal trajectory acceleration when retracting legs in turns/s^2
const float kAccelLegTrajStandup = 100;         // 80; leg motor trapezoidal trajectory acceleration when changing height in turns/s^2
const float kAccelTransTraj = 20;               // 25; translation motor trapezoidal trajectory acceleration;
const float kAccelTransTrajSlow = 10;           // translation motor trapezoidal trajectory acceleration when the body height is high
const float kAccelYawTraj = 10;                 // yaw motor trapezoidal trajectory acceleration

const float kDecelLegTrajSwing = 150;           // 250; leg motor trapezoidal trajectory deceleration when retracting legs in turns/s^2
const float kDecelLegTrajStandup = 80;          // 60; leg motor trapezoidal trajectory deceleration when changing height in turns/s^2;
const float kDecelLegTrajTilt = 30;             // leg motor trapezoidal trajectory deceleration when tilt correcting in turns/s^2;
const float kDecelTransTraj = 15;               // 18; translation motor trapezoidal trajectory acceleration;
const float kDecelTransTrajSlow = 10;           // translation motor trapezoidal trajectory deceleration when the body height is high
const float kDecelYawTraj = 5;                  // yaw motor trapezoidal trajectory acceleration

// motor current limits
const float kCurrentLegMax = 60;                // leg motor current limit in ampere
const float kCurrentLegMaxHoming = 13;          // leg motor current limit while homing in ampere
const float kCurrentTransMax = 8;               // translation motor current limit in ampere
const float kCurrentYawMax = 5;                 // yaw motor current limit in ampere
const float kCurrentTransMaxHoming = 6;         // translation motor current limit while homing in ampere

// motor torque limits
const float kTorqueLegMinJointLimit = -0.3;    // leg motor torque limit in Nm when retracting with at least one leg near the joint limit

// motor temperature limits
const float kTemperatureLegMotorMax = 60;       // leg motor temperature limit in celsius

// motor position control parameters
const float kQErrorMax = 3;                     // maximum allowable position error in mm
const float kDqJointLimit = 10;                 // offset from either side of the leg joint limit in mm when the position setpoint is outside the range of motion

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
uint8_t getNodeID(uint32_t msg_id);

// extracts and returns the command ID from the ODrive's CAN message
uint8_t getCmdID(uint32_t msg_id);

// blocking function that reads the heartbeat msg data
// used only in the initODrive() function
void waitForHeartbeat(uint8_t axis);

void readHeartbeat(uint8_t node_id, CAN_message_t msg);

// enters closed-loop control
void initActuators();

// enters idle state on all axes
void stopActuators();

void updateMotorTorque(uint8_t idx_motor, float torque, float vel_limit = 0);

// updates motor controller limits on all axes
void updateMotorLimits();

// updates motor commands on all axes
void updateMotorCommands();

#endif