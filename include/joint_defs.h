#ifndef JOINT_DEFS_H
#define JOINT_DEFS_H

#include "../src/mcu_util/mcu_util.h"

const int kNumOfActuators =  6;             // number of actuators
const int kNumOfLegs = 8;                   // number of prismatic legs
const int kNumOfJoints = 10;                // number of joints

enum JointID {
    kJointMedialFrontRight = 0,  // medial body front right leg
    kJointMedialFrontLeft = 1,
    kJointMedialRearRight = 2,
    kJointMedialRearLeft = 3,
    kJointLateralFrontRight = 4,
    kJointLateralRearRight = 5,
    kJointLateralFrontLeft = 6,
    kJointLateralRearLeft = 7,
    kJointTranslate = 8,
    kJointYaw = 9
};

// transmission parameters
const float kNumOfTeethDiffInput = 60.0;        // number of teeth of differential input gear
const float kNumOfTeethDiffOutput = 30.0;       // number of teeth of differential output gear
const float kWormGearRatio = 10.0;              // gear ratio of the worm drive
const float kPitchDiaLegDrive = 19.101;         // pitch diameter in mm of the leg timing belt pulley
const float kPitchDiaTrans = 20.0;              // pitch diameter in mm of the translation spur gear
const float kPitchDiaYawInput = 15;             // pitch diameter in mm of the yaw input gear
const float kPitchDiaYawOutput = 80;            // pitch diameter in mm of the yaw output gear

// transmission ratio: joint position in mm or deg = T*motor revolutions
const float kTxRatioLeg = PI*kPitchDiaLegDrive/(kWormGearRatio*kNumOfTeethDiffOutput/kNumOfTeethDiffInput);
const float kTxRatioLegDrive = PI*kPitchDiaLegDrive;
const float kTxRatioTrans = PI*kPitchDiaTrans;
const float kTxRatioYaw = 360*kPitchDiaYawInput/kPitchDiaYawOutput;

// quadrature encoder counts per revolution
const int k_CPR_AMT102_V = 8192;            // counts per revolution for the worm gearbox output encoder
const int kDirectionWorm[] = {-1, 1, 1, -1, 1, -1, -1, 1};

// kinematic parameters
const float kL_1 = 728;                     // perpendicular distance between opposite legs on the medial body
const float kW_1 = 468;                     // perpendicular distance between opposite legs on the lateral body
const float kW_2 = 350;                     // perpendicular distance between the front and rear legs on the lateral body
const float kBodyZOffset = 0;               // vertical distance between medial body's and lateral body's shoulders
const float stance_width[2] = {kW_1, kL_1}; // perpendiculuar distance between opposite legs in stance; used to calculate the required joint displacement for tilt control
const float stance_length[2] = {kL_1, kW_2};

// mechanical joint limits and offsets
const float kQLegMin = 0;               // min leg joint position in mm
// const float kQLegMedMax = 760;          // max medial leg joint position in mm
const float kQLegMedMax = 515;          // max medial leg joint position in mm
const float kQLegLatMax = 515;          // max lateral leg joint position in mm
const float kQLegUnstuck = 10;          // leg joint position limit in mm used for checking stuck legs
const float kQLegNearLimit = 100;       // leg joint position limit in mm used for applying torque limit
const float kLegOffset = 0;             // vertical distance between medial and lateral body in mm; used to adjust leg setpoints
const float kQTransMax = 95;            // max/min translation position from zero position in mm
const float kQTransSoftMax = 80;        // max/min translation position from zero position in mm for the controller
const float kQYawMax = 12;              // max/min yaw position from zero position in degrees
const float kDqTransEndYawLimit = 20;   // distance from translational joint limit below which yaw command is reduced to avoid self-collision
const float kDqYawLimit = 3;            // max yaw angle reduction when the translational joint is near the limit

#endif