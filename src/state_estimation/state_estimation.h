#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <vector>
#include <queue>
#include <Encoder.h>
#include <Adafruit_BNO08x.h>

#define USE_LEG_FEEDBACK
#define USE_DECELERATION_THRESHOLD
// #define USE_VELOCITY_THRESHOLD

//////////////////////////////////////////////////////////////////////////////////////
// control loop periods
const uint16_t k_dtPosUpdate = 250;                 // position sampling period in microseconds; encoders use hardware interrupt, but the pulse counts need to be transformed to joint space at an interval
const uint16_t k_dtVelUpdate = 2000;                // velocity sampling period in microseconds; numerical difference on position and filtering frequency
const uint16_t k_dtAccelUpdate = 10000;             // acceleration sampling period in microseconds; numerical difference on velocity and filtering frequency
const uint8_t k_dtContactUpdate = 5;                // leg ground contact update period in ms
const uint8_t k_dtKinematicsUpdate = 1;             // robot kinematics update period in ms

const uint8_t k_dtIMUUpdate = 5;                    // IMU sampling period in ms
const uint8_t k_dtPowerUpdate = 10;                 // motor power update period in ms
const uint8_t k_dtMotorTorqueFilterUpdate = 10;     // motor torque filtering period in ms

// time variables
extern elapsedMicros dt_last_pos_update;            // time since last position sampling in microseconds
extern elapsedMicros dt_last_vel_update;            // time since last velocity sampling in microseconds
extern elapsedMicros dt_last_accel_update;          // time since last acceleration sampling in microseconds
extern uint32_t t_last_contact_update;              // timestamp in milliseconds at last leg ground contact update
extern uint32_t t_last_kinematics_update;           // timestamp in milliseconds at last kinematics update

extern uint32_t t_last_IMU_update;                  // timestamp in milliseconds at last IMU sampling
extern uint32_t t_last_power_update;                // timestamp in milliseconds at last power update
extern uint32_t t_last_motor_torque_filter_update;  // timestamp in milliseconds at last motor torque filter update


// joystick
const float kJoystickXDeadZone = 100;
const float kJoystickYDeadZone = 300;
const float kJoystickXCenter = 499;
const float kJoystickYCenter = 494;
const float kJoystickXMax = max(1023 - kJoystickXCenter - kJoystickXDeadZone/2, kJoystickXCenter - kJoystickXDeadZone/2);
const float kJoystickYMax = max(1023 - kJoystickYCenter - kJoystickYDeadZone/2, kJoystickYCenter - kJoystickYDeadZone/2);

// web GUI joystick
const float kGUIJoystickXDeadZone = 0.2;    // x-direction (front-back) deadzone in each direction; [0, 1]
const float kGUIJoystickYDeadZone = 0.45;   // y-direction (left-right) deadzone in each direction; [0, 1]

// ACS711EX currrent sensor
const float k_mVPerAmpere = 45;             // output sensitivity in mV per ampere for ACS711EX current sensor

// motor torque moving average filter
const int kTorqueFilterLength = 3;          // number of torque samples to hold in the FIFO queue

// leg velocity moving average filter
const int kLegVelFilterLength = 3;          // number of leg velocity samples to hold in the FIFO queue

// leg acceleration moving average filter
const int kLegAccelFilterLength = 4;        // number of leg acceleration samples to hold in the FIFO queue

// IMU parameters
const int kIMUAvgSize = 100;                                // IMU pose estimation initial value average sample size
const std::vector<int> kBodyFrameAxisIndex = {2, -1, 3};    // IMU frame to body frame mapping
                                                            // e.g. {2, -1, 3} means
                                                            // body x-axis is positive IMU y-axis
                                                            // body y-axis is negative IMU x-axis
                                                            // body z-axis is positive IMU z-axis

// contact detection
const float kDqStartContactLowImpulse = 8;      // leg displacment in mm past which contact detection begins; this value MUST BE AT LEAST less than the leg retraction amount (see leg_swing_percent)
const float kQdotContactLowImpulse = 3;         // leg touchdown velocity in mm/s below which contact is likely
const float kQddotContact = -1100;              // 1600; leg acceleration in mm/s^2 above which (more negative) leg contact is likely
const float kDqStartContactHighImpulse = 15;    // leg displacment in mm past which contact detection begins; this value MUST BE AT LEAST less than the leg retraction amount (see leg_swing_percent)
const float kQdotPercentAtContact = 0.3;        // percentage of max leg touchdown velocity at which ground contact is assumed
                                                // this method seeks to detect contact sooner than standstill by checking for velocity reduction

//////////////////////////////////////////////////////////////////////////////////////
// sensor and filter structs

struct TwoAxisJoystick{
    uint16_t x_raw = 0;             // joystick x-axis input in counts
    uint16_t y_raw = 0;             // joystick y-axis input in counts
    float x_norm = 0;               // joystick x-axis input normalized after deadzone and max value compensation
    float y_norm = 0;               // joystick y-axis input normalized after deadzone and max value compensation
    uint8_t joystick_select = 0;    // joystick select;

    // read the joystick XY analog voltages and the select button and normalize
    void readJoystick();

    // normalize joystick position based on deadzones and calibrated center
    void normalizeJoystick();

    void checkStopButton();
};

struct BNO08X_IMU {
    enum eulers {
        roll = 0,
        pitch = 1,
        yaw = 2
    };
    
    Adafruit_BNO08x bno08x;                     // I2C IMU
    sh2_SensorValue_t imu_sensor_value;         // BNO8X helper object
    uint8_t quat_accuracy = 0;
    std::vector<float> quat = {0, 0, 0, 0};     // quaternion vector given by the IMU algorithm
    std::vector<float> rpy = {0, 0, 0};         // roll pitch yaw calculated from the quaternion in degrees
    std::vector<float> gyro = {0, 0, 0};        // calibrated gyroscope output values

    // initialize BNO08x IMU and set desired reports
    void initIMU(TwoWire *wire);

    // fetch IMU data updates and raise stop flag is IMU is disconnected
    void readIMU();

    // convert quaternion to roll pitch yaw
    void quat2rpy();

    // return roll pitch yaw estimate transformed to robot body frame
    std::vector<float> getRPY();

    // return angular velcotiy estimates transformed to robot body frame
    std::vector<float> getOmega();
};

struct ACS711EX_Sensor {
    uint8_t ACS711EX_fault = HIGH;      // current exceeded limit (31 A) if this latches low
    uint16_t ACS711EX_Vout = 512;       // Vout centered at Vcc/2
    float current_measured = 0;         // current measured across the sensor

    // update current sensor estimate
    void readCurrentSensor();
};

struct MovingAvgFilter {
    std::queue<float> filter_queue;     // queue of motor torque for moving average filter
    uint16_t size = 0;                  // FIFO queue size
    float queue_sum = 0;                // running sum of each motor's recent torque readings
    float filtered_value = 0;           // estimated motor torque from proprioceptive sensing

    MovingAvgFilter(uint8_t _size);

    void updateFilter(float val);
};

//////////////////////////////////////////////////////////////////////////////////////
// global variables
extern Encoder encoders[];                                  // hardware interrupt encoders
extern TwoAxisJoystick two_axis_joystick;                   // 2-axis joystick with select button
extern BNO08X_IMU bno08x_imu;                               // BNO08x 9-DoF IMU
extern ACS711EX_Sensor acs711ex_sensor;                     // ACS711EX currrent sensor
extern std::vector<MovingAvgFilter> motor_torque_filters;   // moving average filter for motor torques
extern float battery_voltage;                               // current battery voltage in voltage
extern float battery_current;                               // current battery current in ampere
extern float battery_power;                                 // current battery power in watt

extern std::vector<float> q;                                // joint position from encoders
extern std::vector<float> q_prev;                           // q at last velocity update
extern std::vector<float> q_dot;                            // unfiltered joint velocity from numerical difference
extern std::vector<float> q_dot_prev;                       // q_dot at last acceleration update
extern std::vector<float> q_ddot;                           // unfiltered joint acceleration from numerical difference
extern std::vector<MovingAvgFilter> q_dot_filters;          // moving average filter for leg joint velocity
extern std::vector<MovingAvgFilter> q_ddot_filters;         // moving average filter for leg joint acceleration
extern float z_body_local;                                  // height of body above local terrain
extern float dist_traveled;                                 // distance traveled estimated with translational joint displacement

extern std::vector<float> rpy_lateral_0;                    // lateral body roll pitch yaw after homing
extern std::vector<float> rpy_lateral;                      // lateral body roll pitch yaw relative to rpy_lateral_0
extern std::vector<float> omega_lateral;                    // lateral body angular velocity with respect to body frame axes

extern std::vector<int> isInContact;                       // true if the corresponding motor's legs are on the ground; see contact estimation
extern std::vector<int> isDecelerated;                     // true if a leg's deceleration has exceeded a threshold during touchdown; reset after each cycle
extern std::vector<float> q_dot_max;                        // maximum leg velocity reached during leg touchdown; used for contact detection; reset after each cycle

//////////////////////////////////////////////////////////////////////////////////////
// global functions

// determine the initial body pose by averaging the IMU roll pitch yaw readings
void zeroIMUReading();

// update robot states and sensor feedback
void updateStates();

// update robot joint position, velocity, accelerations
void updateJointEstimates();

// fetch IMU reading and update the body orientation relative to the initial orientation
void updateIMUEstimate();

// estimates the contact state of each swing leg motors
void updateContactState();

void resetSwingLegContactState();

// estimate total power consumption by motors
void updatePowerMeasurement();

// call the updateFilter() function for all the moving average filters
void updateMotorTorqueFilters();

// call the updateFilter() function for joint acceleration filters
// update frequency tracks sampling rate
void updateVelFilters();

// call the updateFilter() function for joint acceleration filters
// update frequency tracks sampling rate
void updateAccelFilters();

// update robot body pose based on kinematics
void updateKinematics();

#endif