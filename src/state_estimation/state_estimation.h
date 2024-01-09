#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <vector>
#include <queue>
#include <Encoder.h>
#include <Adafruit_BNO08x.h>

//////////////////////////////////////////////////////////////////////////////////////
// constants

// joystick
const float kJoystickXDeadZone = 100;
const float kJoystickYDeadZone = 300;
const float kJoystickXCenter = 499;
const float kJoystickYCenter = 494;
const float kJoystickXMax = max(1023 - kJoystickXCenter - kJoystickXDeadZone/2, kJoystickXCenter - kJoystickXDeadZone/2);
const float kJoystickYMax = max(1023 - kJoystickYCenter - kJoystickYDeadZone/2, kJoystickYCenter - kJoystickYDeadZone/2);

// ACS711EX currrent sensor
const float k_mVPerAmpere = 45;                   // output sensitivity in mV per ampere for ACS711EX current sensor

// motor torque moving average filter
const int kTorqueFilterLength = 5;                // number of torque samples to hold in the FIFO queue

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
    std::vector<float> gyro = {0, 0, 0};        // calibrated gyroscope output values

    // initialize BNO08x IMU and set desired reports
    void initIMU(TwoWire *wire);

    // fetch IMU data updates and raise stop flag is IMU is disconnected
    void readIMU();

    // convert quaternion to roll pitch yaw
    void quat2rpy();
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
    float queue_sum = 0;                // running sum of each motor's recent torque readings
    float filtered_value = 0;           // estimated motor torque from proprioceptive sensing

    MovingAvgFilter(uint8_t size);

    void updateFilter(float val);
};

//////////////////////////////////////////////////////////////////////////////////////
// global variables
extern std::vector<Encoder> encoders;                       // hardware interrupt encoders
extern TwoAxisJoystick two_axis_joystick;                   // 2-axis joystick with select button
extern BNO08X_IMU bno08x_imu;                               // BNO08x 9-DoF IMU
extern ACS711EX_Sensor acs711ex_sensor;                     // ACS711EX currrent sensor
extern std::vector<MovingAvgFilter> motor_torque_filters;   // moving average filter for motor torques

extern std::vector<float> q;                    // see JointID for joint indices
extern std::vector<float> q_dot;                // see JointID for joint indices
extern float z_body_local;                      // height of body above local terrain
extern std::vector<float> rpy_medial;           // medial body orientation estimated by IMU

//////////////////////////////////////////////////////////////////////////////////////
// global functions

// update robot states and sensor feedback
void updateStates();

// update robot joint positions and velocities from encoder feedback
void readJointEncoders();

// call the updateFilter() function for all the moving average filters
void updateMotorTorqueFilters();

// update robot body pose based on kinematics
void updateKinematics();

#endif