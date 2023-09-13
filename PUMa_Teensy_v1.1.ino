#include <math.h>
#include <TimeLib.h>
#include <Encoder.h>
#include <SD.h>
#include <DS1307RTC.h>
#include <Adafruit_BNO08x.h>
#include <Actuator.h>
#include <cppQueue.h>

////////////////////////////////////////////////////////////////////////////////////////////////
// macros
#define PI 3.1415926535897932384626433832795
#define TIME_HEADER  "T"            // Header tag for serial time sync message
#define IMPLEMENTATION FIFO_IMPL    // queue implementation
#define overwrite true              // queue overwrite
typedef struct strRec {             // data structure of each element in a queue
  float val;
} Rec;

// hardware pins and communication parameters
#define ENC_1_A 33  // medial front right A
#define ENC_1_B 34  // medial front right B
#define ENC_2_A 14  // medial front left A
#define ENC_2_B 15  // medial front left B
#define ENC_3_A 11  // medial rear right A
#define ENC_3_B 12  // medial rear right B
#define ENC_4_A 9   // medial rear left A
#define ENC_4_B 10  // medial rear left B
#define ENC_5_A 35  // lateral front right A
#define ENC_5_B 36  // lateral front right B
#define ENC_6_A 24  // lateral rear right A
#define ENC_6_B 25  // lateral rear right B
#define ENC_7_A 37  // lateral front left A
#define ENC_7_B 38  // lateral front left B
#define ENC_8_A 7   // lateral rear left A
#define ENC_8_B 8   // lateral rear left B
#define ACS711EX_OUT 27     // Vout pin for ACS711EX current sensor
#define ACS711EX_FAULT 28   // fault pin for ACS711EX current sensor
#define BNO08X_RESET -1     // reset pin for BNO08X IMU; not needed for I2C
#define XOUT 39             // analog input pin for joystick x-axis
#define YOUT 40             // analog input pin for joystick y-axis
#define SEL 41              // digital pin for joystick button
#define LED 13              // built-in LED pin

#define SERIAL_USB Serial             // Serial (USB), Serial<x> (Hardware serial) e.g. Serial5
#define WIRE_IMU Wire                 // Using I2C bus 1: SCL, SDA
#define BAUD_RATE_ODRIVE_CAN 1000000  // baud rate for ODrive CAN2.0
#define MAX_DATA_SIZE 128             // max number of chars in sent/received data array
#define AXIS_ID_LENGTH 6              // number of bits of the CAN message's axis ID
#define CMD_ID_LENGTH 5               // number of bits of the CAN message's command ID
#define MV_PER_A 45                   // output sensitivity in mV per ampere for ACS711EX current sensor

#define DT_CLOSED_LOOP 50     // wait time in ms between setup function calls
#define DT_FETCH_UPDATE 100   // duration in ms to handle CAN messages and to update states in between function calls in setup
#define DT_ACTUATION 10       // motor command update period in ms
#define DT_STATE 5            // motor state update period in ms
#define DT_SETPOINT 10        // joint setpoint update period in ms
#define DT_PRINT 50           // data print period in ms

// joystick parameters
#define JOYSTICK_X_DEADZONE 100.0
#define JOYSTICK_Y_DEADZONE 300.0
#define JOYSTICK_X_CENTER 499.0
#define JOYSTICK_Y_CENTER 494.0
#define JOYSTICK_X_MAX max(1023 - JOYSTICK_X_CENTER - JOYSTICK_X_DEADZONE/2, JOYSTICK_X_CENTER - JOYSTICK_X_DEADZONE/2)
#define JOYSTICK_Y_MAX max(1023 - JOYSTICK_Y_CENTER - JOYSTICK_Y_DEADZONE/2, JOYSTICK_Y_CENTER - JOYSTICK_Y_DEADZONE/2)

// actuation parameters
#define N_ACTUATOR 6              // number of actuators
#define N_LEGS 8                  // number of prismatic legs
#define N_JOINTS 10               // number of joints
#define N_GAIT_PHASES 2           // number of gait phases in a gait cycle
#define N_ACTUATION_PHASES 3      // number of actuation phases in swing

#define DIR_LEG -1                // positive direction for leg motors
#define DIR_TRANS 1               // positive direction for translation motor
#define DIR_YAW 1                 // positive direction for yaw motor
#define CPR_AMT102_V 8192         // number of quadrature counts per revolution
#define KV_D5312S 330             // speed constant of the leg motor
#define KV_GL60 25                // speed constant of the trans/yaw motor

#define VEL_LEG_HOMING -50.           // leg motor homing velocity in mm/s
#define VEL_LEG_HOMING_STOP -10.      // leg motor homing stop condition velocity in mm/s
#define VEL_TRANS_HOMING 80.          // translation motor homing velocity in mm/s
#define VEL_TRANS_HOMING_STOP 5.      // translation motor homing stop condition velocity in mm/s
#define POS_TRANS_HOMING_OFFSET -110. // distance from the joint limit to the translation zero position in mm
#define POS_YAW_HOME 47.7             // UPDATE VALUE AFTER YAW MOTOR REASSEMBLY; yaw home position in degrees
#define VEL_YAW_HOMING_STOP 0.2       // yaw motor homing stop condition velocity in deg/s

#define VEL_LEG_LIMIT_NOMINAL 25                      // leg motor velocity limit in turns/s
#define VEL_LEG_LIMIT_CONTACT 15                      // leg motor velocity limit during touchdown in turns/s
#define VEL_LEG_TRAJ_STANDUP 15                       // leg motor trapezoidal trajectory velocity when standing up in turns/s
#define VEL_LEG_TRAJ_NOMINAL VEL_LEG_LIMIT_NOMINAL    // leg motor trapezoidal trajectory velocity when retracting legs in turns/s
#define VEL_LEG_TRAJ_SLOW 6                           // leg motor trapezoidal trajectory velocity when tilt correcting in turns/s (tested 10)
#define VEL_LEG_TRAJ_RETRACT 2                        // leg motor trapezoidal trajectory velocity when stance switching in turns/s
#define VEL_TRANS_LIMIT_NOMINAL 5                     // translation motor velocity limit in turns/s

#define DQ_LEG_RETRACT 2                              // leg displacement during stance switching in mm

#define ACCEL_LEG_TRAJ_NOMINAL 100                    // leg motor trapezoidal trajectory acceleration when retracting legs in turns/s^2
#define DECEL_LEG_TRAJ_NOMINAL 80                     // leg motor trapezoidal trajectory deceleration when retracting legs in turns/s^2
#define DECEL_LEG_TRAJ_SLOW 10                        // leg motor trapezoidal trajectory deceleration when tilt correcting in turns/s^2

#define CURRENT_LEG_LIMIT 40                          // leg motor current limit in ampere
#define CURRENT_LEG_LIMIT_HOMING 12                   // leg motor current limit while homing in ampere
#define CURRENT_TRANS_LIMIT 4                         // translation motor current limit in ampere
#define CURRENT_TRANS_LIMIT_HOMING 0.75               // translation motor current limit while homing in ampere

#define Q_LEG_MIN 0               // min leg joint position in mm
#define Q_LEG_MAX 420             // max leg joint position in mm
#define Q_LEG_OFFSET 71           // vertical distance between upper and lower body in mm; added to upper leg setpoints
#define Q_TRANS_MAX 90            // max/min translation position from zero position in mm
#define Q_YAW_MAX 15              // max/min yaw position from zero position in degrees
#define DQ_MAX 100                // max total leg displacements per tilt correction
#define Q_ERROR_MAX 8.            // maximum allowable position error in mm

#define DQ_START_CONTACT 20.      // leg displacmeent in mm past which contact detection begins
#define Q_DOT_CONTACT 0.5         // joint velocity in mm/s below which contact is likely
#define N_TAU_SAMPLE 5            // number of torque samples to hold in the FIFO queue
#define DQ_UNEVEN_TERRAIN 20.     // leg pair stroke difference in mm above which the terrain is assumed to be uneven
#define DH_MAX 40.                // maximum allowed body height deviation in mm
#define TILT_NOMINAL 3.           // acceptable body tilt from zero in degrees
#define TILT_MAX 7.               // maximum allowed body tilt from zero in degrees
#define GYRO_STABLE 0.04          // calibrated gyro output value below which the robot body is considered to be not oscillating due to dynamics

// transmission parameters
#define N_DIFF_RING 34.0        // number of teeth on differential ring gear
#define N_DIFF_PINION 11.0      // number of teeth on the drive hub
#define N_WORM_RATIO 10.0       // gear ratio of the worm drive
#define D_LEG_SPUR 40.0         // pitch diameter in mm of the leg spur gear
#define D_TRANS_SPUR 24.0       // pitch diameter in mm of the translation spur gear
#define D_YAW_SPUR_INPUT 20.0   // pitch diameter in mm of the yaw input gear
#define D_YAW_SPUR_OUTPUT 120.0 // pitch diameter in mm of the yaw output gear

// transmission ratio: joint position in mm or deg = T*motor revolutions
#define T_PINION PI*D_LEG_SPUR
#define T_LEG T_PINION/N_WORM_RATIO
#define J_LEG N_WORM_RATIO/(0.5*D_LEG_SPUR/1000)
#define T_TRANS PI*D_TRANS_SPUR
#define T_YAW 360*D_YAW_SPUR_INPUT/D_YAW_SPUR_OUTPUT

// kinematic parameters
#define L 736.4                 // perpendicular distance between opposite legs on the medial body
#define W 433.4                 // perpendicular distance between opposite legs on the lateral body
#define Z 70.                   // vertical distance between medial and lateral bodies

////////////////////////////////////////////////////////////////////////////////////////////////
// ODrive and actuator variables
ODriveTeensyCAN ODrive_CAN(BAUD_RATE_ODRIVE_CAN);
CAN_message_t msg;
uint8_t axis_id;
uint8_t cmd_id;

// variables for reading the CAN message
struct HeartbeatMsg_t heartbeat_msg;
struct EncoderEstimatesMsg_t encoder_msg;
struct IqMsg_t Iq_msg;
uint32_t active_error;
uint32_t ctrlr_error;

// Actuator instances that hold motor state variables
Actuator motors[N_ACTUATOR] = {Actuator(ODrive_CAN, SERIAL_USB, 0, DIR_LEG, T_LEG, 8.27 / KV_D5312S, Q_LEG_MIN, Q_LEG_MAX),
                               Actuator(ODrive_CAN, SERIAL_USB, 1, DIR_LEG, T_LEG, 8.27 / KV_D5312S, Q_LEG_MIN, Q_LEG_MAX),
                               Actuator(ODrive_CAN, SERIAL_USB, 2, DIR_LEG, T_LEG, 8.27 / KV_D5312S, Q_LEG_MIN, Q_LEG_MAX),
                               Actuator(ODrive_CAN, SERIAL_USB, 3, DIR_LEG, T_LEG, 8.27 / KV_D5312S, Q_LEG_MIN, Q_LEG_MAX),
                               Actuator(ODrive_CAN, SERIAL_USB, 4, DIR_TRANS, T_TRANS, 8.27 / KV_GL60, -Q_TRANS_MAX, Q_TRANS_MAX),
                               Actuator(ODrive_CAN, SERIAL_USB, 5, DIR_YAW, T_YAW, 8.27 / KV_GL60, -Q_YAW_MAX, Q_YAW_MAX)
                              };

// hardware interrupt encoder objects
Encoder encoders[] = {Encoder(ENC_1_A, ENC_1_B),  // medial body; front right leg
                      Encoder(ENC_2_A, ENC_2_B),  // medial body; front left leg
                      Encoder(ENC_3_A, ENC_3_B),  // medial body; rear right leg
                      Encoder(ENC_4_A, ENC_4_B),  // medial body; rear left leg
                      Encoder(ENC_5_A, ENC_5_B),  // lateral body; front right leg
                      Encoder(ENC_6_A, ENC_6_B),  // lateral body; rear right leg
                      Encoder(ENC_7_A, ENC_7_B),  // lateral body; front left leg
                      Encoder(ENC_8_A, ENC_8_B)
                     }; // lateral body; rear left leg

// IMU variables
Adafruit_BNO08x  bno08x(BNO08X_RESET);            // I2C IMU
sh2_SensorValue_t sensorValue;                    // BNO8X helper object
uint8_t quat_accuracy = 0;
float quat[4] = {0, 0, 0, 0};   // quaternion vector given by the IMU algorithm
float gyro[3] = {0, 0, 0};      // calibrated gyroscope output values

// ACS711EX variables
uint8_t ACS711EX_fault = HIGH;  // current exceeded limit (31 A) if this latches low
uint16_t ACS711EX_Vout = 512;   // Vout centered at Vcc/2

// communication variables
char receivedData[MAX_DATA_SIZE] = "";
char sentData[MAX_DATA_SIZE] = "";
const int chipSelect = BUILTIN_SDCARD;
File dataFile;  // SD card file
char dataFileName[128] = "";

// time variables
unsigned long curT;                 // current timestamp
unsigned long prevTPrint = 0;       // timestamp at last serial transmission
unsigned long prevTState = 0;       // timestamp at last state update
unsigned long prevTSetpoint = 0;    // timestamp at last setpoint update
unsigned long prevTActuation = 0;   // timestamp at last motor command update
unsigned long prevTEncoder = 0;     // timestamp at last encoder reading

// joystick variables
uint16_t joystick_x = 0;        // joystick x-axis input in counts
uint16_t joystick_y = 0;        // joystick y-axis input in counts
float joystick_x_norm = 0;      // joystick x-axis input normalized after deadzone and max value compensation
float joystick_y_norm = 0;      // joystick y-axis input normalized after deadzone and max value compensation
uint8_t joystick_select = 0;    // joystick select;

// gait variables
float cmd_vector[2] = {1, 0};   // heading vector; if the vector is zero, the robot will stop at the next stance phase; to be extended to 3D
// follows IMU "coordinate frame"; x-positive: forward, y-positive: left, z-positive: up
// currently take values from the normalized joystick inputs after deadzone and max value compensation (0 to 1)
float yaw_cmd = 0;              // calculated as the angle of the joystick command
uint8_t gaitPhase = 1;          // current gait phase; (0 upper swing/lower stance) -> (1 upper stance/lower swing); double support is omitted
uint8_t actuationPhase = 0;     // current actuation phase of the swing legs; 0 retract -> 1 translate -> 2 touchdown
uint32_t gaitCycles = 0;        // number of completed gait cycles
bool inContact[N_LEGS / 2] = {false, false, false, false}; // true if the motor's current exceeds the threshold during touchdown; stays true until legs lift
bool isBlocking = false;        // true if any motion primitive outside of the standard gait cycle is in progress
bool corrected = false;         // true if a motion primitive has been completed during the gait cycle

// nominal leg trajectory parameters; can be updated by a high-level planner
// exact trajectory is determined by the motor controller's trapezoidal trajectory generation: acceleration, deceleration, max velocity
float q_leg_stance = 250;         // nominal (lower) leg stroke in stance (tested 250)
float q_leg_swing = 10;           // nominal (lower) leg stroke at max clearance in swing (tested 35 and 70)
float q_trans = Q_TRANS_MAX - 15; // nominal translation position in swing
float q_yaw = Q_YAW_MAX - 5;      // nominal yaw rotation per step

float dq_leg_trans = 5;              // remaining swing leg retraction at which forward translation begins (tested 50)
float q_trans_step = 50;              // translation position at which swing leg step down begins; negative value is before the midpoint regardless of gaitPhase
float h_nominal = q_leg_stance;       // nominal body height over local terrain; currently taken as avg of stance leg motors joint position

// motor setpoints that parameterize leg motion
// 1. leg stroke at max clearance in swing; can change to speed up locomotion or clear taller obstacles
// 2. translation position in swing; can change each step based on higher level planning
// 3. leg stroke in stance; can change each step based on proprioceptive sensing (terrain slope change)
float gaitSetpoints[2][3] = {
  {q_leg_swing + Q_LEG_OFFSET, -q_trans, q_leg_stance + Q_LEG_OFFSET},
  {q_leg_swing, q_trans, q_leg_stance}
};

// motor torque setpoints during leg touchdown
// values are determined heuristically
// for each motor, the first torque command is the minimum necessary to initiate motion
// the second torque command is the minimum necessary to maintain motion
// upon transition to touchdown, the first command is sent
// after the motor has moved by DQ_START_CONTACT, the second command is sent
float touchdownTorque[4][2] = {
  {0.09, 0.045},    // 0.07, 0.05
  {0.07, 0.06},     // 0.07, 0.055
  {0.06, 0.045},    // 0.06, 0.04
  {0.08, 0.06},     // 0.065, 0.055
};

float q_last_contacts[4] = {0, 0, 0, 0};

float stanceWidth[2] = {W, L};  // perpendiculuar distance between opposite legs in stance; used to calculate the required joint displacement for tilt control

// state variables
float q[N_JOINTS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // {medial FR, FL, RR, RL, lateral FR, RR, FL, RL, translation, yaw}
float q_dot[N_JOINTS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // {medial FR, FL, RR, RL, lateral FR, RR, FL, RL, translation, yaw}
float p[3] = {0, 0, 0};     // medial body centroid position from kinematics
float h = 0;                // height of body above local terrain
float rpy[3] = {0, 0, 0};   // medial body orientation estimated by IMU
cppQueue tauQueue[N_ACTUATOR] = {  // FIFO queue of motor torque
  cppQueue(sizeof(Rec), N_TAU_SAMPLE, IMPLEMENTATION, overwrite),
  cppQueue(sizeof(Rec), N_TAU_SAMPLE, IMPLEMENTATION, overwrite),
  cppQueue(sizeof(Rec), N_TAU_SAMPLE, IMPLEMENTATION, overwrite),
  cppQueue(sizeof(Rec), N_TAU_SAMPLE, IMPLEMENTATION, overwrite),
  cppQueue(sizeof(Rec), N_TAU_SAMPLE, IMPLEMENTATION, overwrite),
  cppQueue(sizeof(Rec), N_TAU_SAMPLE, IMPLEMENTATION, overwrite)
};
float tau_hat[N_ACTUATOR] = {0, 0, 0, 0, 0, 0}; // estimated motor torque from proprioceptive sensing
float batteryCurrent = 0;       // battery output current
bool stopSignal = false;

////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  initTeensy();
  initODrives();    // check motor/encoder calibration
  initIMU(&WIRE_IMU);
  initActuators();
  digitalWrite(LED, LOW);

  homeMotors();     // actuator position after homing is assumed to be the new zero position
  initSDCard();
  standUp();
}

void loop() {
  while (!stopSignal) {
    checkStopButton();
    handleODriveCANMsg();
    updateCmdVector();
    updateStates();
    updateGait();
    updateMotorCommands();
    transmitData();
  }

//  stopODrives();    // only use if the legs are actually non-backdrviable; if not, the robot will drop when the motors turn off
  dataFile.close();
  delay(10000000);
}

void updateGait() {
  curT = millis();
  if (curT - prevTSetpoint >= DT_SETPOINT) {
    prevTSetpoint = curT;

    // calculate stability metrics here
    regulateBodyPose(); // if this function executes motions, updateSetpoint() is bypassed during the process
    updateSetpoints();  // update setpoints for the motors in the swing phase
  }
}

// currently takes joystick commands for heading
void updateCmdVector() {
  readJoystick();
  cmd_vector[0] = joystick_y_norm;
  cmd_vector[1] = 0.1 * joystick_x_norm; // scaled down for finer heading control
  yaw_cmd = pow(-1, gaitPhase) * atan2(-cmd_vector[1], cmd_vector[0]) * 180 / PI;     // check yaw direction

  if (yaw_cmd > 90) {
    yaw_cmd -= 180;
  } else if (yaw_cmd < -90) {
    yaw_cmd += 180;
  }
}

// % fix this so that it only sets the setpoint once per cycle
// % isBlocking doesn't prevent the if statements from being run repeatedly
void regulateBodyPose() {
  // nominal body height control
  if (!corrected && actuationPhase == 2 && fabs(h - h_nominal) > DH_MAX && fabs(rpy[0]) < TILT_NOMINAL && fabs(rpy[1]) < TILT_NOMINAL) { // if swing legs have made contact and body height is outside allowed range
    updateContactState();
    if (inContact[gaitPhase * 2] && inContact[gaitPhase * 2 + 1]) {
      float dh = h - h_nominal;

      // adjust velocity, acceleration, deceleration limits for trapezoidal trajectory during this maneuver
      for (uint8_t i = 0; i < N_LEGS/2; i++) {
        ODrive_CAN.SetTrajVelLimit(i, VEL_LEG_TRAJ_SLOW);
        ODrive_CAN.SetTrajAccelLimits(i, ACCEL_LEG_TRAJ_NOMINAL, DECEL_LEG_TRAJ_SLOW);
      }

      // move all leg motors by dh
      for (uint8_t i = 0; i < N_LEGS/2; i++) {
        motors[i].states.q_d = motors[i].states.q - dh;
      }
      isBlocking = true;
      corrected = true;
    }

    // nominal body tilt control
  } else if (!corrected && fabs(rpy[gaitPhase]) > TILT_NOMINAL && actuationPhase == 1 && fabs(motors[4].states.q) < 1 && fabs(motors[4].states.q_dot) < 0.5) {  // if in translation and body is tilted
    uint8_t stance = (gaitPhase + 1) % N_GAIT_PHASES;
    float dq = stanceWidth[gaitPhase] * tan(rpy[gaitPhase] * PI / 180);
    if (dq > DQ_MAX) {
      dq = DQ_MAX;
    }
    if (dq < -DQ_MAX) {
      dq = -DQ_MAX;
    }
    
    motors[stance * 2].states.q_d = motors[stance * 2].states.q_d - dq / 2;
    motors[stance * 2 + 1].states.q_d = motors[stance * 2 + 1].states.q_d + dq / 2;
    ODrive_CAN.SetTrajVelLimit(stance * 2, VEL_LEG_TRAJ_SLOW);
    ODrive_CAN.SetTrajVelLimit(stance * 2 + 1, VEL_LEG_TRAJ_SLOW);
    ODrive_CAN.SetTrajAccelLimits(stance * 2, ACCEL_LEG_TRAJ_NOMINAL, DECEL_LEG_TRAJ_SLOW);
    ODrive_CAN.SetTrajAccelLimits(stance * 2 + 1, ACCEL_LEG_TRAJ_NOMINAL, DECEL_LEG_TRAJ_SLOW);
    isBlocking = true;
    corrected = true;

  } else if (isBlocking && fabs(gyro[0]) < GYRO_STABLE && fabs(gyro[1]) < GYRO_STABLE &&
             fabs(motors[0].states.q_dot) < Q_DOT_CONTACT && fabs(motors[1].states.q_dot) < Q_DOT_CONTACT && fabs(motors[2].states.q_dot) < Q_DOT_CONTACT && fabs(motors[3].states.q_dot) < Q_DOT_CONTACT) {
    isBlocking = false;
    for(uint8_t i = 0; i < 4; i++) {  // update the leg stroke at contact
      q_last_contacts[i] = motors[i].states.q;
    }
  }
}

void updateKinematics() {
  uint8_t stance = (gaitPhase + 1) % N_GAIT_PHASES;
  h = (motors[stance * 2].states.q + motors[stance * 2 + 1].states.q) / 2 - gaitPhase * Z;
}

// estimates the contact state of each swing leg motors
void updateContactState() {
  for (uint8_t i = 0; i < 2; i++) {     // for each swing leg motor
    inContact[gaitPhase * 2 + i] = motors[gaitPhase * 2 + i].states.q - gaitSetpoints[gaitPhase][0] > DQ_START_CONTACT && // if the leg stroke is past the initial displacement
                                   fabs(motors[gaitPhase * 2 + i].states.q_dot) < Q_DOT_CONTACT;                        // AND the motor "joint velocity" is below a threshold
  }
}

// returns true if ready for transition to the next actuation phase
bool isReadyForTransition(uint8_t phase) {
  if (phase == 0) { // if currently lifting leg
    float q_trans_1 = motors[gaitPhase*2].states.q_d + dq_leg_trans;      // leg stroke above which translation can begin
    float q_trans_2 = motors[gaitPhase*2 + 1].states.q_d + dq_leg_trans;
    return motors[gaitPhase*2].states.q < q_trans_1 && motors[gaitPhase*2 + 1].states.q < q_trans_2;
//    return q[gaitPhase * 4] < q_transition && q[gaitPhase * 4 + 1] < q_transition && q[gaitPhase * 4 + 2] < q_transition && q[gaitPhase * 4 + 3] < q_transition;

  } else if (phase == 1) {  // if currently translating forward
    return fabs(cmd_vector[0]) > 0.01 && pow(-1, gaitPhase + 1) * q[8] > q_trans_step; // check if there is a forward command and the translation mechanism has reached the transition point

  } else if (phase == 2) {  // if currently touching down
    updateContactState();   // check the contact state of the two swing legs
    return inContact[gaitPhase * 2] && inContact[gaitPhase * 2 + 1]; // true if both swing legs have made contact

  } else {
    return false;
  }
}

// currently implemented for position control locomotion
void updateSetpoints() {
  // update setpoints at gait phase transitions here
  if (!isBlocking && isReadyForTransition(actuationPhase)) {
    if (actuationPhase == 0) {  // if currently lifting leg
      if (fabs(cmd_vector[0]) < 0.01 || fabs(rpy[gaitPhase]) > TILT_NOMINAL) { // if no forward command OR if body is tilted, stop at the neutral body translation
        motors[4].states.q_d = 0;       // move to the neutral point and stop
        motors[5].states.q_d = 0;
      } else {
        motors[4].states.q_d = gaitSetpoints[gaitPhase][1] * cmd_vector[0];                       // forward translation scaled by x-component of cmd_vector
        motors[5].states.q_d = min(motors[5].states.q_max, max(motors[5].states.q_min, yaw_cmd)); // limit the max turn per step
      }

    } else if (actuationPhase == 1) { // if currently translating forward
      // change to torque control for leg touchdown
      motors[gaitPhase * 2].setControlMode(1);
      motors[gaitPhase * 2 + 1].setControlMode(1);

      // limit velocity in torque control during touchdown
      ODrive_CAN.SetLimits(gaitPhase*2, VEL_LEG_LIMIT_CONTACT, CURRENT_LEG_LIMIT);
      ODrive_CAN.SetLimits(gaitPhase*2 + 1, VEL_LEG_LIMIT_CONTACT, CURRENT_LEG_LIMIT);

      // set torque command to step down
      motors[gaitPhase * 2].states.tau_d = touchdownTorque[gaitPhase * 2][0];
      motors[gaitPhase * 2 + 1].states.tau_d = touchdownTorque[gaitPhase * 2 + 1][0];

      // reapply locomotion mechanism setpoints in the case of resuming locomotion from standstill
      motors[4].states.q_d = gaitSetpoints[gaitPhase][1] * cmd_vector[0];                       // forward translation scaled by x-component of cmd_vector
      motors[5].states.q_d = min(motors[5].states.q_max, max(motors[5].states.q_min, yaw_cmd)); // limit the max turn per step
      
    } else if (actuationPhase == 2) { // if currently touching down
      isBlocking = false;
      corrected = false;
      
      // remember contact points
      q_last_contacts[gaitPhase*2] = motors[gaitPhase*2].states.q;
      q_last_contacts[gaitPhase*2 + 1] = motors[gaitPhase*2 + 1].states.q;
      gaitPhase = (gaitPhase + 1) % N_GAIT_PHASES;  // put the other pairs of legs in swing phase
      if (gaitPhase == 0) {
        gaitCycles++; // if the gait phase is back to 0, increment the number of completed gait cycles
      }

      // update and/or reset swing/stance setpoints based on last contact conditions
      for (uint8_t i = 0; i < 2; i++) {
        if (fabs(q[gaitPhase * 4 + i * 2] - q[gaitPhase * 4 + i * 2 + 1]) > DQ_UNEVEN_TERRAIN) { // if the previous stance legs had uneven ground contact
          float q_max = max(q[gaitPhase * 4 + i * 2], q[gaitPhase * 4 + i * 2 + 1]); // longer stroke of the two legs
          float dq = q_max - motors[gaitPhase * 2 + i].states.q_d;            // additional leg stroke due to uneven terrain
          motors[gaitPhase * 2 + i].states.q_d = max(Q_LEG_MIN, gaitSetpoints[gaitPhase][0] - dq); // retract more by the additional leg stroke

        } else {  
          motors[gaitPhase * 2 + i].states.q_d = gaitSetpoints[gaitPhase][0];  // if even terrain, use the nominal swing leg setpoint

        }
        
        ODrive_CAN.SetLimits(gaitPhase*2 + i, VEL_LEG_LIMIT_NOMINAL, CURRENT_LEG_LIMIT);
        ODrive_CAN.SetTrajVelLimit(gaitPhase*2 + i, VEL_LEG_TRAJ_RETRACT);  // retract the swing legs slowly to allow gradual weight shifting
        ODrive_CAN.SetTrajAccelLimits(gaitPhase*2 + i, ACCEL_LEG_TRAJ_NOMINAL, DECEL_LEG_TRAJ_NOMINAL);
        inContact[gaitPhase * 2 + i] = false;                               // set the contact state of swing legs to false again
      }
    }
    actuationPhase = (actuationPhase + 1) % N_ACTUATION_PHASES;

    // update setpoints that do not involve a gait phase transition here
  } else if (!isBlocking) {
    if (actuationPhase == 0) {  // if currently lifting leg

      // if the gradual weight shifting is done
      if (motors[gaitPhase*2].states.q < (q_last_contacts[gaitPhase*2] - DQ_LEG_RETRACT) &&
          motors[gaitPhase*2 + 1].states.q < (q_last_contacts[gaitPhase*2 + 1] - DQ_LEG_RETRACT)) {

        uint8_t stance = (gaitPhase + 1) % N_GAIT_PHASES;
        // if the stance legs are still in torque control
        if (motors[stance * 2].states.ctrl_mode != 3) {
          // zero out torque command
          motors[stance*2].sendCommand(1, 0);
          motors[stance*2 + 1].sendCommand(1, 0);

          // put stance legs into position control
          motors[stance * 2].setControlMode(3);
          motors[stance * 2 + 1].setControlMode(3);

          // set the desired position to the current position
          motors[stance * 2].states.q_d = motors[stance * 2].states.q;
          motors[stance * 2 + 1].states.q_d = motors[stance * 2 + 1].states.q;

          // retract the swing legs at full speed now
          ODrive_CAN.SetTrajVelLimit(gaitPhase*2, VEL_LEG_TRAJ_NOMINAL);
          ODrive_CAN.SetTrajVelLimit(gaitPhase*2 + 1, VEL_LEG_TRAJ_NOMINAL);
        }
      }
    } else if (actuationPhase == 1) {   // if currently translating forward
      
      // if there is no more forward command, stop at the current translation position
      if (fabs(cmd_vector[0]) < 0.01) {
        motors[4].states.q_d = motors[4].states.q;
        motors[5].states.q_d = motors[5].states.q;

      // else if there's body tilt and hasn't been corrected yet, stop at the neutral translation position
      } else if (!corrected && fabs(rpy[gaitPhase]) > TILT_NOMINAL) {
        motors[4].states.q_d = 0;
        motors[5].states.q_d = 0;

      // else, continue moving according to the current command
      } else {
        motors[4].states.q_d = gaitSetpoints[gaitPhase][1] * cmd_vector[0];                       // forward translation scaled by x-component of cmd_vector
        motors[5].states.q_d = min(motors[5].states.q_max, max(motors[5].states.q_min, yaw_cmd)); // limit the max turn per step  
      }
      
    } else if (actuationPhase == 2) {  // if currently touching down
      for (uint8_t i = 0; i < 2; i++) {     // for each swing leg motor
        if (!inContact[gaitPhase * 2 + i] && // if the motor is not in contact
            motors[gaitPhase * 2 + i].states.q - gaitSetpoints[gaitPhase][0] > DQ_START_CONTACT) { // AND the leg stroke is past the initial displacement
              
          motors[gaitPhase * 2 + i].states.tau_d = touchdownTorque[gaitPhase * 2 + i][1];        // apply the lower torque command
        }
      }
    }
  }
}

void standUp() {
  uint8_t stance = (gaitPhase + 1) % N_GAIT_PHASES;
  motors[stance * 2].states.q_d = gaitSetpoints[stance][2];
  motors[stance * 2 + 1].states.q_d = gaitSetpoints[stance][2];
  ODrive_CAN.SetTrajVelLimit(stance * 2, VEL_LEG_TRAJ_STANDUP);
  ODrive_CAN.SetTrajVelLimit(stance * 2 + 1, VEL_LEG_TRAJ_STANDUP);
  inContact[stance * 2] = true;
  inContact[stance * 2 + 1] = true;

  // update states while standing up
  while (!motors[stance * 2].states.holding && !motors[stance * 2 + 1].states.holding) {
    handleODriveCANMsg();
    updateStates();
    updateMotorCommands();
    transmitData();
  }
  q_last_contacts[0] = motors[0].states.q;
  q_last_contacts[1] = motors[1].states.q;
  Serial.println("Starting");
}

void homeMotors() {
  // update Actuator states for the first time
  unsigned long curT = millis();
  while (millis() - curT < DT_FETCH_UPDATE) {
    handleODriveCANMsg();
    updateStates();
  }

  snprintf(sentData, sizeof(sentData), "Homing all legs...\n\n");
  transmitMsg(sentData);

  // put leg motors in velocity control with passthrough input
  // motors are already in closed-loop control from calling initActuators()
  for (uint8_t i = 0; i < 4; i++) {
    ODrive_CAN.SetLimits(i, VEL_LEG_LIMIT_CONTACT, CURRENT_LEG_LIMIT_HOMING);
    motors[i].setControlMode(2);
    motors[i].sendCommand(2, VEL_LEG_HOMING);
  }

  // read CAN messages and update states while waiting for motors to start moving
  curT = millis();
  while (millis() - curT < DT_FETCH_UPDATE*2) {
    handleODriveCANMsg();
    updateStates();
  }

  bool moving[4] = {true, true, true, true};

  // check leg motors to see if they've hit an endstop
  while (moving[0] || moving[1] || moving[2] || moving[3]) {
    handleODriveCANMsg();    // update states
    updateStates();
    checkStopButton();
    if(stopSignal) {
      delay(10000000);
    }

    for (uint8_t i = 0; i < 4; i++ ) {
      if (moving[i] && fabs(motors[i].states.q_dot) < fabs(VEL_LEG_HOMING_STOP)) {
        motors[i].sendCommand(2, 0);
        delay(DT_CLOSED_LOOP);
        motors[i].setControlMode(3);
        ODrive_CAN.SetLimits(i, VEL_LEG_LIMIT_NOMINAL, CURRENT_LEG_LIMIT);
        moving[i] = false;
        motors[i].states.pos_home = motors[i].states.pos_abs;
        motors[i].states.homed = true;
        motors[i].states.pos_rel = motors[i].states.pos_abs - motors[i].states.pos_home;
        snprintf(sentData, sizeof(sentData), "Leg actuator %d done retracting.\n******************************************************\n", i + 1);
        transmitMsg(sentData);
      }
    }
  }
  snprintf(sentData, sizeof(sentData), "Finished homing legs.\n\n");
  transmitMsg(sentData);

  curT = millis();
  while (millis() - curT < 1000) {
    handleODriveCANMsg();
    updateStates();
  }

  for (uint8_t i = 0; i < N_LEGS; i++ ) {
    encoders[i].write(0);
  }

  snprintf(sentData, sizeof(sentData), "Homing the yaw mechanism...\n");
  transmitMsg(sentData);
  motors[5].setControlMode(3);
//  motors[5].sendCommand(3, POS_YAW_HOME);  // yaw mechanism's range of motion is < 1 rev so the absolute encoder can be used directly

//  curT = millis();
//  while (millis() - curT < 1000 || fabs(motors[5].states.q_dot) > fabs(VEL_YAW_HOMING_STOP)) { // check what speed value is good for stoppping
//    handleODriveCANMsg();
//    updateStates();
//    checkStopButton();
//    if(stopSignal) {
//      delay(10000000);
//    }
//  }

  motors[5].states.pos_home = motors[5].states.pos_abs;
  motors[5].states.homed = true;
  motors[5].states.pos_rel = motors[5].states.pos_abs - motors[5].states.pos_home;

  snprintf(sentData, sizeof(sentData), "Finished homing the yaw mechanism.\n\n");
  transmitMsg(sentData);

  // home the translation motor
  snprintf(sentData, sizeof(sentData), "Homing the translation mechanism...\n");
  transmitMsg(sentData);

  motors[4].setControlMode(2);
  ODrive_CAN.SetLimits(4, VEL_TRANS_LIMIT_NOMINAL, CURRENT_TRANS_LIMIT_HOMING);
  motors[4].sendCommand(2, VEL_TRANS_HOMING);

  curT = millis();
  while (millis() - curT < DT_FETCH_UPDATE * 5) {
    handleODriveCANMsg();
    updateStates();
  }

  bool moving2 = true;
  while (moving2) {
    handleODriveCANMsg();
    updateStates();
    checkStopButton();
    if(stopSignal) {
      delay(10000000);
    }
    
    if (moving2 && fabs(motors[4].states.q_dot) < fabs(VEL_TRANS_HOMING_STOP)) {
      motors[4].sendCommand(2, 0);
      delay(DT_CLOSED_LOOP);
      motors[4].setControlMode(3);
      ODrive_CAN.SetLimits(4, VEL_TRANS_LIMIT_NOMINAL, CURRENT_TRANS_LIMIT);
      moving2 = false;
      motors[4].sendCommand(3, motors[4].states.pos_abs * motors[4].params.T + POS_TRANS_HOMING_OFFSET); // move to the zero position

      curT = millis();
      while (millis() - curT < 500 || fabs(motors[4].states.q_dot) > fabs(VEL_TRANS_HOMING_STOP)) {
        handleODriveCANMsg();
        updateStates();
      }
      motors[4].states.pos_home = motors[4].states.pos_abs;
      motors[4].states.homed = true;
      motors[4].states.pos_rel = motors[4].states.pos_abs - motors[4].states.pos_home;
    }
  }

  snprintf(sentData, sizeof(sentData), "Finished homing the translation mechanism.\n\n");
  transmitMsg(sentData);

  snprintf(sentData, sizeof(sentData), "Homing finished.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void updateJoints() {
  // read leg encoders and update leg positions
  curT = millis();
  uint8_t dt = curT - prevTEncoder;
  prevTEncoder = curT;

  for (uint8_t i = 0; i < N_LEGS; i++) {
    float q_prev = q[i];
    q[i] = T_PINION * ((float)encoders[i].read() / CPR_AMT102_V);
    q_dot[i] = (q[i] - q_prev) / dt;
  }

  q[8] = motors[4].states.q; // update translation mechanism joint
  q[9] = motors[5].states.q; // update yaw mechanism joint
}

void updateACS711EX() {
  ACS711EX_fault = digitalRead(ACS711EX_FAULT);
  if (!ACS711EX_fault) {
    snprintf(sentData, sizeof(sentData), "Battery output current exceeded 31A.\n");
    transmitMsg(sentData);
    stopSignal = true;
  } else {
    ACS711EX_Vout = analogRead(ACS711EX_OUT);
    batteryCurrent = (3.3 * ACS711EX_Vout / 1023 - 3.3 / 2) * 1000 / MV_PER_A; // convert counts to voltage, then to current
  }
}


void updateIMU() {
  if (bno08x.wasReset()) {  // if the sensor was reset, pose estimation has drifted. stop the robot
    snprintf(sentData, sizeof(sentData), "IMU was reset.\n");
    transmitMsg(sentData);
    stopSignal = true;

    if (! bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      snprintf(sentData, sizeof(sentData), "IMU could not be reenabled.\n");
      transmitMsg(sentData);
    }
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        quat[0] = sensorValue.un.rotationVector.real;
        quat[1] = sensorValue.un.rotationVector.i;
        quat[2] = sensorValue.un.rotationVector.j;
        quat[3] = sensorValue.un.rotationVector.k;
        quat_accuracy = sensorValue.un.rotationVector.accuracy;
        quat2rpy();
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        gyro[0] = sensorValue.un.gyroscope.x;
        gyro[1] = sensorValue.un.gyroscope.y;
        gyro[2] = sensorValue.un.gyroscope.z;
        break;
    }
  }
}

void quat2rpy() {
  float sinr_cosp = 2 * (quat[0] * quat[1] + quat[2] * quat[3]);
  float cosr_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
  rpy[0] = atan2(sinr_cosp, cosr_cosp) * 180 / PI;

  float sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
  if (abs(sinp) >= 1) {
    rpy[1] = 90;
    if (sinp < 0) {
      rpy[1] *= -1;
    }
  } else {
    rpy[1] = asin(sinp) * 180 / PI;
  }

  float siny_cosp = 2 * (quat[0] * quat[3] + quat[1] * quat[2]);
  float cosy_cosp = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]);
  rpy[2] = atan2(siny_cosp, cosy_cosp) * 180 / PI;
}

void initIMU(TwoWire *wire) {
  wire->begin();
  if (!bno08x.begin_I2C(0x4A, wire, 0)) {      // begin_I2C(uint8_t i2c_addr = BNO08x_I2CADDR_DEFAULT, TwoWire *wire = &Wire, int32_t sensor_id = 0);
    digitalWrite(LED, HIGH);
    snprintf(sentData, sizeof(sentData), "Couldn't find BNO08x chip.\n");
    transmitMsg(sentData);
    while (true) {}
  } else {
    bno08x.wasReset();
    snprintf(sentData, sizeof(sentData), "BNO08x successfully connected.\n\n");
    transmitMsg(sentData);
  }

  snprintf(sentData, sizeof(sentData), "Setting desired reports.\n\n");
  transmitMsg(sentData);
  if (! bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    snprintf(sentData, sizeof(sentData), "Could not enable rotation vector.\n\n");
    transmitMsg(sentData);
    while (true) {}
  }

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    snprintf(sentData, sizeof(sentData), "Could not enable gyroscope.\n\n");
    transmitMsg(sentData);
    while (true) {}
  }

  snprintf(sentData, sizeof(sentData), "BNO08x initialized.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void updateStates() {
  curT = millis();
  uint8_t dt = curT - prevTState;
  if (dt >= DT_STATE) {
    prevTState = curT;
    updateJoints();
    updateKinematics();
    updateIMU();
    updateACS711EX();
  }
}

// updates motor commands on all axes
void updateMotorCommands() {
  curT = millis();
  if (curT - prevTActuation >= DT_ACTUATION) {
    prevTActuation = curT;
    for (uint8_t i = 0; i < N_ACTUATOR; i++) {
      if (motors[i].states.ctrl_mode == 3) {  // if in position control
        motors[i].sendCommand(3, motors[i].states.q_d);
        motors[i].states.holding = fabs(motors[i].states.q - motors[i].states.q_d) < Q_ERROR_MAX;

      } else if (motors[i].states.ctrl_mode == 1) {  // if in torque control
        motors[i].sendCommand(1, motors[i].states.tau_d);
      }
    }
  }
}

void normalizeJoystickVelocity() {
  joystick_x_norm = max(abs(joystick_x - JOYSTICK_X_CENTER) - JOYSTICK_X_DEADZONE / 2, 0) / JOYSTICK_X_MAX;
  joystick_y_norm = max(abs(joystick_y - JOYSTICK_Y_CENTER) - JOYSTICK_Y_DEADZONE / 2, 0) / JOYSTICK_Y_MAX;

  int dir_1 = (joystick_x > JOYSTICK_X_CENTER) - (joystick_x < JOYSTICK_X_CENTER);
  int dir_2 = (joystick_y > JOYSTICK_Y_CENTER) - (joystick_y < JOYSTICK_Y_CENTER);
  if (fabs(joystick_x_norm) < 0.01) {
    dir_1 = 1;
  }
  if (fabs(joystick_y_norm) < 0.01) {
    dir_2 = 1;
  }

  joystick_x_norm = dir_1 * joystick_x_norm;
  joystick_y_norm = dir_2 * joystick_y_norm;
}

// samples the joystick voltages
void readJoystick() {
  joystick_x = analogRead(XOUT);
  joystick_y = analogRead(YOUT);
  joystick_select = digitalRead(SEL);
  normalizeJoystickVelocity();
}

// enters closed-loop control
void initActuators() {
  for (uint8_t i = 0; i < N_ACTUATOR; i++) {
    snprintf(sentData, sizeof(sentData), "Initializing actuator %d...\n", i + 1);
    transmitMsg(sentData);
    motors[i].enable();
    delay(DT_CLOSED_LOOP);  // entering closed-loop is not instant
  }

  snprintf(sentData, sizeof(sentData), "Actuators initialized.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
  delay(2000);  // DO NOT REMOVE; need time for common mode noise on SPI lines caused by motor power wires to dissipate after entering closed-loop control
}

// look for the ODrive CAN heartbeat msg and check errors
void initODrives() {
  delay(100);
  for (uint8_t i = 0; i < N_ACTUATOR; i++) {
    snprintf(sentData, sizeof(sentData), "Checking ODrive axis %d...\n", i + 1);
    transmitMsg(sentData);

    readHeartbeat(i);
    snprintf(sentData, sizeof(sentData), "current state: %d\taxis error: %lu\n\n", heartbeat_msg.currentState, heartbeat_msg.axisError);
    transmitMsg(sentData);
    if (heartbeat_msg.axisError) {
      digitalWrite(LED, HIGH);
      snprintf(sentData, sizeof(sentData), "Stopping. Check the axis error.\n");
      transmitMsg(sentData);
      while (true) {} // if there's an axis error, do not proceed
    }
  }

  snprintf(sentData, sizeof(sentData), "ODrives connected.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

// calls the Estop() function on all axes
void stopODrives() {
  for (uint8_t i = 0; i < N_ACTUATOR; i++) {
    snprintf(sentData, sizeof(sentData), "Stopping ODrive axis %d...\n", i + 1);
    transmitMsg(sentData);
    ODrive_CAN.Estop(i);     // calling the Estop function will raise the error flag; needs ClearErrors(axis_id) or power cycling
  }

  snprintf(sentData, sizeof(sentData), "ODrives stopped.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void checkStopButton() {
  joystick_select = digitalRead(SEL);
  
  if (!joystick_select) {
    uint8_t hold_count = 0;
    for(uint8_t i = 0; i < 10; i++) {
      joystick_select = digitalRead(SEL);
      if (!joystick_select) {
        ++hold_count;
      }
    }
    if (hold_count > 7) {
      Serial.println("Stop button pressed.");
      stopSignal = true;
    }

    Serial.println("End.");
    digitalWrite(LED, HIGH);
  }
}

// reads the next CAN msg from an ODrive CAN node
// calls member variables of Actuator objects
int handleODriveCANMsg() {
  if (ODrive_CAN.ReadMsg(msg)) {
    axis_id = getAxisID(msg);
    cmd_id = getCmdID(msg);

    if (cmd_id == 0x001) {          // read heartbeat and handle errors if any
      ODrive_CAN.Heartbeat(heartbeat_msg, msg);
      motors[axis_id].states.axisError = heartbeat_msg.axisError;
      motors[axis_id].states.state = heartbeat_msg.currentState;

      if (motors[axis_id].states.axisError && motors[axis_id].states.axisError != 67108864) {
        snprintf(sentData, sizeof(sentData), "ERROR: Axis %d encountered error %lu.\n", axis_id + 1, motors[axis_id].states.axisError);
        transmitMsg(sentData);
        writeToCard(sentData);
        stopSignal = true;
      }

    } else if (cmd_id == 0x003) {          // read active_error
      // current ODriveTeensyCAN version expects the 64-bit msg from v3.6
//      active_error = ODrive_CAN.GetMotorErrorResponse(msg);

    } else if (cmd_id == 0x01D) {          // read controller error
      ctrlr_error = ODrive_CAN.GetControllerErrorResponse(msg);

    } else if (cmd_id == 0x009) {   // read position/velocity
      ODrive_CAN.GetPositionVelocityResponse(encoder_msg, msg);
      motors[axis_id].states.pos_abs = encoder_msg.posEstimate;
      motors[axis_id].states.pos_rel = motors[axis_id].states.pos_abs - motors[axis_id].states.pos_home;
      motors[axis_id].states.vel = encoder_msg.velEstimate;
      motors[axis_id].states.q = motors[axis_id].params.direction * motors[axis_id].params.T * motors[axis_id].states.pos_rel;
      motors[axis_id].states.q_dot = motors[axis_id].params.direction * motors[axis_id].params.T * motors[axis_id].states.vel;

    } else if (cmd_id == 0x014) {   // read motor current
      ODrive_CAN.GetIqResponse(Iq_msg, msg);
      motors[axis_id].states.torque = motors[axis_id].params.K_T * Iq_msg.iqMeasured;
      motors[axis_id].states.current = Iq_msg.iqMeasured;

      // moving average smoothing on motor torque
      Rec tmp;
      tauQueue[axis_id].peekIdx(&tmp, 0);
      tau_hat[axis_id] = N_TAU_SAMPLE * tau_hat[axis_id] - tmp.val;
      tmp.val = motors[axis_id].states.torque;
      tauQueue[axis_id].push(&tmp);
      tau_hat[axis_id] = (tau_hat[axis_id] + tmp.val) / N_TAU_SAMPLE;
    }
    return cmd_id;
  } else {
    return -1;
  }
}

// blocking function that reads the heartbeat msg data
// used only in the initODrive() function
void readHeartbeat(uint8_t axis) {
  while (true) {
    if (ODrive_CAN.ReadMsg(msg)) {
      axis_id = getAxisID(msg);
      cmd_id = getCmdID(msg);
      if ((axis_id == axis) & (cmd_id == 0x001)) {
        ODrive_CAN.Heartbeat(heartbeat_msg, msg);
        motors[axis_id].states.state = heartbeat_msg.currentState;
        motors[axis_id].states.axisError = heartbeat_msg.axisError;
        return;
      }
    }
  }
}

// extracts and returns the axis/node ID from the ODrive's CAN message
uint8_t getAxisID(CAN_message_t msg) {
  return (msg.id & ((int)(pow(2, AXIS_ID_LENGTH)) - 1) << CMD_ID_LENGTH) >> CMD_ID_LENGTH; // axis ID is the 6 highest bits
}

// extracts and returns the command ID from the ODrive's CAN message
uint8_t getCmdID(CAN_message_t msg) {
  return msg.id & ((int)pow(2, CMD_ID_LENGTH) - 1); // command ID is the 5 lower bits
}

void initSDCard() {
  // initialize SD card
  if (!SD.begin(chipSelect)) {
    digitalWrite(LED, HIGH);
    Serial.println("SD card failed or not present.\n");
    while (1) {}
  }
  Serial.println("SD card initialized.\n");

  // parse through existing data files
  String file_name = "";

  // using Teensy's internal RTC
  file_name = String(year()) + '_' +
              String(month()) + '_' +
              String(day()) + '_' +
              String(hour()) + '_' +
              String(minute()) + '_' +
              String(second()) + ".txt";
  
  // using external RTC
//  tmElements_t tm;
//  if (RTC.read(tm)) {
//    file_name = String(tmYearToCalendar(tm.Year)) + '_' +
//                String(tm.Month) + '_' +
//                String(tm.Day) + '_' +
//                String(tm.Hour) + '_' +
//                String(tm.Minute) + '_' +
//                String(tm.Second) + ".txt";
//  } else {
//    Serial.println("Couldn't read time");
//    while(1){};
//  }

  // name the data file with the next number
  file_name.toCharArray(dataFileName, sizeof(dataFileName));
  dataFile = SD.open(dataFileName, FILE_WRITE);
  Serial.printf("Data being saved to: %s\n\n", dataFileName);
  delay(100);
}

void initTeensy() {
  delay(100);
  Serial.println("Teensy started.\n");

  // add digital pin initialization here
  pinMode(ACS711EX_FAULT, INPUT);
  pinMode(SEL, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  // time sync for Teensy's internal RTC
  // comment out if not using it
  setSyncProvider(getTeensy3Time);

  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC.\n");
    digitalWrite(LED, HIGH);
    while(1) {};
  } else {
    Serial.println("RTC has set the system time.\n");
  }

  time_t t = processSyncMessage();
  if (t != 0) {
    Teensy3Clock.set(t); // set the RTC
    setTime(t);
  }
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}

void transmitData() {
  curT = millis();
  if (curT - prevTPrint >= DT_PRINT) {
    prevTPrint = curT;
    snprintf(sentData, sizeof(sentData), "%.3f\t", (float)curT / 1000);
    transmitMsg(sentData);
    writeToCard(sentData);

    Serial.print("a: ");
    for(uint8_t i = 0; i < 6; i++) {
      if(fabs(motors[i].states.q) < 1e-2 && motors[i].states.q < 0) {
        motors[i].states.q = 0;
      }
    }
    snprintf(sentData, sizeof(sentData), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motors[0].states.q, motors[1].states.q, motors[2].states.q, motors[3].states.q, motors[4].states.q, motors[5].states.q); // motor positions
    transmitMsg(sentData);
    writeToCard(sentData);

    Serial.print("a_d: ");
    for(uint8_t i = 0; i < 6; i++) {
      if(fabs(motors[i].states.q_d) < 1e-2 && motors[i].states.q_d < 0) {
        motors[i].states.q_d = 0;
      }
    }
    snprintf(sentData, sizeof(sentData), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motors[0].states.q_d, motors[1].states.q_d, motors[2].states.q_d, motors[3].states.q_d, motors[4].states.q_d, motors[5].states.q_d);   // motor setpoints
    transmitMsg(sentData);
    writeToCard(sentData);

    //    Serial.print("q: ");
    snprintf(sentData, sizeof(sentData), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9]);   // joint positions
    //    transmitMsg(sentData);
    writeToCard(sentData);

    Serial.printf("h: %.2f\t", h);

    Serial.print("tau: ");
    for(uint8_t i = 0; i < 6; i++) {
      if(fabs(tau_hat[i]) < 1e-2 && tau_hat[i] < 0) {
        tau_hat[i] = 0;
      }
    }
    snprintf(sentData, sizeof(sentData), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             tau_hat[0], tau_hat[1], tau_hat[2], tau_hat[3], tau_hat[4], tau_hat[5]);   // motor torque
    transmitMsg(sentData);
    writeToCard(sentData);

    Serial.print("inContact: ");
    snprintf(sentData, sizeof(sentData), "%d\t%d\t%d\t%d\t",
             inContact[0], inContact[1], inContact[2], inContact[3]);
    transmitMsg(sentData);
    writeToCard(sentData);

    Serial.print("states: ");
    snprintf(sentData, sizeof(sentData), "%.2f\t%.2f\t%.2f\t%.2f\t%d\t%d\t%lu\n",
             batteryCurrent, rpy[0], rpy[1], rpy[2], gaitPhase, actuationPhase, gaitCycles);    // roll pitch yaw, gait variables
    transmitMsg(sentData);
    writeToCard(sentData);
  }
}

void writeToCard(char data[]) {
  if (dataFile) {
    dataFile.print(data);
  } else {
    digitalWrite(LED, HIGH);
    Serial.println("Error opening data file");
    stopSignal = true;
  }
}

void transmitMsg(char data[]) {
  strcpy(sentData, data);
  SERIAL_USB.print(sentData);
}
