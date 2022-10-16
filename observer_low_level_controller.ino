//#include <ODriveTeensyCAN.h>
#include <Actuator.h>
#include <Adafruit_BNO08x.h>

////////////////////////////////////////////////////////////////////////////////////////////////
// macros
#define PI 3.1415926535897932384626433832795

// communication parameters
#define SERIAL_USB Serial             // Serial (USB), Serial<x> (Hardware serial) e.g. Serial5
#define WIRE_IMU Wire2                // Using I2C bus 2: SCL2, SDA2
#define BAUD_RATE_ODRIVE_CAN 500000  // baud rate for ODrive CAN2.0
#define MAX_DATA_SIZE 128             // max number of chars in sent/received data array
#define AXIS_ID_LENGTH 6              // number of bits of the CAN message's axis ID
#define CMD_ID_LENGTH 5               // number of bits of the CAN message's command ID
#define BNO08X_RESET -1

#define DT_CLOSED_LOOP 50     // wait time in ms between setup function calls
#define DT_FETCH_UPDATE 100   // duration in ms to handle CAN messages and to update states in between function calls in setup
#define DT_ACTUATION 10       // motor command update period in ms
#define DT_STATE 10           // motor state update period in ms
#define DT_SETPOINT 10        // joint setpoint update period in ms
#define DT_PRINT 10           // data print period in ms

// actuation parameters
#define N_ODRIVE 3                // number of ODrive motor controllers
#define N_ACTUATOR 6              // number of actuators
#define D5312S_KV 330             // speed constant of the leg motor
#define GL60_KV 25                // speed constant of the trans/yaw motor
#define HOMING_VEL_LEG -5.        // homing velocity for the leg motor in turns/s
#define HOMING_VEL_TRANS 1.5      // homing velocity for the translation motor in turns/s
#define HOMING_OFFSET_TRANS -1.9  // distance from the joint limit to the zero position for the translation mechanism in turns
#define Q_YAW_ZERO -0.005         // encoder reading at yaw mechanism's zero position
#define Q_ERROR_MAX 0.15          // maximum allowable position error

#define N_DIFF_RING 34.0    // number of teeth on differential ring gear
#define N_DIFF_PINION 11.0  // number of teeth on the drive hub
#define D_LEG_SPUR 17.0     // pitch diameter (in mm) of the leg spur gear
#define D_TRANS_SPUR 19.0   // pitch diameter (in mm) of the translation spur gear
#define N_T_LEG (N_DIFF_PINION/N_DIFF_RING)*(D_LEG_SPUR/2)    // this evaluates to 2.75

////////////////////////////////////////////////////////////////////////////////////////////////
// communication variables
char receivedData[MAX_DATA_SIZE] = "";
char sentData[MAX_DATA_SIZE] = "";
unsigned long dt;

// time variables
unsigned long curT;                 // current timestamp
unsigned long prevTPrint = 0;       // timestamp at last serial transmission
unsigned long prevTState = 0;       // timestamp at last state update
unsigned long prevTSetpoint = 0;    // timestamp at last setpoint update
unsigned long prevTActuation = 0;   // timestamp at last motor command update

// ODrive and actuator variables
ODriveTeensyCAN myCAN(BAUD_RATE_ODRIVE_CAN);
CAN_message_t msg;
uint8_t axis_id;
uint8_t cmd_id;

// structs for parsing the CAN message
struct HeartbeatMsg_t myHeartbeatMsg;
struct EncoderEstimatesMsg_t myEncoderMsg;
struct IqMsg_t myIqMsg;

// Actuator instances that hold motor state variables
Actuator myActuators[N_ACTUATOR] = {Actuator(myCAN, SERIAL_USB, 0, 8.27/D5312S_KV, -0.5, 190),
                                    Actuator(myCAN, SERIAL_USB, 1, 8.27/D5312S_KV, -0.5, 190),
                                    Actuator(myCAN, SERIAL_USB, 2, 8.27/D5312S_KV, -0.5, 190),
                                    Actuator(myCAN, SERIAL_USB, 3, 8.27/D5312S_KV, -0.5, 190),
                                    Actuator(myCAN, SERIAL_USB, 4, 8.27/GL60_KV, -3, 3),
                                    Actuator(myCAN, SERIAL_USB, 5, 8.27/GL60_KV, -0.4, 0.4)
                                   };

// tunable gait parameters
float q_stance = 140;        // target position for lower body stance in turns
float q_stance_offset = 38; // target position offset for upper body stance in turns
float dq_swing_pcnt = 0.8;  // proportion of q_stance (for both lower and upper legs) to retract during swing
float q_trans = 1.2;        // target position for body translation in turns
float q_yaw = 0.05;         // target body yaw for turning step in turns
float max_tilt = 0.04;      // max allowable pitch or roll in radians (not used yet)

// early transition thresholds
// value of 1 means one gait phase is fully completed before next one begins
float q_0[2] = {0, 0};              // initial position of the joint corresponding to the current gait phase
// used as q_leg_0 or q_trans_0 in early transition calculation
float dq_early_step_pcnt = 0.5;     // % of (q_trans_f - q_trans_0) at which step down begins early
float dq_early_lift_pcnt = 0.95;    // % of (q_leg_f - q_leg_0) in stance at which lift off begins early
float dq_early_correct_pcnt = 0.5;  // % of (q_leg_f - q_leg_0) in swing at which tilt correction begins early
float dq_early_trans_pcnt = 1.;     // % of (q_leg_f - q_leg_0) in tilt correction at which translation begins early

float gaitSetpoints[2][3] = {           // joint target positions for upper/lower body gait phases
  { -q_trans, q_stance + q_stance_offset, q_stance*(1 - dq_swing_pcnt) + q_stance_offset},
  {q_trans,   q_stance,                   q_stance*(1 - dq_swing_pcnt)}
};
float gaitPhaseOffset[] = {dq_early_step_pcnt, dq_early_lift_pcnt, dq_early_correct_pcnt, dq_early_trans_pcnt};

// IMU variables
Adafruit_BNO08x  bno08x(BNO08X_RESET);            // I2C IMU
sh2_SensorValue_t sensorValue;                    // BNO8X helper object
uint8_t quat_accuracy = 0;
double quat[4] = {0, 0, 0, 0};
double rpy[3] = {0, 0, 0};
double rpy_0[3] = {0, 0, 0};

// state variables
uint8_t swing = 0;        // 0 if upper swing, 1 if lower swing; swing switches when gaitPhase changes to 2
uint8_t gaitPhase = 1;    // current gait phase; (0 translate) -> (1 step) -> (2 lift) - -> (3 tilt correct); 3 may be skipped if there's no tilt
uint32_t gaitCycles = 0;  // number of completed gait cycles
bool stopSignal = false;

//////////////////////////////////////////////////////////////////////////
void setup() {
  printStartMsg();
  initODrives();    // check motor/encoder calibration
  initIMU(&WIRE_IMU);
  homeMotors();     // actuator position after homing is assumed to be the new zero position

  updateSetpoints();
  updateMotorCommands();
}

void loop() {
  while (!stopSignal & (gaitCycles < 100)) {
    handleODriveCANMsg();
    updateStates();
    updateGait();
    updateMotorCommands();
    transmitData();
  }

  stopODrives();
  Serial.println("End.");
  delay(10000);
}

void updateGait() {
  curT = millis();
  if (curT - prevTSetpoint >= DT_SETPOINT) {
    prevTSetpoint = curT;

    // identify the joint positions to check for the current gait phase
    uint8_t idx[2];
    uint8_t stance = (swing + 1) % 2;
    if (gaitPhase == 0) {
      idx[0] = 4;   // translation motor
      idx[1] = 4;
    } else if (gaitPhase == 3) {
      idx[0] = stance * 2;
      idx[1] = stance * 2 + 1;
    } else {
      idx[0] = swing * 2;     // front or right leg
      idx[1] = swing * 2 + 1; // rear or left leg
    }

    //    float tilt[2] = {(rpy[0] - rpy_0[0])*PI/180, (rpy[1] - rpy_0[1])*PI/180};  // roll and pitch in radians

    // if ready for gait phase transition
    if (isReadyForTransition(idx[0], idx[1], gaitPhase)) {
      gaitPhase = (gaitPhase + 1) % 4; // move to next gait phase
      if (gaitPhase == 2 ) {           // if a gait cycle is completed, switch the swing body
        swing = (swing + 1) % 2;
        stance = (swing + 1) % 2;
        gaitCycles++;
      }

      if (gaitPhase == 3) {     // skip the tilt correct for now
        gaitPhase = (gaitPhase + 1) % 4;
      }

//      if (gaitPhase == 3 && abs(tilt[stance]) < MAX_TILT) {    // if no tilt to correct, skip tilt correct phase
//        gaitPhase = (gaitPhase + 1) % 4;
//      }
      updateSetpoints();  // update q_target
    }
  }
}

bool isReadyForTransition(uint8_t idx1, uint8_t idx2, uint8_t gaitPhase) {
  // 1 or -1 to determine direction of inequality condition
  int sign_1 = (myActuators[idx1].states.q_target > q_0[0]) - (myActuators[idx1].states.q_target < q_0[0]);
  int sign_2 = (myActuators[idx2].states.q_target > q_0[1]) - (myActuators[idx2].states.q_target < q_0[1]);

  // joint positions past which the next gait phase will begin
  float q_transition_1 = (myActuators[idx1].states.q_target - q_0[0]) * gaitPhaseOffset[gaitPhase] + q_0[0] - sign_1 * Q_ERROR_MAX;
  float q_transition_2 = (myActuators[idx2].states.q_target - q_0[1]) * gaitPhaseOffset[gaitPhase] + q_0[1] - sign_2 * Q_ERROR_MAX;

//  SERIAL_USB.println("transition check:");
//  snprintf(sentData, sizeof(sentData), "%.2f\t%.2f\t%.2f\t%.2f\n", sign_1*myActuators[idx1].states.q_cur, sign_1*q_transition_1, sign_2*myActuators[idx2].states.q_cur, sign_2*q_transition_2);
//  transmitMsg(sentData);
  return sign_1 * myActuators[idx1].states.q_cur > sign_1 * q_transition_1 && sign_2 * myActuators[idx2].states.q_cur > sign_2 * q_transition_2;
}

void updateSetpoints() {
  if (gaitPhase == 0) { // if in translation phase
//    Serial.println("in translation");
    myActuators[4].states.q_target = gaitSetpoints[swing][gaitPhase];    // new position setpoint
    q_0[0] = myActuators[4].states.q_cur;                                // remember q_0 to calculate early transition
    q_0[1] = myActuators[4].states.q_cur;                                // in translation, there's only one motor to control so the 2nd is redundant
  } else if (gaitPhase == 3) { // if in tilt correction phase; positive roll (stance = 0) -> decrease left (motorIDMap[1]) stroke; positive pitch (stance = 1) -> decrease rear (motorIDMap[3]) stroke
    //      float dq = 315.4*tan(tilt[stance]);  // tilt is positive in the direction of IMU orientation
    //      q_target[stance*2] = q_target[stance*2] - dq/2;
    //      q_target[stance*2 + 1] = q_target[stance*2 + 1] + dq/2;
  } else {
//    Serial.printf("in step or lift: %d\n", gaitPhase);
    myActuators[swing * 2].states.q_target = gaitSetpoints[swing][gaitPhase];
    myActuators[swing * 2 + 1].states.q_target = gaitSetpoints[swing][gaitPhase];
    q_0[0] = myActuators[swing * 2].states.q_cur;
    q_0[1] = myActuators[swing * 2 + 1].states.q_cur;
  }
}

void updateStates() {
  curT = millis();
  uint8_t dt = curT - prevTState;
  if (dt >= DT_STATE) {
    prevTState = curT;
    updateRPY();
  }
}

// updates motor commands on all axes
void updateMotorCommands() {
  curT = millis();
  if (curT - prevTActuation >= DT_ACTUATION) {
    prevTActuation = curT;
    for (uint8_t i = 0; i < N_ACTUATOR; i++) {
      myActuators[i].sendCommand(3, myActuators[i].states.q_target);
      if (i == 4) {
//        Serial.printf("trans mechanism currently at pos_abs: %.3f\n", myActuators[i].states.pos_abs);
      }
    }
  }
}

// enters closed-loop control
void initActuators() {
  for (uint8_t i = 0; i < N_ACTUATOR; i++) {
    snprintf(sentData, sizeof(sentData), "Initializing actuator %d...\n", i + 1);
    transmitMsg(sentData);
    myActuators[i].enable();
    delay(DT_CLOSED_LOOP);
  }

  snprintf(sentData, sizeof(sentData), "Actuators initialized.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void homeMotors() {
  // update Actuator states for the first time
  unsigned long curT = millis();
  while(millis() - curT < DT_FETCH_UPDATE) {
    handleODriveCANMsg();
  }
  
  // home the leg motors
  snprintf(sentData, sizeof(sentData), "Homing all legs...\n\n");
  transmitMsg(sentData);

  for (uint8_t i = 0; i < 4; i++) {   // put leg motors in velocity control with passthrough input
    myActuators[i].enable();
    delay(DT_CLOSED_LOOP);  // entering closed-loop is not instant
    myActuators[i].setControlMode(2);
    myActuators[i].sendCommand(2, HOMING_VEL_LEG);
  }

  // update states while waiting for motors to start moving
  curT = millis();
  while(millis() - curT < DT_FETCH_UPDATE) {
    handleODriveCANMsg();
  }

  bool moving[4] = {true, true, true, true};

  // check leg motors to see if they've hit an endstop
  while (moving[0] || moving[1] || moving[2] || moving[3]) {
    handleODriveCANMsg();    // update states

    for (uint8_t i = 0; i < 4; i++ ) {
      if (moving[i] && fabs(myActuators[i].states.velocity) < 1) {
        myActuators[i].sendCommand(2, 0);
        delay(DT_CLOSED_LOOP);
        myActuators[i].setControlMode(3);
        moving[i] = false;
        myActuators[i].states.pos_home = myActuators[i].states.pos_abs;
        myActuators[i].states.pos_rel = myActuators[i].states.pos_abs - myActuators[i].states.pos_home;
        snprintf(sentData, sizeof(sentData), "Leg actuator %d done retracting.\n******************************************************\n", i + 1);
        transmitMsg(sentData);
      }
    }
  }
  snprintf(sentData, sizeof(sentData), "Finished homing legs.\n\n");
  transmitMsg(sentData);

  curT = millis();
  while(millis() - curT < 1000) {
    handleODriveCANMsg();
  }

  // home the translation motor
  snprintf(sentData, sizeof(sentData), "Homing the translation mechanism...\n");
  transmitMsg(sentData);

  myActuators[4].enable();
  delay(DT_CLOSED_LOOP);
  myActuators[4].setControlMode(2);
  myActuators[4].sendCommand(2, HOMING_VEL_TRANS);

  curT = millis();
  while(millis() - curT < DT_FETCH_UPDATE) {
    handleODriveCANMsg();
  }

  bool moving2 = true;
  while (moving2) {
    handleODriveCANMsg();
    if (moving2 && fabs(myActuators[4].states.velocity) < 0.1) {
      myActuators[4].sendCommand(2, 0);
      moving2 = false;
      delay(DT_CLOSED_LOOP);
      myActuators[4].setControlMode(3);
      myActuators[4].sendCommand(3, myActuators[4].states.pos_abs + HOMING_OFFSET_TRANS);  // move to the zero position

      curT = millis();
      while(millis() - curT < 500 || fabs(myActuators[4].states.velocity) > 0.1) {
        handleODriveCANMsg();
      }
      myActuators[4].states.pos_home = myActuators[4].states.pos_abs;
      myActuators[4].states.pos_rel = myActuators[4].states.pos_abs - myActuators[4].states.pos_home;
    }
  }

  snprintf(sentData, sizeof(sentData), "Finished homing the translation mechanism.\n\n");
  transmitMsg(sentData);

  snprintf(sentData, sizeof(sentData), "Homing the yaw mechanism...\n");
  transmitMsg(sentData);
  myActuators[5].enable();
  delay(DT_CLOSED_LOOP);
  myActuators[5].setControlMode(3);
  myActuators[5].sendCommand(3, Q_YAW_ZERO);  // yaw mechanism's range of motion is < 1 rev so the abs encoder can be used directly
  
  curT = millis();
  while(millis() - curT < 1000 || fabs(myActuators[5].states.velocity) > 0.05) {
    handleODriveCANMsg();
  }

  snprintf(sentData, sizeof(sentData), "Finished homing the yaw mechanism.\n\n");
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "Homing finished.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void updateRPY() {
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
    }
  }
}

void quat2rpy() {
  double sinr_cosp = 2 * (quat[0] * quat[1] + quat[2] * quat[3]);
  double cosr_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
  rpy[0] = atan2(sinr_cosp, cosr_cosp) * 180 / PI;

  double sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
  if (abs(sinp) >= 1) {
    rpy[1] = 90;
    if (sinp < 0) {
      rpy[1] *= -1;
    }
  } else {
    rpy[1] = asin(sinp) * 180 / PI;
  }

  double siny_cosp = 2 * (quat[0] * quat[3] + quat[1] * quat[2]);
  double cosy_cosp = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]);
  rpy[2] = atan2(siny_cosp, cosy_cosp) * 180 / PI;
}

void initIMU(TwoWire *wire) {
  wire->begin();
  if (!bno08x.begin_I2C(0x4A, wire, 0)) {      // begin_I2C(uint8_t i2c_addr = BNO08x_I2CADDR_DEFAULT, TwoWire *wire = &Wire, int32_t sensor_id = 0);
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

  snprintf(sentData, sizeof(sentData), "BNO08x initialized.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

// look for the ODrive CAN heartbeat msg and check errors
void initODrives() {
  for (uint8_t i = 0; i < N_ODRIVE; i++) {
    snprintf(sentData, sizeof(sentData), "Checking ODrive %d...\n", i + 1);
    transmitMsg(sentData);

    for (uint8_t j = 0; j < 2; j++) { // read the heartbeat message for each motor axis
      readHeartbeat(i * 2 + j);
      snprintf(sentData, sizeof(sentData), "Axis %d\ncurrent state: %d\taxis error: %lu\n\n", j, myHeartbeatMsg.currentState, myHeartbeatMsg.axisError);
      transmitMsg(sentData);
      if (myHeartbeatMsg.axisError) {
        snprintf(sentData, sizeof(sentData), "Stopping. Check the axis error.\n");
        transmitMsg(sentData);
        while (true) {} // if there's an axis error, do not proceed
      }
    }
  }

  snprintf(sentData, sizeof(sentData), "ODrives connected.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

// calls the Estop() function on all axes
void stopODrives() {
  for (uint8_t i = 0; i < N_ODRIVE; i++) {
    snprintf(sentData, sizeof(sentData), "Stopping ODrive %d...\n", i + 1);
    transmitMsg(sentData);
    myCAN.Estop(i * 2);     // calling the Estop function will raise the error flag; needs ClearErrors(axis_id) or power cycling
    myCAN.Estop(i * 2 + 1);
  }

  snprintf(sentData, sizeof(sentData), "ODrives stopped.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

// reads the next CAN msg from an ODrive CAN node
// calls the update functions on Actuator instances
void handleODriveCANMsg() {
  if (myCAN.ReadMsg(msg)) {
    axis_id = getAxisID(msg);
    cmd_id = getCmdID(msg);

    if (cmd_id == 0x001) {          // read heartbeat
      myCAN.Heartbeat(myHeartbeatMsg, msg);
      myActuators[axis_id].states.state = myHeartbeatMsg.currentState;
      myActuators[axis_id].states.axisError = myHeartbeatMsg.axisError;

    } else if (cmd_id == 0x009) {   // read position/velocity
      myCAN.GetPositionVelocityResponse(myEncoderMsg, msg);
      myActuators[axis_id].states.pos_abs = myEncoderMsg.posEstimate;
      myActuators[axis_id].states.pos_rel = myActuators[axis_id].states.pos_abs - myActuators[axis_id].states.pos_home;
      myActuators[axis_id].states.q_cur = myActuators[axis_id].states.pos_rel; // q = pos for now
      myActuators[axis_id].states.velocity = myEncoderMsg.velEstimate;

    } else if (cmd_id == 0x014) {   // read motor current
      myCAN.GetIqResponse(myIqMsg, msg);
      myActuators[axis_id].states.torque = myActuators[axis_id].params.torqueConstant * myIqMsg.iqMeasured;
      myActuators[axis_id].states.current = myIqMsg.iqMeasured;
    }
  }
}

// blocking function that reads the heartbeat msg data
// used only in the initODrive() function
void readHeartbeat(uint8_t axis) {
  while (true) {
    if (myCAN.ReadMsg(msg)) {
      axis_id = getAxisID(msg);
      cmd_id = getCmdID(msg);
      if ((axis_id == axis) & (cmd_id == 0x001)) {
        myCAN.Heartbeat(myHeartbeatMsg, msg);
        myActuators[axis_id].states.state = myHeartbeatMsg.currentState;
        myActuators[axis_id].states.axisError = myHeartbeatMsg.axisError;
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

void printStartMsg() {
  while(!SERIAL_USB && millis() < 5000){}  // wait for the Arduino IDE serial monitor or wait up to 5s
  snprintf(sentData, sizeof(sentData), "Teensy started.\n\n");
  transmitMsg(sentData);
}

void transmitData() {
  curT = millis();
  if (curT - prevTPrint >= DT_PRINT) {
    prevTPrint = curT;
    snprintf(sentData, sizeof(sentData), "%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             (float)curT / 1000, myActuators[0].states.q_cur, myActuators[1].states.q_cur, myActuators[2].states.q_cur, myActuators[3].states.q_cur, myActuators[4].states.q_cur, myActuators[5].states.q_cur); // time and joint positions
    transmitMsg(sentData);

    snprintf(sentData, sizeof(sentData), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             myActuators[0].states.q_target, myActuators[1].states.q_target, myActuators[2].states.q_target, myActuators[3].states.q_target, myActuators[4].states.q_target, myActuators[5].states.q_target);   // joint setpoints
    transmitMsg(sentData);

    snprintf(sentData, sizeof(sentData), "%.2f\t%.2f\t%.2f\t%d\t%d\t%lu\n",
             rpy[0], rpy[1], rpy[2], swing, gaitPhase, gaitCycles);                            // roll pitch yaw, gait variables
    transmitMsg(sentData);
  }
}

void transmitMsg(char data[]) {
  strcpy(sentData, data);
  SERIAL_USB.print(sentData);
}
