#include "ODrive.h"
#include "Actuator.h"

// serial communication parameters
#define SERIAL_USB Serial       // Serial (USB), Serial<x> (Hardware serial) e.g. Serial5
#define SERIAL_OD1 Serial2      // hardware serial port for ODrive 1 (1 rear, 2 front)
#define SERIAL_OD2 Serial6      // hardware serial port for ODrive 2 (1 left, 2 right)
#define SERIAL_OD3 Serial7      // hardware serial port for ODrive 3 (1 translate, 2 yaw)
#define BAUD_RATE_SERIAL 230400 // baud rate for USB serial
#define BAUD_RATE_ODRIVE 230400 // baud rate for ODrive
//#define BAUD_RATE_ODRIVE 57600  // baud rate for ODrive
#define DELAY_STD 20            // standard delay in ms between setup function calls
#define N_ODRIVE 3              // number of ODrive motor controllers
#define N_ACTUATOR 6            // number of actuators
#define MAX_DATA_SIZE 128       // max number of chars in sent/received data array

// actuation parameters
#define HOMING_VEL_LEG -5.        // homing velocity for the leg motor in turns/s
#define HOMING_VEL_TRANS 2.       // homing velocity for the translation motor in turns/s
#define HOMING_OFFSET_TRANS -2.1  // distance from the joint limit to the zero position for the translation mechanism in turns

#define Q_ERROR_MAX 0.15          // maximum allowable position error

#define N_DIFF_RING 34.0    // number of teeth on differential ring gear
#define N_DIFF_PINION 11.0  // number of teeth on the drive hub
#define D_LEG_SPUR 17.0     // pitch diameter (in mm) of the leg spur gear
#define D_TRANS_SPUR 19.0   // pitch diameter (in mm) of the translation spur gear
#define N_T_LEG (N_DIFF_PINION/N_DIFF_RING)*(D_LEG_SPUR/2)    // this evaluates to 2.75

// update loop parameters
#define DT_ACTUATION 5      // motor command update period in ms
#define DT_STATE 5          // motor state update period in ms
#define DT_SETPOINT 10      // joint setpoint update period in ms
#define DT_PRINT 50         // data print period in ms

// ODrive and actuator variables
ODrive myODrives[N_ODRIVE] = {ODrive(SERIAL_OD1, BAUD_RATE_ODRIVE, SERIAL_USB),
                              ODrive(SERIAL_OD2, BAUD_RATE_ODRIVE, SERIAL_USB),
                              ODrive(SERIAL_OD3, BAUD_RATE_ODRIVE, SERIAL_USB)};  // array of ODrive objects

Actuator myActuators[N_ACTUATOR] = {Actuator(myODrives[0], 0, SERIAL_USB),
                                    Actuator(myODrives[0], 1, SERIAL_USB),
                                    Actuator(myODrives[1], 0, SERIAL_USB),
                                    Actuator(myODrives[1], 1, SERIAL_USB),
                                    Actuator(myODrives[2], 0, SERIAL_USB),
                                    Actuator(myODrives[2], 1, SERIAL_USB)};

float motorStates[N_ACTUATOR][4] =  {   // array of motor position (turns), velocity (turns/s), torque (Nm), and current (A)
  {0, 0, 0, 0},   // front legs
  {0, 0, 0, 0},   // rear legs
  {0, 0, 0, 0},   // right legs
  {0, 0, 0, 0},   // left legs
  {0, 0, 0, 0},   // translation
  {0, 0, 0, 0}    // yaw
};

uint32_t motorErrors[N_ACTUATOR] = {0, 0, 0, 0, 0, 0};  // motor axis error state reported by ODrive

float q_cur[] = {0, 0, 0, 0, 0, 0};     // current joint position in turns (need to be updated to radians and mm)
float q_target[] = {0, 0, 0, 0, 0, 0};  // target joint position in turns (need to be updated to radians and mm)

// joint, actuation, and gait parameters  
float q_stance = 82;        // target position for lower body stance in turns
float q_stance_offset = 38; // target position offset for upper body stance in turns
float dq_swing_pcnt = 0.5;  // proportion of q_stance (for both lower and upper legs) to retract during swing
float q_trans = 0.7;        // target position for body translation in turns
float q_yaw = 0.05;         // target body yaw for turning step in turns
float max_tilt = 0.04;      // max allowable pitch or roll in radians (not used yet)

// early transition thresholds
// value of 1 means one gait phase is fully completed before next one begins
float q_0[2] = {0, 0};              // initial position of the joint corresponding to the current gait phase
                                    // used for q_leg_0 or q_trans_0 in early transition calculation
float dq_early_step_pcnt = 0.5;     // % of (q_trans_f - q_trans_0) at which step down begins early
float dq_early_lift_pcnt = 1;       // % of (q_leg_f - q_leg_0) in stance at which lift off begins early
float dq_early_correct_pcnt = 0.6;  // % of (q_leg_f - q_leg_0) in swing at which tilt correction begins early
float dq_early_trans_pcnt = 0.8;    // % of (q_leg_f - q_leg_0) in tilt correction at which translation begins early

float gaitSetpoints[2][3] = {           // joint target positions for upper/lower body gait phases
  {-q_trans, q_stance + q_stance_offset, q_stance*(1-dq_swing_pcnt) + q_stance_offset},
  {q_trans, q_stance, q_stance*(1-dq_swing_pcnt)}
};
float gaitPhaseOffset[] = {dq_early_step_pcnt, dq_early_lift_pcnt, dq_early_correct_pcnt, dq_early_trans_pcnt};

float odrivePower[N_ODRIVE][3] = {    // array of ODrive voltage, current, power
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

float systemPower[4] = {0, 0, 0, 0};  // battery voltage (avg of 3 ODrive bus voltage), battery current, battery power, total battery energy used

// communication variables
char receivedData[MAX_DATA_SIZE] = "";
char sentData[MAX_DATA_SIZE] = "";
unsigned long dt;

// time variables
unsigned long curT;                 // current timestamp
unsigned long prevTPrint = 0;       // timestamp at last serial transmission
unsigned long prevTState = 0;       // timestamp at last motor state update
unsigned long prevTSetpoint = 0;    // timestamp at last setpoint update
unsigned long prevTActuation = 0;   // timestamp at last motor command update

// state variables
uint8_t swing = 1;        // 0 if upper swing, 1 if lower swing; swing switches when gaitPhase changes to 2
uint8_t gaitPhase = 1;    // current gait phase; (0 translate) -> (1 step) -> (2 lift) - -> (3 tilt correct); 3 may be skipped if there's no tilt
uint32_t gaitCycles = 0;  // number of completed gait cycles
bool stopSignal = false;

//////////////////////////////////////////////////////////////////////////
void setup() {
  initUSBSerial();
  initODrives();    // check motor/encoder calibration
  homeMotors();     // actuator position after homing is assumed to be the new zero position
  initActuators();  // set zero position and enter closed-loop control

  updateStates();
  updateSetpoints();
  updateMotorCommands();
  transmitData();

//  myActuators[0].setPosition(120);
//  myActuators[1].setPosition(120);
//  delay(1700);
//  myActuators[4].setPosition(0.7);
//  delay(1000);
//  myActuators[2].setPosition(82);
//  myActuators[3].setPosition(82);
//  delay(2000);
}

void loop() {
  while (!stopSignal & (gaitCycles < 20)) {
    updateStates();
    updateGait();
    updateMotorCommands();
    transmitData();
  }
//  
//  while (!stopSignal & (gaitCycles < 20)) {
//    myActuators[0].setPosition(70);
//    myActuators[1].setPosition(70);
//    delay(1000);
//    myActuators[4].setPosition(-0.7);
//    delay(1500);
//    myActuators[0].setPosition(120);
//    myActuators[1].setPosition(120);
//    delay(2000);
//  
//    myActuators[2].setPosition(32);
//    myActuators[3].setPosition(32);
//    delay(1000);
//    myActuators[4].setPosition(0.7);
//    delay(1500);
//    myActuators[2].setPosition(82);
//    myActuators[3].setPosition(82);
//    delay(2000);
//  }
//  stopActuators();
//  Serial.println("End.");
//  delay(10000);
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
    }else if (gaitPhase == 3){
      idx[0] = stance*2;
      idx[1] = stance*2 + 1;
    }else {
      idx[0] = swing*2;       // front or right leg
      idx[1] = swing*2 + 1;   // rear or left leg
    }

//    float tilt[2] = {(rpy[0] - rpy_0[0])*PI/180, (rpy[1] - rpy_0[1])*PI/180};  // roll and pitch in radians

    // if ready for gait phase transition
    if (isReadyForTransition(idx[0], idx[1], gaitPhase)) {
//      SERIAL_USB.println("switching gait phase");
      gaitPhase = (gaitPhase + 1) % 4; // move to next gait phase
      if(gaitPhase == 2 ) {            // if a gait cycle is completed, switch the swing body
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
      
      updateSetpoints();
    }
    snprintf(sentData, sizeof(sentData), "updateGait() took %lu ms\n", (millis()-curT));
    transmitMsg(sentData);
  }
}

bool isReadyForTransition(uint8_t idx1, uint8_t idx2, uint8_t gaitPhase){
  // 1 or -1 to determine direction of inequality condition
  int sign_1 = (q_target[idx1] > q_0[0]) - (q_target[idx1] < q_0[0]);
  int sign_2 = (q_target[idx2] > q_0[1]) - (q_target[idx2] < q_0[1]);
  
  // joint positions past which the next gait phase will begin
  float q_transition_1 = (q_target[idx1] - q_0[0])*gaitPhaseOffset[gaitPhase] + q_0[0] - sign_1*Q_ERROR_MAX;
  float q_transition_2 = (q_target[idx2] - q_0[1])*gaitPhaseOffset[gaitPhase] + q_0[0] - sign_2*Q_ERROR_MAX;

//  SERIAL_USB.println("transition check:");
//  snprintf(sentData, sizeof(sentData), "%.2f\t%.2f\t%.2f\t%.2f\n", sign_1*q_cur[idx1], sign_1*q_transition_1, sign_2*q_cur[idx2], sign_2*q_transition_2);
//  transmitMsg(sentData);
  return sign_1*q_cur[idx1] > sign_1*q_transition_1 && sign_2*q_cur[idx2] > sign_2*q_transition_2;
}

void updateSetpoints() {
  if (gaitPhase == 0) { // if in translation phase
    q_target[4] = gaitSetpoints[swing][gaitPhase];    // new position setpoint
    q_0[0] = q_cur[4];                                // remember q_0 to calculate early transition
    q_0[1] = q_cur[4];                                // in translation, there's only one motor to control so the 2nd is redundant
  } else if (gaitPhase == 3){ // if in tilt correction phase; positive roll (stance = 0) -> decrease left (motorIDMap[1]) stroke; positive pitch (stance = 1) -> decrease rear (motorIDMap[3]) stroke
//      float dq = 315.4*tan(tilt[stance]);  // tilt is positive in the direction of IMU orientation
//      q_target[stance*2] = q_target[stance*2] - dq/2;
//      q_target[stance*2 + 1] = q_target[stance*2 + 1] + dq/2;
  } else {
    q_target[swing*2] = gaitSetpoints[swing][gaitPhase];
    q_target[swing*2 + 1] = gaitSetpoints[swing][gaitPhase];
    q_0[0] = q_cur[swing*2];
    q_0[1] = q_cur[swing*2 + 1];
  }
}

void updateStates() {
  curT = millis();
  uint8_t dt = curT - prevTState;
  if (dt >= DT_STATE) {
    prevTState = curT;

    // update motor position, velocity, torque, current, error
    for (uint8_t i = 0; i < N_ACTUATOR; i++) {  // update motor states
      motorStates[i][0] = myActuators[i].getPosition();
      q_cur[i] = motorStates[i][0];   // for now, just track states in units of motor turns
      motorStates[i][1] = myActuators[i].getVelocity();
      motorStates[i][2] = myActuators[i].getTorque();
      motorStates[i][3] = myActuators[i].getCurrent();
      motorErrors[i] = myActuators[i].getError();

      if (motorErrors[i]) { // if ODrive reports an axis error
        myODrives[i/2].clearErrors();  // try to clear the error and enter closed-loop control again
        if (!myActuators[i].getError() & myODrives[i/2].runClosedLoopControl(0) & myODrives[i/2].runClosedLoopControl(1)) {
          snprintf(sentData, sizeof(sentData), "Actuator %d recovered from error %lu.\n", i+1, motorErrors[i]);
          transmitMsg(sentData);
          motorErrors[i] = 0;
        } else {            // if not recoverable (requires power cycling), stop now
          stopSignal = true;
          snprintf(sentData, sizeof(sentData), "Actuator %d could not recover from error %lu.\n", i+1, motorErrors[i]);
          transmitMsg(sentData);
          return;
        }
      }
    }

    // update controller voltage, current, power
    for (uint8_t i = 0; i < N_ODRIVE; i++) {    // update ODrive power states
      odrivePower[i][0] = myODrives[i].getBusVoltage();
      odrivePower[i][1] = myODrives[i].getBusCurrent();
      odrivePower[i][2] = odrivePower[i][0]*odrivePower[i][1];
    }

    // calculate system voltage, current, power, energy consumption
    systemPower[0] = (odrivePower[0][0] + odrivePower[1][0] + odrivePower[2][0])/3;
    systemPower[1] = odrivePower[0][1] + odrivePower[1][1] + odrivePower[2][1];
    systemPower[2] = odrivePower[0][2] + odrivePower[1][2] + odrivePower[2][2];
    systemPower[3] += systemPower[2]*(dt*0.001);

    // update IMU pose estimation
    // update any other sensors
    snprintf(sentData, sizeof(sentData), "updateStates() took %lu ms\n", (millis()-curT));
    transmitMsg(sentData);
  }
}

void updateMotorCommands() {
  curT = millis();
  if (curT - prevTActuation >= DT_ACTUATION) {
    prevTActuation = curT;
    for (uint8_t i = 0; i < N_ACTUATOR; i++) {
      myActuators[i].setPosition(q_target[i]);   // may be replaced with torque control with MPC
    }
    snprintf(sentData, sizeof(sentData), "updateMotorCommands() took %lu ms\n", (millis()-curT));
    transmitMsg(sentData);
  }
}

void initActuators() {
  for (uint8_t i = 0; i < N_ACTUATOR; i++) {
    snprintf(sentData, sizeof(sentData), "Initializing actuator %d...\n", i+1);
    transmitMsg(sentData);
    myActuators[i].setHome();
    myActuators[i].enable();
    delay(DELAY_STD);
  }

  snprintf(sentData, sizeof(sentData), "Actuators initialized.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
  delay(500);
}

void stopActuators() {
  for (uint8_t i = 0; i < N_ACTUATOR; i++) {
    snprintf(sentData, sizeof(sentData), "Stopping actuator %d...\n", i+1);
    transmitMsg(sentData);
    myActuators[i].disable();
    delay(DELAY_STD);
  }

  snprintf(sentData, sizeof(sentData), "Actuators stopped.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void initUSBSerial() {
  SERIAL_USB.begin(BAUD_RATE_SERIAL);
  delay(100);
  snprintf(sentData, sizeof(sentData), "Teensy started.\n\n");
  transmitMsg(sentData);
}

void initODrives() {
  // check motor/encocder calibration and enter closed-loop control
  for (uint8_t i = 0; i < N_ODRIVE; i++) {
    snprintf(sentData, sizeof(sentData), "Initializing ODrive %d...\n", i+1);
    transmitMsg(sentData);
    myODrives[i].ready();
    delay(DELAY_STD);
  }

  snprintf(sentData, sizeof(sentData), "ODrives initialized.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
  delay(500);
}

void stopODrives() {
  for (uint8_t i = 0; i < N_ODRIVE; i++) {
    snprintf(sentData, sizeof(sentData), "Stopping ODrive %d...\n", i+1);
    transmitMsg(sentData);
    myODrives[i].stop();
    delay(DELAY_STD);
  }

  snprintf(sentData, sizeof(sentData), "ODrives stopped.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void homeMotors() {
  snprintf(sentData, sizeof(sentData), "Homing upper legs...\n");
  transmitMsg(sentData);
  myODrives[0].runHoming(0, HOMING_VEL_LEG, 0);
  myODrives[0].runHoming(1, HOMING_VEL_LEG, 0);

  snprintf(sentData, sizeof(sentData), "Homing lower legs...\n");
  transmitMsg(sentData);
  myODrives[1].runHoming(0, HOMING_VEL_LEG, 0);
  myODrives[1].runHoming(1, HOMING_VEL_LEG, 0);

  snprintf(sentData, sizeof(sentData), "Homing the translation mechanism...\n");
  transmitMsg(sentData);
  myODrives[2].runHoming(0, HOMING_VEL_TRANS, HOMING_OFFSET_TRANS);

  snprintf(sentData, sizeof(sentData), "Homing the yaw mechanism...\n");
  transmitMsg(sentData);
  myODrives[2].setPosition(1, 0.18);  // yaw mechanism's range of motion is < 1 rev so the abs encoder can be used directly

  snprintf(sentData, sizeof(sentData), "Homing finished.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void transmitData() {
  curT = millis();
  if (curT - prevTPrint >= DT_PRINT) {
    prevTPrint = curT;
    snprintf(sentData, sizeof(sentData), "%lu\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t\
            %.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
            curT, q_cur[0], q_cur[1], q_cur[2], q_cur[3], q_cur[4], q_cur[5],   // time and joint positions
            q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
    transmitMsg(sentData);
    snprintf(sentData, sizeof(sentData), "%d\t%d\t%lu\t\
            %.2f\t%.2f\t%.2f\t%.2f\n", 
            swing, gaitPhase, gaitCycles,
            systemPower[0], systemPower[1], systemPower[2], systemPower[3]);    // system voltage, current, power, energy consumption
    transmitMsg(sentData);
  }
}

void transmitMsg(char data[]) {
  strcpy(sentData, data);
  SERIAL_USB.print(sentData);
}
