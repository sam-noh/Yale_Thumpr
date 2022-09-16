#include "ODrive.h"
#include "Actuator.h"

// serial communication parameters
#define SERIAL_USB Serial       // Serial (USB), Serial<x> (Hardware serial) e.g. Serial5
#define SERIAL_OD1 Serial2      // hardware serial port for ODrive 1 (1 rear, 2 front)
#define SERIAL_OD2 Serial6      // hardware serial port for ODrive 2 (1 left, 2 right)
#define SERIAL_OD3 Serial7      // hardware serial port for ODrive 3 (1 translate, 2 yaw)
#define BAUD_RATE_SERIAL 230400 // baud rate for USB serial
#define BAUD_RATE_ODRIVE 57600 // baud rate for ODrive
#define N_ODRIVE 3              // number of ODrive motor controllers
#define N_ACTUATOR 6            // number of actuators
#define MAX_DATA_SIZE 128       // max number of chars in sent/received data array

// update loop parameters
#define DT_ACTUATION 2      // motor command update period in ms
#define DT_STATE 2          // motor state update period in ms
#define DT_SETPOINT 50      // joint setpoint update period in ms
#define DT_SENSOR 50        // sensor reading period in ms
#define DT_PRINT 100        // data print period in ms

// transmission parameters
#define N_DIFF_RING 34.0    // number of teeth on differential ring gear
#define N_DIFF_PINION 11.0  // number of teeth on the drive hub
#define D_LEG_SPUR 17.0     // pitch diameter (in mm) of the leg spur gear
#define D_TRANS_SPUR 19.0   // pitch diameter (in mm) of the translation spur gear
#define N_T_LEG (N_DIFF_PINION/N_DIFF_RING)*(D_LEG_SPUR/2)    // this evaluates to 2.75

// joint, actuation, and gait parameters  
#define Q_SWING -50.        // leg retraction amount during swing in turns
#define Q_STANCE 82.        // motor position at lower body stance in turns
#define Q_STANCE_OFFSET 38. // motor position offset for upper body stance in turns
#define Q_TRANS 0.7         // motor position at max translation in turns
#define Q_YAW_MAX 0.15      // max body yaw from sagittal plane (zero position) in radians
#define MAX_TILT 0.04       // max allowable pitch or roll in radians

#define DQ_EARLY_STEP 0.2   // distance to Q_TRANS (in percentage) at which step down begins early
#define DQ_EARLY_LIFT 0.    // distance to Q_LEG_STANCE (in percentage) at which lift off begins early
#define DQ_EARLY_CORRECT 0.4// distance to Q_LEG_SWING (in percentage) at which tilt correct begins early
#define DQ_EARLY_TRANS 0.2  // distance to tilt correction setpoint at which translation begins early

// ODrive and actuator variables
ODrive myODrives[N_ODRIVE] = {ODrive(SERIAL_OD1, BAUD_RATE_ODRIVE, SERIAL_USB),
                              ODrive(SERIAL_OD2, BAUD_RATE_ODRIVE, SERIAL_USB),
                              ODrive(SERIAL_OD3, BAUD_RATE_ODRIVE, SERIAL_USB)};  // array of ODrive objects

float odrivePower[N_ODRIVE][3] = {    // array of ODrive voltage, current, power
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

float systemPower[4] = {0, 0, 0, 0};  // battery voltage (avg of 3 ODrive bus voltage), battery current, battery power, total battery energy used

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
float gaitPhaseOffset[] = {DQ_EARLY_STEP, DQ_EARLY_LIFT, DQ_EARLY_CORRECT, DQ_EARLY_TRANS};

// communication variables
char receivedData[MAX_DATA_SIZE] = "";
char sentData[MAX_DATA_SIZE] = "";
unsigned long dt;
unsigned long t_start;

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
//    updateGait();
    updateStates();
    updateMotorCommands();
    transmitData();
  }
  
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
  stopActuators();
  Serial.println("End.");
  delay(10000);
}

void updateGait() {
  curT = millis();
  if (curT - prevTSetpoint >= DT_SETPOINT) {
    prevTSetpoint = curT;

    // identify which joint positions to check
    uint8_t motor_id[2];    // two motor indices corresponding to the order in motorPin[][]
    uint8_t stance = (swing + 1) % 2;
    if (gaitPhase == 0) {
      motor_id[0] = 4;
      motor_id[1] = 4;
    }else if (gaitPhase == 3){
      motor_id[0] = stance*2;
      motor_id[1] = stance*2 + 1;
    }else {
      motor_id[0] = swing*2
      motor_id[1] = swing*2 + 1;
    }

    uint8_t idx1 = motor_id[0];
    uint8_t idx2 = motor_id[1];
    double tilt[2] = {(rpy[0] - rpy_0[0])*PI/180, (rpy[1] - rpy_0[1])*PI/180};  // roll and pitch in radians

    // check if ready for gait phase transition
    if (isReadyForTransition(idx1, idx2, gaitPhase)) {

      gaitPhase = (gaitPhase + 1) % 4; // move to next gait phase
      if(gaitPhase == 2 ) {            // if a gait cycle is completed, switch the swing body
        swing = (swing + 1) % 2;
        stance = (swing + 1) % 2;
        gaitCycles++;
      }

      if (gaitPhase == 3 && abs(tilt[stance]) < MAX_TILT) {    // if no tilt to correct, skip tilt correct phase
        gaitPhase = (gaitPhase + 1) % 4;
      }
      
      // set new target joint positions
      double cmds[4];
      cmds[0] = 1;
      if (gaitPhase == 0) { // if in body translation phase
        cmds[1] = 2;
        cmds[2] = gaitSetpoints[swing][gaitPhase];
        cmds[3] = 0;
      } else if (gaitPhase == 3){ // if in tilt correction phase; positive roll (stance = 0) -> decrease left (motorIDMap[1]) stroke; positive pitch (stance = 1) -> decrease rear (motorIDMap[3]) stroke
        cmds[1] = stance;
        double dq = 315.4*tan(tilt[stance]);  // tilt is positive in the direction of IMU orientation
        cmds[2] = q_target[motorIDMap[stance*2]] - dq/2;
        cmds[3] = q_target[motorIDMap[stance*2+1]] + dq/2;
      } else {
        cmds[1] = swing;
        cmds[2] = gaitSetpoints[swing][gaitPhase];
        cmds[3] = gaitSetpoints[swing][gaitPhase];
      }
      setJointPosition(cmds);
    }
  }
}

bool isReadyForTransition(uint8_t idx1, uint8_t idx2, uint8_t gaitPhase){
  if (gaitPhase == 0){
    int sign = (q_target[idx1] > 0) - (q_target[idx1] < 0);
    double q_min = q_target[idx1] - sign*(gaitPhaseOffset[gaitPhase] + e_p_max[idx1]);
    if (q_target[idx1] > 0 && q_cur[idx1] > q_min || q_target[idx1] < 0 && q_cur[idx1] < q_min){
      return true;
    }
    return false;

  } else{
    double e_p_1 = q_target[idx1] - q_cur[idx1];
    double e_p_2 = q_target[idx2] - q_cur[idx2];
    double margin_1 = gaitPhaseOffset[gaitPhase] + e_p_max[idx1];
    double margin_2 = gaitPhaseOffset[gaitPhase] + e_p_max[idx2];
    if (abs(e_p_1) < abs(margin_1) && abs(e_p_2) < abs(margin_2)){
      return true;
    }
    return false;
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
      motorStates[i][1] = myActuators[i].getVelocity();
      motorStates[i][2] = myActuators[i].getTorque();
      motorStates[i][3] = myActuators[i].getCurrent();
      motorErrors[i] = myActuators[i].getError();

      if (motorErrors[i]) {
        stopSignal = true;
        return;
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
    
  }  
}

void updateMotorCommands() {
  curT = millis();
  if (curT - prevTActuation >= DT_ACTUATION) {
    prevTActuation = curT;
    for (uint8_t i = 0; i < N_ACTUATOR; i++) {
      myActuators[i].setPosition(q_target[i]);   // may be replaced with torque control with MPC
    }
  }
}

void initActuators() {
  for (uint8_t i = 0; i < N_ACTUATOR; i++) {
    snprintf(sentData, sizeof(sentData), "Initializing actuator %d...\n", i+1);
    transmitMsg(sentData);
    myActuators[i].setHome();
    myActuators[i].enable();
    delay(50);
  }

  snprintf(sentData, sizeof(sentData), "Actuators initialized.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void stopActuators() {
  for (uint8_t i = 0; i < N_ACTUATOR; i++) {
    snprintf(sentData, sizeof(sentData), "Stopping actuator %d...\n", i+1);
    transmitMsg(sentData);
    myActuators[i].disable();
    delay(50);
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
  }

  snprintf(sentData, sizeof(sentData), "ODrives initialized.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
  delay(500);
}

// 
void stopODrives() {
  for (uint8_t i = 0; i < N_ODRIVE; i++) {
    snprintf(sentData, sizeof(sentData), "Stopping ODrive %d...\n", i+1);
    transmitMsg(sentData);
    myODrives[i].stop();
    delay(50);
  }

  snprintf(sentData, sizeof(sentData), "ODrives stopped.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void homeMotors() {
  snprintf(sentData, sizeof(sentData), "Homing upper legs...\n");
  transmitMsg(sentData);
  myODrives[0].runHoming(0, -5, 0);
  myODrives[0].runHoming(1, -5, 0);
  delay(50);

  snprintf(sentData, sizeof(sentData), "Homing lower legs...\n");
  transmitMsg(sentData);
  myODrives[1].runHoming(0, -5, 0);
  myODrives[1].runHoming(1, -5, 0);
  delay(50);

  snprintf(sentData, sizeof(sentData), "Homing the translation mechanism...\n");
  transmitMsg(sentData);
  myODrives[2].runHoming(0, 2, -2.1);
  delay(50);

  snprintf(sentData, sizeof(sentData), "Homing the yaw mechanism...\n");
  transmitMsg(sentData);
  myODrives[2].setPosition(1, 0.18);
  delay(50);

  snprintf(sentData, sizeof(sentData), "Homing finished.\n---------------------------------------------\n\n");
  transmitMsg(sentData);
}

void transmitData() {
  curT = millis();
  if (curT - prevTPrint >= DT_PRINT) {
    prevTPrint = curT;
    snprintf(sentData, sizeof(sentData), "%lu\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%d\t%d\t%lu\t%.2f\t%.2f\t%.2f\t%.2f\n", 
            curT, q_cur[0], q_cur[1], q_cur[2], q_cur[3], q_cur[4], q_cur[5], swing, gaitPhase, gaitCycles, 
            systemPower[0], systemPower[1], systemPower[2], systemPower[3]);
    transmitMsg(sentData);
  }
}

void transmitMsg(char data[]) {
  strcpy(sentData, data);
  SERIAL_USB.print(sentData);
}
