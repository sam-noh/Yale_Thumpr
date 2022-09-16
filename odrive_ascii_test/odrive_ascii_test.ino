#include "ODrive.h"
//#include "Actuator.h"

// serial communication parameters
#define SERIAL_USB Serial       // Serial (USB), Serial<x> (Hardware serial) e.g. Serial5
#define SERIAL_OD1 Serial2      // hardware serial port for ODrive 1 (1 rear, 2 front)
#define SERIAL_OD2 Serial6      // hardware serial port for ODrive 2 (1 left, 2 right)
#define SERIAL_OD3 Serial7      // hardware serial port for ODrive 3 (1 translate, 2 yaw)
#define N_ODRIVE 3              // number of ODrive motor controllers
#define BAUD_RATE_SERIAL 230400 // baud rate for USB serial
#define BAUD_RATE_ODRIVE 115200 // baud rate for ODrive

// Ethernet communication parameters
#define PORT 1178               // Ethernet connection port
#define MAX_DATA_SIZE 128       // max number of bytes for a data array
#define DELAY 100               // used in sendHTTPRequest

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
#define Q_LEG_MIN 10.       // min prismatic leg extension in mm
#define Q_LEG_MAX 450.      // max prismatic leg extension in mm
#define Q_LEG_OFFSET 52.0   // additional leg extension for upper prismatic legs in mm
#define Q_TRANS_MAX 140.    // max body translation from midpoint (zero position) in mm 
#define Q_YAW_MAX 0.15      // max body yaw from sagittal plane (zero position) in radians
#define MAX_TILT 0.04       // max allowable pitch or roll in radians
#define Q_DOT_LEG_MAX 300.  // max prismatic leg speed in mm/s
#define Q_DOT_TRANS_MAX 50. // max body translation speed in mm/s
#define Q_DOT_YAW_MAX 0.52  // max body yaw speed in rad/s
#define E_LINEAR 5          // allowable positional error in mm
#define E_ANGULAR 0.015     // allowable positional error in radians

#define Q_LEG_STANCE 300.   // prismatic leg stance setpoint in mm
#define Q_LEG_SWING 250.    // prismatic leg swing setpoint in mm
#define Q_TRANS 70.         // body translation setpoint in mm 

#define DQ_EARLY_STEP 0.2   // distance to Q_TRANS (in percentage) at which step down begins early
#define DQ_EARLY_LIFT 0.    // distance to Q_LEG_STANCE (in percentage) at which lift off begins early
#define DQ_EARLY_CORRECT 0.4// distance to Q_LEG_SWING (in percentage) at which tilt correct begins early
#define DQ_EARLY_TRANS 0.2  // distance to tilt correction setpoint at which translation begins early

// ODrive variables
HardwareSerial SERIAL_OD[N_ODRIVE] = {SERIAL_OD1, SERIAL_OD2, SERIAL_OD3};                  // array of serial ports for ODrive motor controllers
ODrive myODrives[N_ODRIVE] = {ODrive(SERIAL_USB, SERIAL_OD1, BAUD_RATE_ODRIVE),
                              ODrive(SERIAL_USB, SERIAL_OD2, BAUD_RATE_ODRIVE),
                              ODrive(SERIAL_USB, SERIAL_OD3, BAUD_RATE_ODRIVE)};  // array of ODrive objects

// communication variables
char receivedData[MAX_DATA_SIZE] = "";
char sentData[MAX_DATA_SIZE] = "";
unsigned long dt;
unsigned long t_start;

//////////////////////////////////////////////////////////////////////////
void setup() {
  SERIAL_USB.begin(BAUD_RATE_SERIAL);
  delay(100);
  snprintf(sentData, sizeof(sentData), "Teensy started.\n\n");
  transmitMsg(sentData);

  initODrives();
  homeMotors();

//  snprintf(sentData, sizeof(sentData), "%f %f\n", myODrives[0].getPosition(0), myODrives[0].getPosition(1));
//  transmitMsg(sentData);

}

void loop() {
  
}

void initODrives() {
  // check motor/encocder calibration and enter closed-loop control
  for (uint8_t i = 0; i < N_ODRIVE; i++) {
    snprintf(sentData, sizeof(sentData), "Initializing ODrive %d...\n", i+1);
    transmitMsg(sentData);
    myODrives[i].ready();
    SERIAL_USB.println("---------------------------------------------\n");
    delay(200);
  }
}

void stopODrives() {
  for (uint8_t i = 0; i < N_ODRIVE; i++) {
    snprintf(sentData, sizeof(sentData), "Stopping ODrive %d...\n", i+1);
    transmitMsg(sentData);
    myODrives[i].stop();
    SERIAL_USB.println("---------------------------------------------\n");
    delay(200);
  }
}

void homeMotors() {
  transmitMsg("Homing upper legs...\n");
  myODrives[0].runHoming(0, -5, 5);
  myODrives[0].runHoming(1, -5, 5);
  delay(200);

  transmitMsg("Homing lower legs...\n");
  myODrives[1].runHoming(0, -5, 5);
  myODrives[1].runHoming(1, -5, 5);
  delay(200);

  transmitMsg("Homing the translation mechanism...\n");
  myODrives[2].runHoming(0, 2, -2.1);
  delay(200);

  transmitMsg("Homing the yaw mechanism...\n");
  myODrives[2].setPosition(1, 0.18);
  delay(500);
  transmitMsg("Homing finished...\n---------------------------------------------\n");
}

void printAndSetParams() {
  snprintf(sentData, sizeof(sentData), "position 1: %f\n", myODrives[0].getPosition(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "position 2: %f\n", myODrives[0].getPosition(1));
  transmitMsg(sentData);

  snprintf(sentData, sizeof(sentData), "velocity 1: %f\n", myODrives[0].getVelocity(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "velocity 2: %f\n", myODrives[0].getVelocity(1));
  transmitMsg(sentData);

  snprintf(sentData, sizeof(sentData), "torque 1: %f\n", myODrives[0].getTorque(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "torque 2: %f\n", myODrives[0].getTorque(1));
  transmitMsg(sentData);

  snprintf(sentData, sizeof(sentData), "state 1: %d\n", myODrives[0].getCurrentState(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "state 2: %d\n", myODrives[0].getCurrentState(1));
  transmitMsg(sentData);

  snprintf(sentData, sizeof(sentData), "error 1: %lu\n", myODrives[0].getAxisError(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "error 2: %lu\n", myODrives[0].getAxisError(1));
  transmitMsg(sentData);

  myODrives[0].setControlMode(0, 3);
  myODrives[0].setControlMode(1, 3);

  snprintf(sentData, sizeof(sentData), "control mode 1: %d\n", myODrives[0].getControlMode(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "control mode 2: %d\n", myODrives[0].getControlMode(1));
  transmitMsg(sentData);

  myODrives[0].setInputMode(0, 5);
  myODrives[0].setInputMode(1, 5);

  snprintf(sentData, sizeof(sentData), "input mode 1: %d\n", myODrives[0].getInputMode(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "input mode 2: %d\n", myODrives[0].getInputMode(1));
  transmitMsg(sentData);

  myODrives[0].setCurrentLim(0, 40);
  myODrives[0].setCurrentLim(1, 40);

  snprintf(sentData, sizeof(sentData), "current limit 1: %f\n", myODrives[0].getCurrentLim(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "current limit 2: %f\n", myODrives[0].getCurrentLim(1));
  transmitMsg(sentData);

  myODrives[0].setVelLim(0, 90);
  myODrives[0].setVelLim(1, 90);

  snprintf(sentData, sizeof(sentData), "velocity limit 1: %f\n", myODrives[0].getVelLim(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "velocity  limit 2: %f\n", myODrives[0].getVelLim(1));
  transmitMsg(sentData);

  myODrives[0].setPosGain(0, 20);
  myODrives[0].setPosGain(1, 20);

  snprintf(sentData, sizeof(sentData), "position gain 1: %f\n", myODrives[0].getPosGain(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "position gain 2: %f\n", myODrives[0].getPosGain(1));
  transmitMsg(sentData);

  myODrives[0].setVelGain(0, 0.07);
  myODrives[0].setVelGain(1, 0.07);

  snprintf(sentData, sizeof(sentData), "velocity gain 1: %f\n", myODrives[0].getVelGain(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "velocity gain 2: %f\n", myODrives[0].getVelGain(1));
  transmitMsg(sentData);

  myODrives[0].setVelIntGain(0, 0.02);
  myODrives[0].setVelIntGain(1, 0.02);

  snprintf(sentData, sizeof(sentData), "velocity int gain 1: %f\n", myODrives[0].getVelIntGain(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "velocity int gain 2: %f\n", myODrives[0].getVelIntGain(1));
  transmitMsg(sentData);

  myODrives[0].setTrapAccelLim(0, 25);
  myODrives[0].setTrapAccelLim(1, 25);

  snprintf(sentData, sizeof(sentData), "trap accel limit 1: %f\n", myODrives[0].getTrapAccelLim(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "trap accel limit 2: %f\n", myODrives[0].getTrapAccelLim(1));
  transmitMsg(sentData);

  myODrives[0].setTrapDecelLim(0, 25);
  myODrives[0].setTrapDecelLim(1, 25);

  snprintf(sentData, sizeof(sentData), "trap decel limit 1: %f\n", myODrives[0].getTrapDecelLim(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "trap decel limit 2: %f\n", myODrives[0].getTrapDecelLim(1));
  transmitMsg(sentData);

  myODrives[0].setTrapVelLim(0, 69);
  myODrives[0].setTrapVelLim(1, 69);

  snprintf(sentData, sizeof(sentData), "trap vel limit 1: %f\n", myODrives[0].getTrapVelLim(0));
  transmitMsg(sentData);
  snprintf(sentData, sizeof(sentData), "trap vel limit 2: %f\n", myODrives[0].getTrapVelLim(1));
  transmitMsg(sentData);
}

void transmitMsg(char data[]) {
  strcpy(sentData, data);
  SERIAL_USB.print(sentData);
}
