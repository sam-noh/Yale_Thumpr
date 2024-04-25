#ifndef MCU_UTIL_H
#define MCU_UTIL_H

#include <TimeLib.h>
#include <SD.h>
#include <vector>

//////////////////////////////////////////////////////////////////////////////////////
// macros and constants

#define ENABLE_JETSON
#define ENABLE_2ND_TEENSY
#define ENABLE_SD_CARD
// #define ENABLE_IMU

// #define DEBUG_TIMER
#define DEBUG_ACTUATOR_POSITION
#define DEBUG_ACTUATOR_SETPOINT
// #define DEBUG_ACTUATOR_TORQUE
// #define DEBUG_ACTUATOR_TEMP
// #define DEBUG_LEG_POSITION
// #define DEBUG_LEG_VELOCITY
// #define DEBUG_LEG_ACCELERATION
// #define DEBUG_CONTACT
#define DEBUG_POWER
#define DEBUG_RPY
// #define DEBUG_OMEGA
#define DEBUG_TRAJECTORY
#define DEBUG_GAIT

// AMT102-V incremental encoder pins
#define ENC_1_A 1  // medial front right A
#define ENC_1_B 2  // medial front right B
#define ENC_2_A 3  // medial front left A
#define ENC_2_B 4  // medial front left B
#define ENC_3_A 5  // medial rear right A
#define ENC_3_B 6  // medial rear right B
#define ENC_4_A 7   // medial rear left A
#define ENC_4_B 8  // medial rear left B
#define ENC_5_A 9  // lateral front right A
#define ENC_5_B 10  // lateral front right B
#define ENC_6_A 11  // lateral front left A
#define ENC_6_B 12  // lateral front left B
#define ENC_7_A 24  // lateral rear right A
#define ENC_7_B 25  // lateral rear right B
#define ENC_8_A 26  // lateral rear left A
#define ENC_8_B 27  // lateral rear left B

// additional sensor pins
#define ACS711EX_OUT 20     // Vout pin for ACS711EX current sensor
#define ACS711EX_FAULT 21   // fault pin for ACS711EX current sensor
#define BNO08X_RESET -1     // reset pin for BNO08X IMU; not needed for I2C
#define XOUT 39             // analog input pin for joystick x-axis
#define YOUT 40             // analog input pin for joystick y-axis
#define SEL 41              // digital pin for joystick button
#define LED 13              // built-in LED pin

#define SERIAL_USB Serial               // Serial (USB), Serial<x> (Hardware serial) e.g. Serial5
#define SERIAL_JETSON Serial3           // hardware serial bus with the Jetson
#define SERIAL_BAUDRATE_JETSON 9600     // Jetson serial connection baudrate
#define SERIAL_TEENSY Serial7           // hardware serial bus with a second Teensy 4.1
#define SERIAL_BAUDRATE_TEENSY 500000   // Teens 4.1 serial connection baudrate
#define WIRE_IMU Wire                   // Using I2C bus 1: SCL, SDA
#define MAX_DATA_SIZE 128               // max number of chars in sent/received data array

#define EPS 1e-2                                // epsilon
#define PI 3.1415926535897932384626433832795    // pi
#define RAD2DEG 180/PI                          // radians to degrees conversion factor
#define DEG2RAD PI/180                          // degrees to radians conversion factor
#define TIME_HEADER  "T"                        // Header tag for serial time sync message

// control loop periods
const uint8_t k_dtPrint = 10;               // data print period in ms
const uint8_t k_dtSetpointUpdate = 2;       // joint setpoint update period in ms
const uint8_t k_dtMotorLimitsUpdate = 50;   // motor controller limits update period in ms
const uint8_t k_dtMotorCmdUpdate = 20;      // motor command update period in ms;
const uint8_t k_dtTrajectoryUpdate = 10;    // robot trajectory update period in ms;

//////////////////////////////////////////////////////////////////////////////////////
// global variables

// time variables
extern uint32_t t_last_print;               // timestamp at last serial print
extern uint32_t t_last_setpoint_update;     // timestamp at last setpoint update
extern uint32_t t_last_motor_limits_update; // timestamp at last motor controller limits update
extern uint32_t t_last_motor_cmd_update;    // timestamp at last motor command update
extern uint32_t t_last_CAN_msg;             // timestamp at last CAN msg
extern uint32_t t_last_Jetson;              // timestamp at last Jetson read

// communication variables
extern String received_data;
extern float input_x, input_y, input_height, input_swing;
extern std::vector<float> input_rpy;
extern std::vector<float> input_omega;

extern char sent_data[MAX_DATA_SIZE];
extern const int chip_select;
extern File data_file;
extern char data_file_name[128];

// state variable
extern bool stop_signal;
extern bool isSDInit;      // true if SD.open() has been attempted; safeguards against writing attempts before opening the file during setup

//////////////////////////////////////////////////////////////////////////////////////
// global functions

// initialize Teensy 4.1 digital pins, RTC and etc.
void initTeensy();

// start serial connections with other devices such as Jetson, a 2nd Teensy
void initSerial();

// return Teensy time
time_t getTeensy3Time();

// process timestamp from USB serial during Teensy code upload
unsigned long processSyncMessage();

// initialize the onboard RTC
void initRTC();

// initialize SD card and set file name
void initSDCard();

// open a data file on the SD card for writing
void openDataFile();

// write data to the onboard microSD card
void writeToCard(char data[]);

// stop writing to the file and save
void closeDataFile();

// write data to the Serial output
void writeToSerial();

// returns a token between the given separators at the specified index
String getValue(String data, char separator, int index);

// retrieves only the latest string of commands sent from the Jetson
String getLast(String data);

// extracts the robot trajectory commands from the Jetson serial data
void parseJetsonSerial();

// extracts the IMU data sent from a 2nd Teensy over serial
void parseTeensySerial();

// write telemetry data to serial and SD card, if enabled
void sendTelemetry();

#endif