#ifndef MCU_UTIL_H
#define MCU_UTIL_H

#include <TimeLib.h>
#include <SD.h>

//////////////////////////////////////////////////////////////////////////////////////
// macros and constants

// AMT102-V incremental encoder pins
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

// additional sensor pins
#define ACS711EX_OUT 27     // Vout pin for ACS711EX current sensor
#define ACS711EX_FAULT 28   // fault pin for ACS711EX current sensor
#define BNO08X_RESET -1     // reset pin for BNO08X IMU; not needed for I2C
#define XOUT 39             // analog input pin for joystick x-axis
#define YOUT 40             // analog input pin for joystick y-axis
#define SEL 41              // digital pin for joystick button
#define LED 13              // built-in LED pin

#define SERIAL_USB Serial             // Serial (USB), Serial<x> (Hardware serial) e.g. Serial5
#define WIRE_IMU Wire                 // Using I2C bus 1: SCL, SDA
#define MAX_DATA_SIZE 128             // max number of chars in sent/received data array

#define PI 3.1415926535897932384626433832795
#define TIME_HEADER  "T"            // Header tag for serial time sync message

// control loop periods
const uint8_t k_dtPrint = 50;              // data print period in ms
const uint8_t k_dtStateUpdate = 5;         // state update period in ms
const uint8_t k_dtSetpointUpdate = 10;     // joint setpoint update period in ms
const uint8_t k_dtMotorCmd = 10;           // motor command update period in ms
const uint8_t k_dtODriveResponse = 50;     // wait time for ODrive/motor response in ms

//////////////////////////////////////////////////////////////////////////////////////
// global variables

// time variables
extern uint32_t t_last_print;              // timestamp at last serial print
extern uint32_t t_last_state_update;       // timestamp at last state update
extern uint32_t t_last_setpoint_update;    // timestamp at last setpoint update
extern uint32_t t_last_motor_cmd;          // timestamp at last motor command update
extern uint32_t t_last_encoder_update;     // timestamp at last encoder reading

// communication variables
extern char received_data[MAX_DATA_SIZE];
extern char sent_data[MAX_DATA_SIZE];
extern const int chip_select;
extern File data_file;
extern char data_file_name[128];

// state variable
extern bool stop_signal;

//////////////////////////////////////////////////////////////////////////////////////
// global functions

// initialize Teensy 4.1 digital pins, RTC and etc.
void initTeensy();

// return Teensy time
time_t getTeensy3Time();

// process timestamp from USB serial during Teensy code upload
unsigned long processSyncMessage();

// initialize SD card and set file name
void initSDCard();

// write data to the onboard microSD card
void writeToCard(char data[]);

// write data to the Serial output
void transmitMsg();

#endif