#include "mcu_util.h"
#include "../motor_control/motor_control.h"
#include "../state_estimation/state_estimation.h"
#include "../locomotion/locomotion.h"

// time variables
uint32_t t_last_print = 0;                // timestamp at last serial print
uint32_t t_last_setpoint_update = 0;      // timestamp at last setpoint update
uint32_t t_last_motor_limits_update = 0;  // timestamp at last motor controller limits update
uint32_t t_last_motor_cmd_update = 0;     // timestamp at last motor command update
uint32_t t_last_CAN_msg = 0;              // timestamp at last CAN msg
uint32_t t_last_Jetson = 0;               // timestamp at last Jetson read

String received_data;
float input_x = 0;
float input_y = 0;
float input_height = 0;
float input_swing = 0;
std::vector<float> input_rpy{0, 0, 0};
std::vector<float> input_omega{0, 0, 0};

char sent_data[MAX_DATA_SIZE] = "";
const int chip_select = BUILTIN_SDCARD;
File data_file;  // SD card file
char data_file_name[128] = "";

bool stop_signal = false;
bool isSDInit = false;

// initialize Teensy 4.1 digital pins, RTC and etc.
void initTeensy() {
  delay(100);   // short delay after the start to ensure the first serial print is not missed over USB
  SERIAL_USB.println("Teensy started.\n");

  // add digital pin initialization here
  pinMode(LED, OUTPUT);

  initSerial();                   // start serial connections with other devices such as Jetson, a 2nd Teensy
  initRTC();                      // initialize the onboard RTC (requires coin cell battery)
  bno08x_imu.initIMU(&WIRE_IMU);  // initialize I2C IMU sensor (if enabled)
  initSDCard();                   // initialize the microSD card for writing (if enabled)
}

// start serial connections with other devices such as Jetson, a 2nd Teensy
void initSerial() {

  // start Serial connection with the 2nd Teensy 4.1
  #ifdef ENABLE_2ND_TEENSY
  SERIAL_TEENSY.begin(SERIAL_BAUDRATE_TEENSY);
  SERIAL_USB.println("Serial connection with Teensy 4.1 started.\n");
  #endif

  // start Serial connection with the Jetson
  #ifdef ENABLE_JETSON
  SERIAL_JETSON.begin(SERIAL_BAUDRATE_JETSON);
  SERIAL_USB.println("Serial connection with Jetson started.\n");
  #endif
}

// return Teensy time
time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

// process timestamp from USB serial during Teensy code upload
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(SERIAL_USB.find(TIME_HEADER)) {
     pctime = SERIAL_USB.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}

// initialize the onboard RTC
void initRTC() {
  setSyncProvider(getTeensy3Time);    // time sync for Teensy's internal RTC

  if (timeStatus()!= timeSet) {
    digitalWrite(LED, HIGH);
    SERIAL_USB.println("Unable to sync with the RTC.\n");
    while(1) {};
  } else {
    SERIAL_USB.println("RTC has set the system time.\n");
  }

  time_t t = processSyncMessage();
  if (t != 0) {
    Teensy3Clock.set(t); // set the RTC
    setTime(t);
  }
}

// initialize microSD card and set file name
// TODO: try waiting a bit after bootup to see if the SD card is found and can be initialized
void initSDCard() {
  #ifdef ENABLE_SD_CARD

  uint32_t t_current = millis();
  bool success = false;

  snprintf(sent_data, sizeof(sent_data), "Waiting for SD card.\n");
  writeToSerial();
  while (!success && millis() - t_current < 1000) {
    if (SD.begin(chip_select)) {
      success = true;
    }
  }

  if (!success) {
    digitalWrite(LED, HIGH);
    SERIAL_USB.println("SD card failed or not present.\n");
    while (1) {}
  } else {
    SERIAL_USB.println("SD card initialized.\n");
  }
  
  #endif
}

// open a data file on the SD card for writing
void openDataFile() {
  #ifdef ENABLE_SD_CARD

  String file_name = String(year()) + '_' +
                     String(month()) + '_' +
                     String(day()) + '_' +
                     String(hour()) + '_' +
                     String(minute()) + '_' +
                     String(second()) + ".txt";

  // name the data file with the timestamp
  file_name.toCharArray(data_file_name, sizeof(data_file_name));
  data_file = SD.open(data_file_name, FILE_WRITE);
  isSDInit = true;
  if (data_file) {
    SERIAL_USB.printf("Data being saved to: %s\n\n", data_file_name);
  } else {
    digitalWrite(LED, HIGH);
    SERIAL_USB.println("Could not open the data file for writing.\n");
    while (1) {}
  }

  #endif
}

// write data to the onboard microSD card
void writeToCard(char data[]) {
  #ifdef ENABLE_SD_CARD

  if (isSDInit) {               // don't attempt to write before openDataFile() has been called
    if (data_file) {            // if false, that means the SD file opened successfully but failed afterwards
      data_file.print(data);
    } else {
      digitalWrite(LED, HIGH);
      SERIAL_USB.println("Error opening the data file.\n");
      stop_signal = true;
    }
  }
  
  #endif
}

// stop writing to the file and save
void closeDataFile() {
  #ifdef ENABLE_SD_CARD

  if (isSDInit) {           // if false, openDataFile() was not called yet
    if (data_file) {        // if false, the SD file opened successfully but failed afterwards
      data_file.close();
      SERIAL_USB.println("Saved the data file");
    } else {
      SERIAL_USB.println("Failed to save the data file");
    }
  }

  #endif
}

// write data to the Serial output
void writeToSerial() {
  SERIAL_USB.print(sent_data);
}

// returns a token between the given separators at the specified index
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
      if (data.charAt(i) == separator || i == maxIndex) {
          found++;
          strIndex[0] = strIndex[1] + 1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// retrieves only the latest string of commands sent from the Jetson
String getLast(String data) {
  for (int i = data.length()-2; i > 0; ) {
      if (isSpace(data.charAt(i)))
          return data.substring(i+1, data.length()-1);
      --i;
  }
  
  return data;
}

// extracts the robot trajectory commands from the Jetson serial data
void parseJetsonSerial() {
  #ifdef ENABLE_JETSON

  if (SERIAL_JETSON.available() >= 22) {  // setting this number too low will cause readStringUntil() to waste time waiting for the ending character
    #ifdef DEBUG_TIMER
    elapsedMicros timer;
    #endif

    t_last_Jetson = millis();
    
    received_data = SERIAL_JETSON.readStringUntil(' ');
    received_data = getLast(received_data);

    input_y = getValue(received_data, ':', 0).toFloat();
    input_x = getValue(received_data, ':', 1).toFloat();
    input_height = getValue(received_data, ':', 2).toFloat();
    input_swing = getValue(received_data, ':', 3).toFloat();

    #ifdef DEBUG_TIMER
    SERIAL_USB.print("parseJetsonSerial() took\t\t");
    SERIAL_USB.print(timer);
    SERIAL_USB.print(" microseconds\n");
    #endif
  }

  #endif
}

// extracts the IMU data sent from a 2nd Teensy over serial
void parseTeensySerial() {
  #ifdef ENABLE_2ND_TEENSY
  
  if (SERIAL_TEENSY.available() >= 50) {  // setting this number too low will cause readStringUntil() to waste time waiting for the ending character
    #ifdef DEBUG_TIMER
    elapsedMicros timer;
    #endif

    received_data = SERIAL_TEENSY.readStringUntil(' ');
    received_data = getLast(received_data);

    input_rpy[0] = getValue(received_data, ':', 0).toFloat();
    input_rpy[1] = getValue(received_data, ':', 1).toFloat();
    input_rpy[2] = getValue(received_data, ':', 2).toFloat();
    input_omega[0] = getValue(received_data, ':', 3).toFloat();
    input_omega[1] = getValue(received_data, ':', 4).toFloat();
    input_omega[2] = getValue(received_data, ':', 5).toFloat();

    #ifdef DEBUG_TIMER
    SERIAL_USB.print("parseTeensySerial() took\t\t");
    SERIAL_USB.print(timer);
    SERIAL_USB.print(" microseconds\n");
    #endif
  }

  #endif
}

// write telemetry data to serial and SD card, if enabled
void sendTelemetry() {
  uint32_t t_current = millis();
  if (t_current - t_last_print >= k_dtPrint) {
    t_last_print = t_current;

    // current time
    snprintf(sent_data, sizeof(sent_data), "%.3f\t", (float)t_current / 1000);
    writeToSerial();
    writeToCard(sent_data);

    // actuator position
    for(uint8_t i = 0; i < 6; i++) {
      if(fabs(motors[i].states_.q) < EPS && motors[i].states_.q < 0) {
        motors[i].states_.q = 0;
      }
    }
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motors[0].states_.q, motors[1].states_.q, motors[2].states_.q, motors[3].states_.q, motors[4].states_.q, motors[5].states_.q);

    #ifdef DEBUG_ACTUATOR_POSITION
    SERIAL_USB.print("actuator q (mm): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // actuator position setpoint
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motors[0].states_.q_d, motors[1].states_.q_d, motors[2].states_.q_d, motors[3].states_.q_d, motors[4].states_.q_d, motors[5].states_.q_d);

    #ifdef DEBUG_ACTUATOR_SETPOINT
    SERIAL_USB.print("actuator q_d (mm): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // motor torque
    for(uint8_t i = 0; i < 6; i++) {
      if(fabs(motor_torque_filters[i].filtered_value) < EPS && motor_torque_filters[i].filtered_value < 0) {
        motor_torque_filters[i].filtered_value = 0;
      }
    }
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motor_torque_filters[0].filtered_value, motor_torque_filters[1].filtered_value, motor_torque_filters[2].filtered_value,
             motor_torque_filters[3].filtered_value, motor_torque_filters[4].filtered_value, motor_torque_filters[5].filtered_value);

    #ifdef DEBUG_ACTUATOR_TORQUE
    SERIAL_USB.print("motor torque (Nm): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // motor temp
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t",
             motors[0].states_.motor_temp, motors[1].states_.motor_temp, motors[2].states_.motor_temp, motors[3].states_.motor_temp);

    #ifdef DEBUG_ACTUATOR_TEMP
    SERIAL_USB.print("actuator temp (C): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // leg positions
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7]);

    #ifdef DEBUG_LEG_POSITION
    SERIAL_USB.print("leg q (mm): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // leg velocities
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             q_dot_filters[0].filtered_value, q_dot_filters[1].filtered_value, q_dot_filters[2].filtered_value, q_dot_filters[3].filtered_value, q_dot_filters[4].filtered_value, q_dot_filters[5].filtered_value, q_dot_filters[6].filtered_value, q_dot_filters[7].filtered_value);

    #ifdef DEBUG_LEG_VELOCITY
    SERIAL_USB.print("leg q_dot (mm/s): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // leg accelerations
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             q_ddot_filters[0].filtered_value, q_ddot_filters[1].filtered_value, q_ddot_filters[2].filtered_value, q_ddot_filters[3].filtered_value, q_ddot_filters[4].filtered_value, q_ddot_filters[5].filtered_value, q_ddot_filters[6].filtered_value, q_ddot_filters[7].filtered_value);

    #ifdef DEBUG_LEG_ACCELERATION
    SERIAL_USB.print("leg q_ddot (mm/s^2): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // ground contacts
    snprintf(sent_data, sizeof(sent_data), "%d\t%d\t%d\t%d\t",
             isInContact[0], isInContact[1], isInContact[2], isInContact[3]);

    #ifdef DEBUG_CONTACT
    SERIAL_USB.print("contacts: ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // battery voltage, current, power for motors
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t", battery_voltage, battery_current, battery_power);

    #ifdef DEBUG_POWER
    SERIAL_USB.print("battery V, A, P: ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // individual motor power
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
              motors[0].states_.bus_voltage*motors[0].states_.bus_current, motors[1].states_.bus_voltage*motors[1].states_.bus_current, motors[2].states_.bus_voltage*motors[2].states_.bus_current,
              motors[3].states_.bus_voltage*motors[3].states_.bus_current, motors[4].states_.bus_voltage*motors[4].states_.bus_current, motors[5].states_.bus_voltage*motors[5].states_.bus_current);

    #ifdef DEBUG_POWER
    // SERIAL_USB.print("motor P: ");
    // writeToSerial();
    #endif
    writeToCard(sent_data);

    // IMU pose estimation
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t", rpy_lateral[0], rpy_lateral[1], rpy_lateral[2]);

    #ifdef DEBUG_RPY
    SERIAL_USB.print("RPY: ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // IMU angular velocity
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t", omega_filters[0].filtered_value, omega_filters[1].filtered_value, omega_filters[2].filtered_value);

    #ifdef DEBUG_OMEGA
    SERIAL_USB.print("omega: ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // robot trajectory parameters
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t", cmd_vector[0], cmd_vector[1], z_body_nominal, leg_swing_percent);

    #ifdef DEBUG_TRAJECTORY
    SERIAL_USB.print("trajectory: ");
    writeToSerial();

    #endif
    writeToCard(sent_data);

    // gait variables
    snprintf(sent_data, sizeof(sent_data), "%d\t%d\t%lu\t%.2f\t", gait_phase, actuation_phase, gait_cycles, dist_traveled/1000);

    #ifdef DEBUG_GAIT
    SERIAL_USB.print("gait, actuation, cycles, dist: ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    snprintf(sent_data, sizeof(sent_data), "\n");
    writeToSerial();
    writeToCard(sent_data);
  }
}

