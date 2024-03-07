#include "mcu_util.h"
#include "..\state_estimation\state_estimation.h"

// time variables
uint32_t t_last_print = 0;                // timestamp at last serial print
uint32_t t_last_setpoint_update = 0;      // timestamp at last setpoint update
uint32_t t_last_motor_limits_update = 0;  // timestamp at last motor controller limits update
uint32_t t_last_motor_cmd_update = 0;     // timestamp at last motor command update
uint32_t t_last_CAN_msg = 0;              // timestamp at last CAN msg

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

// initialize Teensy 4.1 digital pins, RTC and etc.
void initTeensy() {
  delay(100);   // short delay after the start to ensure the first serial print is not missed over USB
  SERIAL_USB.println("Teensy started.\n");

  delay(1000);

  // add digital pin initialization here
  pinMode(ACS711EX_FAULT, INPUT);
  pinMode(SEL, INPUT_PULLUP);
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
void initSDCard() {
  #ifdef ENABLE_SD_CARD

  if (!SD.begin(chip_select)) {
    digitalWrite(LED, HIGH);
    SERIAL_USB.println("SD card failed or not present.\n");
    while (1) {}
  }
  SERIAL_USB.println("SD card initialized.\n");

  // parse through existing data files
  String file_name = "";

  // using Teensy's internal RTC
  file_name = String(year()) + '_' +
              String(month()) + '_' +
              String(day()) + '_' +
              String(hour()) + '_' +
              String(minute()) + '_' +
              String(second()) + ".txt";

  // name the data file with the next number
  file_name.toCharArray(data_file_name, sizeof(data_file_name));
  data_file = SD.open(data_file_name, FILE_WRITE);
  SERIAL_USB.printf("Data being saved to: %s\n\n", data_file_name);

  #endif
}

// write data to the onboard microSD card
void writeToCard(char data[]) {
  #ifdef ENABLE_SD_CARD

  if (data_file) {
    data_file.print(data);
  } else {
    digitalWrite(LED, HIGH);
    SERIAL_USB.println("Error opening the data file");
    stop_signal = true;
  }

  #endif
}

// stop writing to the file and save
void closeDataFile() {
  #ifdef ENABLE_SD_CARD

  data_file.close();
  SERIAL_USB.println("Saved the data file");

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

