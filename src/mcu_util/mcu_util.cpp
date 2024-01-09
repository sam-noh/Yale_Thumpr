#include "mcu_util.h"

// time variables
uint32_t t_last_print = 0;           // timestamp at last serial print
uint32_t t_last_state_update = 0;    // timestamp at last state update
uint32_t t_last_setpoint_update = 0; // timestamp at last setpoint update
uint32_t t_last_motor_cmd = 0;       // timestamp at last motor command update
uint32_t t_last_encoder_update = 0;  // timestamp at last encoder reading

char received_data[MAX_DATA_SIZE] = "";
char sent_data[MAX_DATA_SIZE] = "";
const int chip_select = BUILTIN_SDCARD;
File data_file;  // SD card file
char data_file_name[128] = "";

// initialize Teensy 4.1 digital pins, RTC and etc.
void initTeensy() {
  delay(100);   // short delay after the start to ensure the first print is not missed
  SERIAL_USB.println("Teensy started.\n");

  // add digital pin initialization here
  pinMode(ACS711EX_FAULT, INPUT);
  pinMode(SEL, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  // time sync for Teensy's internal RTC
  // comment out if not using it
  setSyncProvider(getTeensy3Time);

  if (timeStatus()!= timeSet) {
    SERIAL_USB.println("Unable to sync with the RTC.\n");
    digitalWrite(LED, HIGH);
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

// initialize SD card and set file name
void initSDCard() {
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
//    SERIAL_USB.println("Couldn't read time");
//    while(1){};
//  }

  // name the data file with the next number
  file_name.toCharArray(data_file_name, sizeof(data_file_name));
  data_file = SD.open(data_file_name, FILE_WRITE);
  SERIAL_USB.printf("Data being saved to: %s\n\n", data_file_name);
}

// write data to the onboard microSD card
void writeToCard(char data[]) {
  if (data_file) {
    data_file.print(data);
  } else {
    digitalWrite(LED, HIGH);
    SERIAL_USB.println("Error opening data file");
    stop_signal = true;
  }
}

// write data to the Serial output
void transmitMsg() {
    SERIAL_USB.print(sent_data);
}

