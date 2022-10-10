#include "ODrive.h"
#include "Actuator.h"

#define BAUD_RATE_ODRIVE 115200

void setup() {
  Serial.begin(9200);
  ODrive od1 = ODrive(Serial1, BAUD_RATE_ODRIVE, Serial);
  ODrive od2 = ODrive(Serial2, BAUD_RATE_ODRIVE, Serial);
  ODrive od3 = ODrive(Serial7, BAUD_RATE_ODRIVE, Serial);

  while(true) {
    Serial.print("1: ");
    Serial.print(od1.getBusVoltage());
    Serial.print('\t');
    Serial.print("2: ");
    Serial.print(od2.getBusVoltage());
    Serial.print('\t');
    Serial.print("3: ");
    Serial.print(od3.getBusVoltage());
    Serial.println();
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
