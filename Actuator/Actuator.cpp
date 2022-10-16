#include "Actuator.h"

// alternate constructor
Actuator::Actuator(const ODriveTeensyCAN & _ODrive, usb_serial_class & _USB_Serial, uint8_t _axis, float _torqueConstant, float _minPos, float _maxPos): myODrive(_ODrive), myUSBSerial(_USB_Serial) {
    states.axis = _axis;
    states.homed = false;
    states.state = 0;
    states.pos_home = 0;
    states.pos_min = _minPos;
    states.pos_max = _maxPos;
    params.torqueConstant = _torqueConstant;
}

bool Actuator::enable() {
    bool success = myODrive.RunState(states.axis, 8);
    if (success) {
        snprintf(sentData, sizeof(sentData), "Actuator %d entered closed-loop control.\n", states.axis+1);
        myUSBSerial.print(sentData);
    } else {
        snprintf(sentData, sizeof(sentData), "Actuator %d failed to enter closed-loop control.\n", states.axis+1);
        myUSBSerial.print(sentData);
    }
    return success;
}

bool Actuator::disable() {
    if (states.state != 1) {
        bool success = myODrive.RunState(states.axis, 1);
        if (success) {
            snprintf(sentData, sizeof(sentData), "Actuator %d entered idle mode.\n", states.axis+1);
            myUSBSerial.print(sentData);
        } else {
            snprintf(sentData, sizeof(sentData), "Actuator %d failed to enter idle mode.\n", states.axis+1);
            myUSBSerial.print(sentData);
        }
        return success;
        
    } else {
        snprintf(sentData, sizeof(sentData), "Actuator %d already in idle.\n", states.axis+1);
        myUSBSerial.print(sentData);
        return true;
    }
}

void Actuator::sendCommand(uint8_t mode, float val) {
    if (mode == 1) {
        myODrive.SetTorque(states.axis, val);
    } else if (mode == 2) {
        myODrive.SetVelocity(states.axis, val);
    } else if (mode == 3) {
        if (val >= states.pos_min && val <= states.pos_max) { // checking bounds on pos_rel
            myODrive.SetPosition(states.axis, val  + states.pos_home); // converts to and sends pos_abs as the position command to the ODrive
        } else {
            snprintf(sentData, sizeof(sentData), "Actuator %d position setpoint outside of range.\n", states.axis+1);
            // Serial.printf("%.3f\t%.3f\t%.3f\n", val, )
            // delay(100000);
            myUSBSerial.print(sentData);
        }
        
    } else {
        snprintf(sentData, sizeof(sentData), "Invalid command mode.\n");
        myUSBSerial.print(sentData);
    }
}

void Actuator::setControlMode(uint8_t mode) {
    if (mode == 3) {
        myODrive.SetControllerModes(states.axis, 3, 5);
        snprintf(sentData, sizeof(sentData), "Entering position control.\n");
        myUSBSerial.print(sentData);
    } else if (mode == 1 || mode == 2) {
        myODrive.SetControllerModes(states.axis, mode, 1);
        snprintf(sentData, sizeof(sentData), "Entering velocity control.\n");
        myUSBSerial.print(sentData);
    } else {
        snprintf(sentData, sizeof(sentData), "Invalid control mode.\n");
        myUSBSerial.print(sentData);
    }
}