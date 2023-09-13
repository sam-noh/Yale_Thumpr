#include "Actuator.h"

// alternate constructor
Actuator::Actuator(const ODriveTeensyCAN & _ODrive, usb_serial_class & _USB_Serial, uint8_t _axis, int _direction, float _T, float _K_T, float _qMin, float _qMax): myODrive(_ODrive), myUSBSerial(_USB_Serial) {
    states.homed = false;
    states.holding = false;
    states.state = 0;
    states.ctrl_mode = 0;
    states.pos_home = 0;
    states.q_min = _qMin;
    states.q_max = _qMax;
    params.axis = _axis;
    params.direction = _direction;
    params.T = _T;
    params.K_T = _K_T;
}

bool Actuator::enable() {
    bool success = myODrive.RunState(params.axis, 8);
    if (success) {
        states.state = 8;
        snprintf(sentData, sizeof(sentData), "Actuator %d entered closed-loop control.\n", params.axis+1);
        myUSBSerial.print(sentData);
    } else {
        snprintf(sentData, sizeof(sentData), "Actuator %d failed to enter closed-loop control.\n", params.axis+1);
        myUSBSerial.print(sentData);
    }
    return success;
}

bool Actuator::disable() {
    if (states.state != 1) {
        bool success = myODrive.RunState(params.axis, 1);
        if (success) {
            states.state = 1;
            snprintf(sentData, sizeof(sentData), "Actuator %d entered idle mode.\n", params.axis+1);
            myUSBSerial.print(sentData);
        } else {
            snprintf(sentData, sizeof(sentData), "Actuator %d failed to enter idle mode.\n", params.axis+1);
            myUSBSerial.print(sentData);
        }
        return success;
        
    } else {
        snprintf(sentData, sizeof(sentData), "Actuator %d already in idle.\n", params.axis+1);
        myUSBSerial.print(sentData);
        return true;
    }
}

void Actuator::sendCommand(uint8_t mode, float val) {
    float cmd = 0;
    if (mode == 1) {    // for now, torque command is NOT in joint space
        cmd = params.direction*val;
    } else {
        cmd = (params.direction*val)/params.T;
    }
    
    if (mode == 1) {
        myODrive.SetTorque(params.axis, cmd);
    } else if (mode == 2) {
        myODrive.SetVelocity(params.axis, cmd);
    } else if (mode == 3) {
        if (!states.homed || states.homed && val >= states.q_min && val <= states.q_max) { // ONLY CHECK BOUNDS IF HOMED; checking bounds on pos_rel
            myODrive.SetPosition(params.axis, cmd  + states.pos_home); // converts to and sends pos_abs as the position command to the ODrive
        } else {
            snprintf(sentData, sizeof(sentData), "Actuator %d position setpoint outside of range.\n", params.axis+1);
            myUSBSerial.print(sentData);
            Serial.printf("Min: %.3f\tMax: %.3f\tq:%.3f\n", states.q_min, states.q_max, val);
        }
        
    } else {
        snprintf(sentData, sizeof(sentData), "Invalid command mode.\n");
        myUSBSerial.print(sentData);
    }
}

void Actuator::setControlMode(uint8_t mode) {
    if (mode == 3) {
        myODrive.SetControllerModes(params.axis, 3, 5);     // 3 = pos ctrl, 5 = trapezoidal trajectory
        states.ctrl_mode = 3;
        snprintf(sentData, sizeof(sentData), "Entering position control.\n");
        myUSBSerial.print(sentData);
    } else if (mode == 2) {
        myODrive.SetControllerModes(params.axis, 2, 1);  // 2 = vel ctrl, 1 = passthrough input
        states.ctrl_mode = 2;
        snprintf(sentData, sizeof(sentData), "Entering velocity control.\n");
        myUSBSerial.print(sentData);
    } else if (mode  == 1) {
        myODrive.SetControllerModes(params.axis, 1, 1);  // 1 = torque ctrl, 1 = passthrough input
        states.ctrl_mode = 1;
        snprintf(sentData, sizeof(sentData), "Entering torque control.\n");
        myUSBSerial.print(sentData);
    } else {
        snprintf(sentData, sizeof(sentData), "Invalid control mode.\n");
        myUSBSerial.print(sentData);
    }
}