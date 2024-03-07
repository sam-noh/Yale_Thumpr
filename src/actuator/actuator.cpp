#include "actuator.h"
#include "..\mcu_util\mcu_util.h"

// default constructor
// Actuator::Actuator() {}

// alternate constructor
Actuator::Actuator(const ODriveTeensyCAN & _ODrive, usb_serial_class & _USB_Serial, uint8_t _axis, int _direction, float _T, float _K_T, float _qMin, float _qMax,
                   float _vel_lim, float _cur_lim, float _tt_accel_limit, float _tt_decel_limit, float _tt_vel_limit): ODrive_(_ODrive), USB_serial_(_USB_Serial) {
    
    states_.axis_state = 0;
    states_.ctrl_mode = 0;
    states_.axis_error = 0;

    states_.velocity_limit = _vel_lim;
    states_.current_limit = _cur_lim;

    states_.trap_traj_accel_limit = _tt_accel_limit;
    states_.trap_traj_decel_limit = _tt_decel_limit;
    states_.trap_traj_vel_limit = _tt_vel_limit;

    states_.homed = false;
    states_.holding = false;

    states_.pos_home = 0;
    states_.q_d = 0;
    states_.q_dot_d = 0;
    states_.tau_d = 0;
    states_.q_min = _qMin;
    states_.q_max = _qMax;
    params_.axis = _axis;
    params_.direction = _direction;
    params_.T = _T;
    params_.K_T = _K_T;
}

bool Actuator::enable() {
    bool success = ODrive_.RunState(params_.axis, ODriveTeensyCAN::AxisState_t::kAxisStateClosedLoopControl);
    if (success) {
        states_.axis_state = ODriveTeensyCAN::AxisState_t::kAxisStateClosedLoopControl;
        snprintf(sent_data, sizeof(sent_data), "Actuator %d entered closed-loop control.\n\n", params_.axis+1);
        USB_serial_.print(sent_data);
    } else {
        snprintf(sent_data, sizeof(sent_data), "Actuator %d failed to enter closed-loop control.\n\n", params_.axis+1);
        USB_serial_.print(sent_data);
    }
    return success;
}

bool Actuator::disable() {
    if (states_.axis_state != ODriveTeensyCAN::AxisState_t::kAxisStateIdle) {
        bool success = ODrive_.RunState(params_.axis, ODriveTeensyCAN::AxisState_t::kAxisStateIdle);
        if (success) {
            states_.axis_state = ODriveTeensyCAN::AxisState_t::kAxisStateIdle;
            snprintf(sent_data, sizeof(sent_data), "Actuator %d entered idle mode.\n\n", params_.axis+1);
            USB_serial_.print(sent_data);
        } else {
            snprintf(sent_data, sizeof(sent_data), "Actuator %d failed to enter idle mode.\n\n", params_.axis+1);
            USB_serial_.print(sent_data);
        }
        return success;
        
    } else {
        snprintf(sent_data, sizeof(sent_data), "Actuator %d already in idle.\n\n", params_.axis+1);
        USB_serial_.print(sent_data);
        return true;
    }
}

void Actuator::sendCommand(uint8_t mode, float val) {
    if (mode != states_.ctrl_mode) {
        snprintf(sent_data, sizeof(sent_data), "Actuator %d is in incorrect control mode.\n\n", params_.axis+1);
        USB_serial_.print(sent_data);
        return;
    }

    float cmd = 0;
    if (mode == ODriveTeensyCAN::ControlMode_t::kTorqueControl) {    // for now, torque command is NOT in joint space
        cmd = params_.direction*val;
    } else {
        cmd = (params_.direction*val)/params_.T;
    }
    
    if (mode == ODriveTeensyCAN::ControlMode_t::kTorqueControl) {
        ODrive_.SetTorque(params_.axis, cmd);
    } else if (mode == ODriveTeensyCAN::ControlMode_t::kVelocityControl) {
        ODrive_.SetVelocity(params_.axis, cmd);
    } else if (mode == ODriveTeensyCAN::ControlMode_t::kPositionControl) {
        if (!states_.homed || (states_.homed && val >= states_.q_min && val <= states_.q_max)) { // ONLY CHECK BOUNDS IF HOMED; checking bounds on pos_rel
            ODrive_.SetPosition(params_.axis, cmd  + states_.pos_home); // converts to and sends pos_abs as the position command to the ODrive
        } else {
            snprintf(sent_data, sizeof(sent_data), "Actuator %d position setpoint outside of range.\n", params_.axis+1);
            USB_serial_.print(sent_data);
        }
        
    } else {
        snprintf(sent_data, sizeof(sent_data), "Invalid command mode.\n");
        USB_serial_.print(sent_data);
    }
}

void Actuator::setControlMode(uint8_t mode) {
    if (mode == ODriveTeensyCAN::ControlMode_t::kPositionControl) {
        ODrive_.SetControllerModes(params_.axis,
                                   ODriveTeensyCAN::ControlMode_t::kPositionControl,
                                   ODriveTeensyCAN::InputMode_t::kTrapTraj);     // 3 = pos ctrl, 5 = trapezoidal trajectory
        states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kPositionControl;

        #ifdef DEBUG_MOTOR_CONTROLLER
        snprintf(sent_data, sizeof(sent_data), "Entering position control.\n");
        writeToSerial();
        #endif

    } else if (mode == ODriveTeensyCAN::ControlMode_t::kVelocityControl) {
        ODrive_.SetControllerModes(params_.axis,
                                   ODriveTeensyCAN::ControlMode_t::kVelocityControl,
                                   ODriveTeensyCAN::InputMode_t::kPassthrough);  // 2 = vel ctrl, 1 = passthrough input
        states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kVelocityControl;

        #ifdef DEBUG_MOTOR_CONTROLLER
        snprintf(sent_data, sizeof(sent_data), "Entering velocity control.\n");
        writeToSerial();
        #endif

    } else if (mode  == ODriveTeensyCAN::ControlMode_t::kTorqueControl) {
        ODrive_.SetControllerModes(params_.axis,
                                   ODriveTeensyCAN::ControlMode_t::kTorqueControl,
                                   ODriveTeensyCAN::InputMode_t::kPassthrough);  // 1 = torque ctrl, 1 = passthrough input
        states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kTorqueControl;

        #ifdef DEBUG_MOTOR_CONTROLLER
        snprintf(sent_data, sizeof(sent_data), "Entering torque control.\n");
        writeToSerial();
        #endif
        
    } else {
        #ifdef DEBUG_MOTOR_CONTROLLER
        snprintf(sent_data, sizeof(sent_data), "Invalid control mode.\n");
        writeToSerial();
        #endif
    }
}