#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "..\ODriveTeensyCAN\ODriveTeensyCAN.h"

// #define DEBUG_MOTOR_CONTROLLER

class Actuator {

  // the ODrive only knows pos_abs
  // all position commands are converted to pos_abs by subtracting pos_home (when not homed, pos_home = 0)
  // e.g. if pos_home is -3 and the position command is 50, the actuator will move to pos_abs = 53
  struct axisStates{
    uint8_t axis_state;     // ODrive axis current state (odrv<>.axis<>.current_state)
    uint8_t ctrl_mode;      // ODrive axis control mdoe (odrv<>.axis<>.controller.config.control_mode)
    uint32_t axis_error;    // ODrive axis error (odrv<>.axis<>.error)

    // motor limits
    float velocity_limit;
    float current_limit;

    float trap_traj_accel_limit;
    float trap_traj_decel_limit;
    float trap_traj_vel_limit;

    bool homed;             // false until homed and manually to true
    bool holding;           // true if fabs(q-q_d) < Q_MAX_ERROR

    // "motor space" values
    float pos_abs;          // motor position reported by the ODrive after powerup (turns); <axis>.pos_vel_mapper.pos_rel or <axis>.pos_vel_mapper.pos_abs, depending on ODrive.Controller.Config.absolute_setpoints
    float pos_home;         // pos_abs at home
    float pos_rel;          // pos_rel = pos_abs - pos_home (turns)
    float vel;              // motor velocity (turns/s)
    float current;          // motor current draw (amps)
    float torque;           // motor torque (Nm)
    float torque_d;         // motor torque setpoint (Nm)

    // joint space values
    float q;                // joint position in mm or deg (transmission ratio*pos_rel)
    float q_d;              // joint position setpoint in mm or deg
    float q_min;            // minimum allowed q
    float q_max;            // maximum allowed q

    float q_dot;            // joint velocity in mm/s or deg/s (transmission ratio*vel)
    float q_dot_d;          // joint velocity setpoint in mm/s or deg/s

    float tau;              // joint force/torque in F or Nm (equal to motor torque for now)
    float tau_d;            // joint force/torque setpoint in F or Nm

    // additional ODrive states
    float fet_temp;         // FET thermistor temperature
    float motor_temp;       // motor thermistor temperature

    float bus_voltage;
    float bus_current;
  };

  // constant values
  struct axisParams{
    uint8_t axis;           // CAN node id of the control axis (odrv<>.axis<>.config.can.node_id)
    int direction;          // actuator motion direction; 1 or -1
    float K_T;              // motor torque constant
    float T;                // transmission ratio = joint disp./motor shaft disp.
  };

  private:
    ODriveTeensyCAN ODrive_;                 // ODrive object to call member functions
    usb_serial_class & USB_serial_ = Serial;  // Serial object for printing debug info

  public:
    struct axisStates states_;
    struct axisParams params_;

    // default constructor
    // Actuator();

    // alternate constructor
    Actuator(const ODriveTeensyCAN & _ODrive, usb_serial_class & _USB_Serial,  uint8_t _axis, int _direction, float _transmissionRatio, float _torqueConstant, float _minPos, float _maxPos,
             float _vel_lim, float _cur_lim, float _tt_accel_limit, float _tt_decel_limit, float _tt_vel_limit);

    bool enable();                // enters closed-loop control
    bool disable();               // enters idle state

    // sends torque, velocity, or position control
    // mode: 1 (torque control), 2 (velocity control), 3 (position control)
    // val: torque cmd, velocity cmd, position (pos_rel) cmd
    void sendCommand(uint8_t mode, float val);

    // changes control mode and input mode; does NOT enter closed-loop control
    // enable() must be called to enter closed-loop control
    // mode: 1 (torque control/passthrough input), 2 (velocity control), 3 (position control)
    void setControlMode(uint8_t mode);
};

#endif