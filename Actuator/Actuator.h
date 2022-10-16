#ifndef Actuator_h
#define Actuator_h

#include "ODriveTeensyCAN.h"
#include "float.h"

class Actuator {

  // the ODrive only knows pos_abs
  // all position commands are converted to pos_abs by subtracting pos_home (when not homed, pos_home = 0)
  // e.g. if pos_home is -3 and the position command is 50, the actuator will move to pos_abs = 53
  struct axisStates{
    uint8_t axis;           // CAN node id of the control axis (odrv<>.axis<>.config.can.node_id)
    uint8_t state;          // ODrive axis current state (odrv<>.axis<>.current_state)
    uint32_t axisError;     // ODrive axis error (odrv<>.axis<>.error)
    bool homed;             // false initially; calling setHomePosition() sets it to true

    float pos_abs;          // motor position reported by the ODrive after powerup (turns)
    float pos_rel;          // pos_rel = pos_abs - pos_home (turns)
    float pos_home;         // pos_abs at home; call setHomePosition()
    float pos_min;          // minimum allowed pos_rel
    float pos_max;          // maximum allowed pos_rel
    float velocity;         // motor velocity (turns/s)
    float current;          // motor current draw (amps)
    float torque;           // motor torque (Nm)

    float q_cur;            // joint position in mm or rad (transmission ratio*pos_rel)
    float q_target;         // joint setpoint in mm or rad
  };

  // constant values
  struct axisParams{
    float torqueConstant;   // motor torque constant
  };

  private:
    ODriveTeensyCAN myODrive;                 // ODrive object to call member functions
    usb_serial_class & myUSBSerial = Serial;  // Serial object for printing debug info
    char sentData[128] = "";
    

  public:
    struct axisStates states;
    struct axisParams params;

    // alternate constructor
    Actuator(const ODriveTeensyCAN & _ODrive, usb_serial_class & _USB_Serial,  uint8_t _axis, float _torqueConstant, float _minPos, float _maxPos);

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