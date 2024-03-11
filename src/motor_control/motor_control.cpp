#include "motor_control.h"
#include "..\..\include\joint_defs.h"
#include "..\mcu_util\mcu_util.h"
#include "..\state_estimation\state_estimation.h"

ODriveTeensyCAN ODrive_CAN = ODriveTeensyCAN(kODriveCANBaudRate);

std::vector<Actuator> motors = {Actuator(ODrive_CAN, SERIAL_USB, 0, kDirectionLeg, kTxRatioLeg, k_KV_to_KT / k_KV_D5312S, kQLegMin, kQLegMax,
                                kVelLegMax, kCurrentLegMax, kAccelLegTrajSwing, kDecelLegTrajSwing, kVelLegTrajSwing),

                                Actuator(ODrive_CAN, SERIAL_USB, 1, kDirectionLeg, kTxRatioLeg, k_KV_to_KT / k_KV_D5312S, kQLegMin, kQLegMax,
                                kVelLegMax, kCurrentLegMax, kAccelLegTrajSwing, kDecelLegTrajSwing, kVelLegTrajSwing),

                                Actuator(ODrive_CAN, SERIAL_USB, 2, kDirectionLeg, kTxRatioLeg, k_KV_to_KT / k_KV_D5312S, kQLegMin, kQLegMax,
                                kVelLegMax, kCurrentLegMax, kAccelLegTrajSwing, kDecelLegTrajSwing, kVelLegTrajSwing),

                                Actuator(ODrive_CAN, SERIAL_USB, 3, kDirectionLeg, kTxRatioLeg, k_KV_to_KT / k_KV_D5312S, kQLegMin, kQLegMax,
                                kVelLegMax, kCurrentLegMax, kAccelLegTrajSwing, kDecelLegTrajSwing, kVelLegTrajSwing),

                                Actuator(ODrive_CAN, SERIAL_USB, 4, kDirectionTrans, kTxRatioTrans, k_KV_to_KT / k_KV_GL60, -kQTransMax, kQTransMax,
                                kVelTransMax, kCurrentTransMax, kAccelTransTraj, kDecelTransTraj, kVelTransTraj),

                                Actuator(ODrive_CAN, SERIAL_USB, 5, kDirectionYaw, kTxRatioYaw, k_KV_to_KT / k_KV_GL60, -kQYawMax, kQYawMax,
                                kVelYawMax, kCurrentYawMax, kAccelYawTraj, kDecelYawTraj, kVelYawTraj)};

// reads the next CAN msg from an ODrive CAN node
// calls member variables of Actuator objects
uint32_t handleODriveCANMsg() {
  CAN_message_t msg;
  uint32_t msg_id = 0;  // this value corresponds to node_id = 0, cmd_id = 0

  // stop code if the CAN messages have stopped coming in
  if (t_last_CAN_msg > 0 && millis() - t_last_CAN_msg > kODriveCANWatchdogTimeout) {
    snprintf(sent_data, sizeof(sent_data), "CAN watchdog timeout\n\n");
    writeToSerial();
    stop_signal = true;
  }

  if (ODrive_CAN.ReadMsg(msg)) {
    t_last_CAN_msg = millis();    // feed the watchdog

    msg_id = msg.id;
    uint8_t axis_id = getAxisID(msg_id);
    uint8_t cmd_id = getCmdID(msg_id);

    // read heartbeat message
    if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdHeartbeat) {
      readHeartbeat(axis_id, msg);

      if (motors[axis_id].states_.axis_error) {
        snprintf(sent_data, sizeof(sent_data), "ERROR: Axis %d %s.\n\n", axis_id + 1, ODrive_CAN.error_to_string(motors[axis_id].states_.axis_error).c_str());
        writeToSerial();
        #ifdef ENABLE_SD_CARD
        writeToCard(sent_data);
        #endif
        if (motors[axis_id].states_.axis_error != ODriveTeensyCAN::ODriveError::ODRIVE_ERROR_INITIALIZING) {
          stop_signal = true;
        }
      }

    // read active_error
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetError) {
      ErrorMsg_t error_msg;
      ODrive_CAN.GetErrorResponse(error_msg, msg);

    // read position/velocity
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetEncoderEstimates) {
      EncoderEstimatesMsg_t encoder_msg;
      ODrive_CAN.GetPositionVelocityResponse(encoder_msg, msg);
      motors[axis_id].states_.pos_abs = encoder_msg.Pos_Estimate;
      motors[axis_id].states_.pos_rel = motors[axis_id].states_.pos_abs - motors[axis_id].states_.pos_home;
      motors[axis_id].states_.vel = encoder_msg.Vel_Estimate;
      motors[axis_id].states_.q = motors[axis_id].params_.direction * motors[axis_id].params_.T * motors[axis_id].states_.pos_rel;
      motors[axis_id].states_.q_dot = motors[axis_id].params_.direction * motors[axis_id].params_.T * motors[axis_id].states_.vel;

    // read motor current
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetIq) {
      IqMsg_t Iq_msg;
      ODrive_CAN.GetIqResponse(Iq_msg, msg);
      motors[axis_id].states_.current = Iq_msg.Iq_Measured;

    // read motor torque
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetTorques) {
      TorqueMsg_t torque_msg;
      ODrive_CAN.GetTorquesResponse(torque_msg, msg);
      motors[axis_id].states_.torque = torque_msg.Torque_Estimate;
      motors[axis_id].states_.torque_d = torque_msg.Torque_Target;

    // read temperature
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetTemperature) {
      TemperatureMsg_t temp_msg;
      ODrive_CAN.GetTemperatureResponse(temp_msg, msg);
      motors[axis_id].states_.fet_temp = temp_msg.FET_Temperature;
      motors[axis_id].states_.motor_temp = temp_msg.Motor_Temperature;

      if (motors[axis_id].states_.motor_temp > kTemperatureLegMotorMax) {
        snprintf(sent_data, sizeof(sent_data), "Motor %d over %.1f degrees Celsius.\n\n", axis_id + 1, kTemperatureLegMotorMax);
        writeToSerial();
        #ifdef ENABLE_SD_CARD
        writeToCard(sent_data);
        #endif
        stop_signal = true;
      }

    // read bus voltage/current
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetBusVoltageCurrent) {
      BusViMsg_t bus_vi_msg;
      ODrive_CAN.GetBusVoltageCurrentResponse(bus_vi_msg, msg);
      motors[axis_id].states_.bus_voltage = bus_vi_msg.Bus_Voltage;
      motors[axis_id].states_.bus_current = bus_vi_msg.Bus_Current;
      
    }
  }
  return msg_id;
}

// extracts and returns the axis/node ID from the ODrive's CAN message
uint8_t getAxisID(uint32_t msg_id) {
  return (msg_id & ((int)(pow(2, kAxisIDLength)) - 1) << kCmdIDLength) >> kCmdIDLength; // axis ID is the 6 highest bits
}

// extracts and returns the command ID from the ODrive's CAN message
uint8_t getCmdID(uint32_t msg_id) {
  return msg_id & ((int)pow(2, kCmdIDLength) - 1); // command ID is the 5 lower bits
}

// blocking function that reads the heartbeat msg data
// used only in the initODrive() function
void waitForHeartbeat(uint8_t axis) {
  while (true) {
    uint32_t msg_id = handleODriveCANMsg();
    uint8_t axis_id = getAxisID(msg_id);
    uint8_t cmd_id = getCmdID(msg_id);
    if ((axis_id == axis) & (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdHeartbeat)) {
      return;
    }
  }
}

void readHeartbeat(uint8_t axis_id, CAN_message_t msg) {
  HeartbeatMsg_t heartbeat_msg;
  ODrive_CAN.Heartbeat(heartbeat_msg, msg);
  motors[axis_id].states_.axis_state = heartbeat_msg.Axis_State;
  motors[axis_id].states_.axis_error = heartbeat_msg.Axis_Error;
}

// looks for the ODrive CAN heartbeat msg and enters closed-loop control
void initActuators() {
  snprintf(sent_data, sizeof(sent_data), "Waiting for ODrives to power on...\n");
  writeToSerial();

  // wait for the first CAN message to come in
  while (!handleODriveCANMsg()) {
  }
  uint32_t t_start = millis();
  while(millis() - t_start < 1000) {
    handleODriveCANMsg();
  }
  
  // powering on the motor controllers after the logic leads to ODRIVE_ERROR_INITIALIZING, which can be cleared
  for (uint8_t axis_id = 0; axis_id < kNumOfActuators; ++axis_id) {
    ODrive_CAN.ClearErrors(axis_id);
  }
  t_start = millis();
  while(millis() - t_start < 1000) {
    handleODriveCANMsg();
  }

  for (uint8_t axis_id = 0; axis_id < kNumOfActuators; ++axis_id) {
    snprintf(sent_data, sizeof(sent_data), "Checking ODrive axis %d...\n", axis_id + 1);
    writeToSerial();

    waitForHeartbeat(axis_id);
    snprintf(sent_data, sizeof(sent_data), "Current state: %d\tAxis error: %lu\n\n", motors[axis_id].states_.axis_state, motors[axis_id].states_.axis_error);
    writeToSerial();

    if (motors[axis_id].states_.axis_error) {
      digitalWrite(LED, HIGH);

      snprintf(sent_data, sizeof(sent_data), "Stopping. Check the axis error.\n");
      writeToSerial();
      
      while (true) {} // if there's an axis error, do not proceed
      
    } else {
      snprintf(sent_data, sizeof(sent_data), "Putting actuator %d in closed-loop control...\n", axis_id + 1);
      writeToSerial();
      motors[axis_id].enable();
    }
  }

  snprintf(sent_data, sizeof(sent_data), "All actuators in closed-loop control.\n---------------------------------------------\n\n");
  writeToSerial();
}

// calls the Estop() function on all axes
void stopActuators() {
  for (uint8_t axis_id = 0; axis_id < kNumOfActuators; ++axis_id) {
    snprintf(sent_data, sizeof(sent_data), "Stopping actuator %d...\n", axis_id + 1);
    writeToSerial();
    ODrive_CAN.Estop(axis_id);     // calling the Estop function will raise the error flag; needs ClearErrors(axis_id) or power cycling
  }

  snprintf(sent_data, sizeof(sent_data), "Actuators stopped.\n---------------------------------------------\n\n");
  writeToSerial();
}

// updates motor controller limits on all axes
void updateMotorLimits() {
  uint32_t t_current = millis();
  if (t_current - t_last_motor_limits_update >= k_dtMotorLimitsUpdate) {
    t_last_motor_limits_update = t_current;
    
    #ifdef DEBUG_TIMER
    elapsedMicros timer;
    #endif

    for (uint8_t axis_id = 0; axis_id < kNumOfActuators; ++axis_id) {

      // update motor and controller limits
      ODrive_CAN.SetLimits(axis_id, motors[axis_id].states_.velocity_limit, motors[axis_id].states_.current_limit);
      ODrive_CAN.SetTrajVelLimit(axis_id, motors[axis_id].states_.trap_traj_vel_limit);
      ODrive_CAN.SetTrajAccelLimits(axis_id, motors[axis_id].states_.trap_traj_accel_limit, motors[axis_id].states_.trap_traj_decel_limit);
    }

    #ifdef DEBUG_TIMER
    SERIAL_USB.print("updating controller limits took\t\t");
    SERIAL_USB.print(timer);
    SERIAL_USB.print(" microseconds\n");
    #endif

  }
}

// updates motor commands on all axes
void updateMotorCommands() {
  uint32_t t_current = millis();
  if (t_current - t_last_motor_cmd_update >= k_dtMotorCmdUpdate) {
    t_last_motor_cmd_update = t_current;
    
    #ifdef DEBUG_TIMER
    elapsedMicros timer;
    #endif

    for (uint8_t axis_id = 0; axis_id < kNumOfActuators; ++axis_id) {

      // update motor and controller limits
      ODrive_CAN.SetLimits(axis_id, motors[axis_id].states_.velocity_limit, motors[axis_id].states_.current_limit);
      ODrive_CAN.SetTrajVelLimit(axis_id, motors[axis_id].states_.trap_traj_vel_limit);
      ODrive_CAN.SetTrajAccelLimits(axis_id, motors[axis_id].states_.trap_traj_accel_limit, motors[axis_id].states_.trap_traj_decel_limit);
      motors[axis_id].setControlMode(motors[axis_id].states_.ctrl_mode);
    }

    delay(5);

    for (uint8_t axis_id = 0; axis_id < kNumOfActuators; ++axis_id) {
      // update motor command
      if (motors[axis_id].states_.ctrl_mode == ODriveTeensyCAN::ControlMode_t::kPositionControl) {  // if in position control
        motors[axis_id].sendCommand(ODriveTeensyCAN::ControlMode_t::kPositionControl, motors[axis_id].states_.q_d);
        motors[axis_id].states_.holding = fabs(motors[axis_id].states_.q - motors[axis_id].states_.q_d) < kQErrorMax;

      } else if (motors[axis_id].states_.ctrl_mode == ODriveTeensyCAN::ControlMode_t::kTorqueControl) {  // if in torque control
        motors[axis_id].sendCommand(ODriveTeensyCAN::ControlMode_t::kTorqueControl, motors[axis_id].states_.tau_d);

      } else if (motors[axis_id].states_.ctrl_mode == ODriveTeensyCAN::ControlMode_t::kVelocityControl) {  // if in velocity control
        motors[axis_id].sendCommand(ODriveTeensyCAN::ControlMode_t::kVelocityControl, motors[axis_id].states_.q_dot_d);
      }
    }

    #ifdef DEBUG_TIMER
    SERIAL_USB.print("updating motor commands took\t\t");
    SERIAL_USB.print(timer);
    SERIAL_USB.print(" microseconds\n");
    #endif

  }
}

