#include "motor_control.h"
#include "../../include/joint_defs.h"
#include "../mcu_util/mcu_util.h"
#include "../state_estimation/state_estimation.h"

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
    uint8_t node_id = getNodeID(msg_id);
    uint8_t cmd_id = getCmdID(msg_id);

    // read heartbeat message
    if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdHeartbeat) {
      readHeartbeat(node_id, msg);

      if (motors[node_id].states_.axis_error) {
        snprintf(sent_data, sizeof(sent_data), "ERROR: Axis %d %s.\n\n", node_id + 1, ODrive_CAN.error_to_string(motors[node_id].states_.axis_error).c_str());
        writeToSerial();
        if (motors[node_id].states_.axis_error != ODriveTeensyCAN::ODriveError::ODRIVE_ERROR_INITIALIZING) {
          writeToCard(sent_data);
          stop_signal = true;
        }
      }

    // read returned parameter
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdTxSdo) {
      EndpointMsg_t endpoint_msg;
      ODrive_CAN.GetEndpointResponse(endpoint_msg, msg);
      
    // read active_error
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetError) {
      ErrorMsg_t error_msg;
      ODrive_CAN.GetErrorResponse(error_msg, msg);

    // read position/velocity
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetEncoderEstimates) {
      EncoderEstimatesMsg_t encoder_msg;
      ODrive_CAN.GetPositionVelocityResponse(encoder_msg, msg);
      motors[node_id].states_.pos_abs = encoder_msg.Pos_Estimate;
      motors[node_id].states_.pos_rel = motors[node_id].states_.pos_abs - motors[node_id].states_.pos_home;
      motors[node_id].states_.vel = encoder_msg.Vel_Estimate;
      motors[node_id].states_.q = motors[node_id].params_.direction * motors[node_id].params_.T * motors[node_id].states_.pos_rel;
      motors[node_id].states_.q_dot = motors[node_id].params_.direction * motors[node_id].params_.T * motors[node_id].states_.vel;

    // read motor current
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetIq) {
      IqMsg_t Iq_msg;
      ODrive_CAN.GetIqResponse(Iq_msg, msg);
      motors[node_id].states_.current = Iq_msg.Iq_Measured;

    // read motor torque
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetTorques) {
      TorqueMsg_t torque_msg;
      ODrive_CAN.GetTorquesResponse(torque_msg, msg);
      motors[node_id].states_.torque = torque_msg.Torque_Estimate;
      motors[node_id].states_.torque_d = torque_msg.Torque_Target;

    // read temperature
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetTemperature) {
      TemperatureMsg_t temp_msg;
      ODrive_CAN.GetTemperatureResponse(temp_msg, msg);
      motors[node_id].states_.fet_temp = temp_msg.FET_Temperature;
      motors[node_id].states_.motor_temp = temp_msg.Motor_Temperature;

      if (motors[node_id].states_.motor_temp > kTemperatureLegMotorMax) {
        snprintf(sent_data, sizeof(sent_data), "Motor %d over %.1f degrees Celsius.\n\n", node_id + 1, kTemperatureLegMotorMax);
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
      motors[node_id].states_.bus_voltage = bus_vi_msg.Bus_Voltage;
      motors[node_id].states_.bus_current = bus_vi_msg.Bus_Current;
      
    }
  }
  return msg_id;
}

// extracts and returns the axis/node ID from the ODrive's CAN message
uint8_t getNodeID(uint32_t msg_id) {
  return (msg_id & ((int)(pow(2, kAxisIDLength)) - 1) << kCmdIDLength) >> kCmdIDLength; // node ID is the 6 highest bits
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
    uint8_t node_id = getNodeID(msg_id);
    uint8_t cmd_id = getCmdID(msg_id);
    if ((node_id == axis) & (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdHeartbeat)) {
      return;
    }
  }
}

void readHeartbeat(uint8_t node_id, CAN_message_t msg) {
  HeartbeatMsg_t heartbeat_msg;
  ODrive_CAN.Heartbeat(heartbeat_msg, msg);
  motors[node_id].states_.axis_state = heartbeat_msg.Axis_State;
  motors[node_id].states_.axis_error = heartbeat_msg.Axis_Error;
}

// looks for the ODrive CAN heartbeat msg and enters closed-loop control
void initActuators() {
  elapsedMillis t_wait;

  // wait for the first CAN message to come in
  while (!handleODriveCANMsg()) {
    if (t_wait > 5000) {
      snprintf(sent_data, sizeof(sent_data), "Waiting for ODrives to power on...\n");
      writeToSerial();
      t_wait = 0;
    }
  }
  
  uint32_t t_start = millis();
  while(millis() - t_start < 1000) {
    handleODriveCANMsg();
  }
  
  // powering on the motor controllers after the logic leads to ODRIVE_ERROR_INITIALIZING, which can be cleared
  for (uint8_t idx_motor = 0; idx_motor < kNumOfActuators; ++idx_motor) {
    ODrive_CAN.ClearErrors(idx_motor);
  }

  for (uint8_t idx_motor = 0; idx_motor < kNumOfActuators; ++idx_motor) {
    snprintf(sent_data, sizeof(sent_data), "Checking ODrive axis %d...\n", idx_motor + 1);
    writeToSerial();

    waitForHeartbeat(idx_motor);
    snprintf(sent_data, sizeof(sent_data), "Current state: %d\tAxis error: %lu\n\n", motors[idx_motor].states_.axis_state, motors[idx_motor].states_.axis_error);
    writeToSerial();

    if (motors[idx_motor].states_.axis_error) {
      digitalWrite(LED, HIGH);

      snprintf(sent_data, sizeof(sent_data), "Stopping. Check the axis error.\n");
      writeToSerial();
      
      while (true) {} // if there's an axis error, do not proceed
      
    } else if (motors[idx_motor].states_.bus_voltage < kMinBatteryVoltage) {
      digitalWrite(LED, HIGH);

      snprintf(sent_data, sizeof(sent_data), "Battery voltage below %.1f.\n", kMinBatteryVoltage);
      writeToSerial();
      
      while (true) {} // if there's an axis error, do not proceed

    } else {
      snprintf(sent_data, sizeof(sent_data), "Putting actuator %d in closed-loop control...\n", idx_motor + 1);
      writeToSerial();
      motors[idx_motor].enable();
    }
  }

  snprintf(sent_data, sizeof(sent_data), "All actuators in closed-loop control.\n---------------------------------------------\n\n");
  writeToSerial();
}

// enters idle state on all axes
void stopActuators() {
  for (uint8_t idx_motor = 0; idx_motor < kNumOfActuators; ++idx_motor) {
    snprintf(sent_data, sizeof(sent_data), "Stopping actuator %d...\n", idx_motor + 1);
    writeToSerial();
    motors[idx_motor].disable();
  }

  snprintf(sent_data, sizeof(sent_data), "Actuators stopped.\n---------------------------------------------\n\n");
  writeToSerial();
}

void updateMotorTorque(uint8_t idx_motor, float torque, float vel_limit) {
  motors[idx_motor].states_.ctrl_mode = ODriveTeensyCAN::ControlMode_t::kTorqueControl;
  motors[idx_motor].states_.tau_d = torque;
  if (vel_limit > 0) {motors[idx_motor].states_.velocity_limit = vel_limit;}
}

// updates motor controller limits on all axes
void updateMotorLimits() {
  uint32_t t_current = millis();
  if (t_current - t_last_motor_limits_update >= k_dtMotorLimitsUpdate) {
    t_last_motor_limits_update = t_current;
    
    #ifdef DEBUG_TIMER
    elapsedMicros timer;
    #endif

    for (uint8_t idx_motor = 0; idx_motor < kNumOfActuators; ++idx_motor) {

      // update motor and controller limits
      ODrive_CAN.SetLimits(idx_motor, motors[idx_motor].states_.velocity_limit, motors[idx_motor].states_.current_limit);
      ODrive_CAN.SetTrajVelLimit(idx_motor, motors[idx_motor].states_.trap_traj_vel_limit);
      ODrive_CAN.SetTrajAccelLimits(idx_motor, motors[idx_motor].states_.trap_traj_accel_limit, motors[idx_motor].states_.trap_traj_decel_limit);
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

    for (uint8_t idx_motor = 0; idx_motor < kNumOfActuators; ++idx_motor) {

      // update motor and controller limits
      ODrive_CAN.SetLimits(idx_motor, motors[idx_motor].states_.velocity_limit, motors[idx_motor].states_.current_limit);
      ODrive_CAN.SetTrajVelLimit(idx_motor, motors[idx_motor].states_.trap_traj_vel_limit);
      ODrive_CAN.SetTrajAccelLimits(idx_motor, motors[idx_motor].states_.trap_traj_accel_limit, motors[idx_motor].states_.trap_traj_decel_limit);
      motors[idx_motor].setControlMode(motors[idx_motor].states_.ctrl_mode);
    }

    delayMicroseconds(250); // give ODrive some time to process the limit changes; 100 us too low

    for (uint8_t idx_motor = 0; idx_motor < kNumOfActuators; ++idx_motor) {
      // update motor command
      if (motors[idx_motor].states_.ctrl_mode == ODriveTeensyCAN::ControlMode_t::kPositionControl) {  // if in position control
        motors[idx_motor].sendCommand(ODriveTeensyCAN::ControlMode_t::kPositionControl, motors[idx_motor].states_.q_d);
        motors[idx_motor].states_.holding = fabs(motors[idx_motor].states_.q - motors[idx_motor].states_.q_d) < kQErrorMax;

      } else if (motors[idx_motor].states_.ctrl_mode == ODriveTeensyCAN::ControlMode_t::kTorqueControl) {  // if in torque control
        motors[idx_motor].sendCommand(ODriveTeensyCAN::ControlMode_t::kTorqueControl, motors[idx_motor].states_.tau_d);

      } else if (motors[idx_motor].states_.ctrl_mode == ODriveTeensyCAN::ControlMode_t::kVelocityControl) {  // if in velocity control
        motors[idx_motor].sendCommand(ODriveTeensyCAN::ControlMode_t::kVelocityControl, motors[idx_motor].states_.q_dot_d);
      }
    }

    #ifdef DEBUG_TIMER
    SERIAL_USB.print("updating motor commands took\t\t");
    SERIAL_USB.print(timer);
    SERIAL_USB.print(" microseconds\n");
    #endif

  }
}

