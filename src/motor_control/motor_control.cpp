#include "motor_control.h"
#include "..\..\include\joint_defs.h"
#include "..\mcu_util\mcu_util.h"

ODriveTeensyCAN ODrive_CAN = ODriveTeensyCAN(kODriveCANBaudRate);

std::vector<Actuator> motors = {Actuator(ODrive_CAN, SERIAL_USB, 0, kDirectionLeg, kTxRatioLeg, k_KV_to_KT / k_KV_D5312S, kQLegMin, kQLegMax),
                                Actuator(ODrive_CAN, SERIAL_USB, 1, kDirectionLeg, kTxRatioLeg, k_KV_to_KT / k_KV_D5312S, kQLegMin, kQLegMax),
                                Actuator(ODrive_CAN, SERIAL_USB, 2, kDirectionLeg, kTxRatioLeg, k_KV_to_KT / k_KV_D5312S, kQLegMin, kQLegMax),
                                Actuator(ODrive_CAN, SERIAL_USB, 3, kDirectionLeg, kTxRatioLeg, k_KV_to_KT / k_KV_D5312S, kQLegMin, kQLegMax),
                                Actuator(ODrive_CAN, SERIAL_USB, 4, kDirectionTrans, kTxRatioTrans, k_KV_to_KT / k_KV_GL60, -kQTransMax, kQTransMax),
                                Actuator(ODrive_CAN, SERIAL_USB, 5, kDirectionYaw, kTxRatioYaw, k_KV_to_KT / k_KV_GL60, -kQYawMax, kQYawMax)
                                };

// reads the next CAN msg from an ODrive CAN node
// calls member variables of Actuator objects
uint32_t handleODriveCANMsg() {
  CAN_message_t msg;
  uint32_t msg_id = 0;  // this value corresponds to node_id = 0, cmd_id = 0
  if (ODrive_CAN.ReadMsg(msg)) {
    msg_id = msg.id;
    uint8_t axis_id = getAxisID(msg_id);
    uint8_t cmd_id = getCmdID(msg_id);

    // read heartbeat message
    if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdHeartbeat) {
      readHeartbeat(axis_id, msg);

      if (motors[axis_id].states_.axis_error) {
        snprintf(sent_data, sizeof(sent_data), "ERROR: Axis %d encountered error %lu.\n", axis_id + 1, motors[axis_id].states_.axis_error);
        transmitMsg();
        writeToCard(sent_data);
        stop_signal = true;
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
      motors[axis_id].states_.torque_setpoint = torque_msg.Torque_Target;

    // read temperature
    } else if (cmd_id == ODriveTeensyCAN::CommandId_t::kCmdIdGetTemperature) {
      TemperatureMsg_t temp_msg;
      ODrive_CAN.GetTemperatureResponse(temp_msg, msg);
      motors[axis_id].states_.fet_temp = temp_msg.FET_Temperature;
      motors[axis_id].states_.motor_temp = temp_msg.Motor_Temperature;
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
  for (uint8_t i = 0; i < kNumOfActuators; i++) {
    snprintf(sent_data, sizeof(sent_data), "Checking ODrive axis %d...\n", i + 1);
    transmitMsg();

    waitForHeartbeat(i);
    snprintf(sent_data, sizeof(sent_data), "Current state: %d\tAxis error: %lu\n\n", motors[i].states_.axis_state, motors[i].states_.axis_error);
    transmitMsg();

    if (motors[i].states_.axis_error) {
      digitalWrite(LED, HIGH);
      snprintf(sent_data, sizeof(sent_data), "Stopping. Check the axis error.\n");
      transmitMsg();
      while (true) {} // if there's an axis error, do not proceed
      
    } else {
      snprintf(sent_data, sizeof(sent_data), "Putting actuator %d in closed-loop control...\n", i + 1);
      transmitMsg();
      motors[i].enable();
    }
  }

  snprintf(sent_data, sizeof(sent_data), "All actuators in closed-loop control.\n---------------------------------------------\n\n");
  transmitMsg();
}

// calls the Estop() function on all axes
void stopActuators() {
  for (uint8_t i = 0; i < kNumOfActuators; i++) {
    snprintf(sent_data, sizeof(sent_data), "Stopping actuator %d...\n", i + 1);
    transmitMsg();
    ODrive_CAN.Estop(i);     // calling the Estop function will raise the error flag; needs ClearErrors(axis_id) or power cycling
  }

  snprintf(sent_data, sizeof(sent_data), "Actuators stopped.\n---------------------------------------------\n\n");
  transmitMsg();
}

// updates motor commands on all axes
void updateMotorCommands() {
  uint32_t t_current = millis();
  if (t_current - t_last_motor_cmd >= k_dtMotorCmd) {
    t_last_motor_cmd = t_current;
    
    for (uint8_t i = 0; i < kNumOfActuators; i++) {
      if (motors[i].states_.ctrl_mode == ODriveTeensyCAN::ControlMode_t::kPositionControl) {  // if in position control
        motors[i].sendCommand(ODriveTeensyCAN::ControlMode_t::kPositionControl, motors[i].states_.q_d);
        motors[i].states_.holding = fabs(motors[i].states_.q - motors[i].states_.q_d) < kQErrorMax;

      } else if (motors[i].states_.ctrl_mode == ODriveTeensyCAN::ControlMode_t::kTorqueControl) {  // if in torque control
        motors[i].sendCommand(ODriveTeensyCAN::ControlMode_t::kTorqueControl, motors[i].states_.tau_d);
      }
    }
  }
}

