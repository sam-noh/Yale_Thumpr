#include "Arduino.h"
#include "ODriveTeensyCAN.h"

static const int NodeIDLength = 6;
static const int CommandIDLength = 5;

static const float feedforwardFactor = 1 / 0.001;

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

FlexCAN_T4<CAN1, RX_SIZE_512, TX_SIZE_128> myCAN;    // CAN1, RX_SIZE_512, TX_SIZE_128

ODriveTeensyCAN::ODriveTeensyCAN(int CANBaudRate) {
    this->CANBaudRate = CANBaudRate;
	myCAN.begin();
    myCAN.setBaudRate(CANBaudRate);
    myCAN.enableFIFO();
}

void ODriveTeensyCAN::sendMessage(int axis_id, int cmd_id, bool remote_transmission_request, int length, byte *signal_bytes) {
    CAN_message_t msg;

    msg.id = (axis_id << CommandIDLength) + cmd_id;
    msg.flags.remote = remote_transmission_request;
    msg.len = length;
    if(!remote_transmission_request) {
        memcpy(msg.buf, signal_bytes, length);
        myCAN.write(msg);
        return;
    }

	myCAN.write(msg);
}

bool ODriveTeensyCAN::ReadMsg(CAN_message_t& inMsg) {
	if(myCAN.read(inMsg)) {
		return true;
	} else {
		return false;
	}
}

// void ODriveTeensyCAN::RxSdo(int axis_id, uint16 endpoint_id) {
// 	byte* node_id_b = (byte*) &node_id;
	
// 	sendMessage(axis_id, kCmdIdRxSdo, false, 4, node_id_b);
// }

void ODriveTeensyCAN::Heartbeat(HeartbeatMsg_t &returnVals, CAN_message_t &inMsg) {
	returnVals.parseMessage(inMsg);
}

void ODriveTeensyCAN::SetAxisNodeId(int axis_id, int node_id) {
	byte* node_id_b = (byte*) &node_id;
	
	sendMessage(axis_id, kCmdIdSetAxisNodeID, false, 4, node_id_b);
}

void ODriveTeensyCAN::SetControllerModes(int axis_id, int control_mode, int input_mode) {
	byte* control_mode_b = (byte*) &control_mode;
	byte* input_mode_b = (byte*) &input_mode;
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	msg_data[0] = control_mode_b[0];
	msg_data[1] = control_mode_b[1];
	msg_data[2] = control_mode_b[2];
	msg_data[3] = control_mode_b[3];	
	msg_data[4] = input_mode_b[0];
	msg_data[5] = input_mode_b[1];
	msg_data[6] = input_mode_b[2];
	msg_data[7] = input_mode_b[3];
	
	sendMessage(axis_id, kCmdIdSetControllerMode, false, 8, msg_data);
}

void ODriveTeensyCAN::SetPosition(int axis_id, float position) {
    SetPosition(axis_id, position, 0.0f, 0.0f);
}

void ODriveTeensyCAN::SetPosition(int axis_id, float position, float velocity_feedforward) {
    SetPosition(axis_id, position, velocity_feedforward, 0.0f);
}

void ODriveTeensyCAN::SetPosition(int axis_id, float position, float velocity_feedforward, float current_feedforward) {
    int16_t vel_ff = (int16_t) (feedforwardFactor * velocity_feedforward);
    int16_t curr_ff = (int16_t) (feedforwardFactor * current_feedforward);

    byte* position_b = (byte*) &position;
    byte* velocity_feedforward_b = (byte*) &vel_ff;
    byte* current_feedforward_b = (byte*) &curr_ff;
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg_data[0] = position_b[0];
    msg_data[1] = position_b[1];
    msg_data[2] = position_b[2];
    msg_data[3] = position_b[3];
    msg_data[4] = velocity_feedforward_b[0];
    msg_data[5] = velocity_feedforward_b[1];
    msg_data[6] = current_feedforward_b[0];
    msg_data[7] = current_feedforward_b[1];

    sendMessage(axis_id, kCmdIdSetInputPos, false, 8, msg_data);
}

void ODriveTeensyCAN::SetVelocity(int axis_id, float velocity) {
    SetVelocity(axis_id, velocity, 0.0f);
}

void ODriveTeensyCAN::SetVelocity(int axis_id, float velocity, float current_feedforward) {
    byte* velocity_b = (byte*) &velocity;
    byte* current_feedforward_b = (byte*) &current_feedforward;
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg_data[0] = velocity_b[0];
    msg_data[1] = velocity_b[1];
    msg_data[2] = velocity_b[2];
    msg_data[3] = velocity_b[3];
    msg_data[4] = current_feedforward_b[0];
    msg_data[5] = current_feedforward_b[1];
    msg_data[6] = current_feedforward_b[2];
    msg_data[7] = current_feedforward_b[3];
    
    sendMessage(axis_id, kCmdIdSetInputVel, false, 8, msg_data);
}

void ODriveTeensyCAN::SetTorque(int axis_id, float torque) {
    byte* torque_b = (byte*) &torque;

    sendMessage(axis_id, kCmdIdSetInputTorque, false, 4, torque_b);
}

void ODriveTeensyCAN::SetLimits(int axis_id, float velocity_limit, float current_limit) {
    byte* velocity_limit_b = (byte*) &velocity_limit;
	byte* current_limit_b = (byte*) &current_limit;
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg_data[0] = velocity_limit_b[0];
    msg_data[1] = velocity_limit_b[1];
    msg_data[2] = velocity_limit_b[2];
    msg_data[3] = velocity_limit_b[3];
    msg_data[4] = current_limit_b[0];
    msg_data[5] = current_limit_b[1];
    msg_data[6] = current_limit_b[2];
    msg_data[7] = current_limit_b[3];

    sendMessage(axis_id, kCmdIdSetLimits, false, 8, msg_data);
}

void ODriveTeensyCAN::SetTrajVelLimit(int axis_id, float traj_vel_limit) {
    byte* traj_vel_limit_b = (byte*) &traj_vel_limit;

    sendMessage(axis_id, kCmdIdSetTrajVelLimit, false, 4, traj_vel_limit_b);
}

void ODriveTeensyCAN::SetTrajAccelLimits(int axis_id, float traj_accel_limit, float traj_decel_limit) {
	byte* traj_accel_limit_b = (byte*) &traj_accel_limit;
	byte* traj_decel_limit_b = (byte*) &traj_decel_limit;
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	msg_data[0] = traj_accel_limit_b[0];
	msg_data[1] = traj_accel_limit_b[1];
	msg_data[2] = traj_accel_limit_b[2];
	msg_data[3] = traj_accel_limit_b[3];
	msg_data[4] = traj_decel_limit_b[0];
	msg_data[5] = traj_decel_limit_b[1];
	msg_data[6] = traj_decel_limit_b[2];
	msg_data[7] = traj_decel_limit_b[3];
	
	sendMessage(axis_id, kCmdIdSetTrajAccelLimits, false, 8, msg_data);
}

void ODriveTeensyCAN::SetTrajInertia(int axis_id, float traj_inertia) {
    byte* traj_inertia_b = (byte*) &traj_inertia;

    sendMessage(axis_id, kCmdIdSetTrajInertia, false, 4, traj_inertia_b);
}

void ODriveTeensyCAN::SetAbsolutePosition(int axis_id, float abs_position) {
    byte* abs_position_b = (byte*) &abs_position;

    sendMessage(axis_id, kCmdIdSetAbsolutePosition, false, 4, abs_position_b);
}

void ODriveTeensyCAN::SetPositionGain(int axis_id, float position_gain) {
    byte* position_gain_b = (byte*) &position_gain;

    sendMessage(axis_id, kCmdIdSetPosGain, false, 4, position_gain_b);
}

void ODriveTeensyCAN::SetVelocityGains(int axis_id, float velocity_gain, float velocity_integrator_gain) {
    byte* velocity_gain_b = (byte*) &velocity_gain;
	byte* velocity_integrator_gain_b = (byte*) &velocity_integrator_gain;
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg_data[0] = velocity_gain_b[0];
    msg_data[1] = velocity_gain_b[1];
    msg_data[2] = velocity_gain_b[2];
    msg_data[3] = velocity_gain_b[3];
    msg_data[4] = velocity_integrator_gain_b[0];
    msg_data[5] = velocity_integrator_gain_b[1];
    msg_data[6] = velocity_integrator_gain_b[2];
    msg_data[7] = velocity_integrator_gain_b[3];

    sendMessage(axis_id, kCmdIdSetVelGains, false, 8, msg_data);
}

//////////// Get functions ///////////

void ODriveTeensyCAN::GetPositionVelocity(int axis_id) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
    sendMessage(axis_id, kCmdIdGetEncoderEstimates, true, 8, msg_data);
}

void ODriveTeensyCAN::GetPositionVelocityResponse(EncoderEstimatesMsg_t &returnVal, CAN_message_t &inMsg) {
	returnVal.parseMessage(inMsg);
}

void ODriveTeensyCAN::GetIq(int axis_id) {
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(axis_id, kCmdIdGetIq, true, 8, msg_data);
}

void ODriveTeensyCAN::GetIqResponse(IqMsg_t &returnVal, CAN_message_t &inMsg) {
	returnVal.parseMessage(inMsg);
}

void ODriveTeensyCAN::GetTemperature(int axis_id) {
	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(axis_id, kCmdIdGetTemperature, true, 8, msg_data);
}

void ODriveTeensyCAN::GetTemperatureResponse(TemperatureMsg_t &returnVal, CAN_message_t &inMsg) {
	returnVal.parseMessage(inMsg);
}

void ODriveTeensyCAN::GetError(int axis_id) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(axis_id, kCmdIdGetError, true, 8, msg_data);
}

void ODriveTeensyCAN::GetErrorResponse(ErrorMsg_t &returnVal, CAN_message_t &inMsg) {
    returnVal.parseMessage(inMsg);
}

void ODriveTeensyCAN::GetBusVoltageCurrent(int axis_id) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(axis_id, kCmdIdGetBusVoltageCurrent, true, 8, msg_data);
}

void ODriveTeensyCAN::GetBusVoltageCurrentResponse(BusViMsg_t &returnVal, CAN_message_t &inMsg) {
    returnVal.parseMessage(inMsg);
}

void ODriveTeensyCAN::GetTorques(int axis_id) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
    sendMessage(axis_id, kCmdIdGetTorques, true, 8, msg_data);
}

void ODriveTeensyCAN::GetTorquesResponse(TorqueMsg_t &returnVal, CAN_message_t &inMsg) {
    returnVal.parseMessage(inMsg);
}

//////////// Other functions ///////////

void ODriveTeensyCAN::Estop(int axis_id) {
    sendMessage(axis_id, kCmdIdEstop, false, 0, 0);  //message requires no data, thus the 0, 0
}
void ODriveTeensyCAN::RebootOdrive(int axis_id) {
    sendMessage(axis_id, kCmdIdReboot, false, 0, 0);
}
void ODriveTeensyCAN::ClearErrors(int axis_id) {
    sendMessage(axis_id, kCmdIdClearErrors, false, 0, 0);  //message requires no data, thus the 0, 0
}

//////////// State helper ///////////

bool ODriveTeensyCAN::RunState(int axis_id, int requested_state) {
    sendMessage(axis_id, kCmdIdSetAxisState, false, 4, (byte*) &requested_state);
    return true;
}

std::string ODriveTeensyCAN::error_to_string(uint32_t err) {
switch(err) {
    case ODRIVE_ERROR_NONE:
    return "ODRIVE_ERROR_NONE";
    case ODRIVE_ERROR_INITIALIZING:
    return "ODRIVE_ERROR_INITIALIZING";
    case ODRIVE_ERROR_SYSTEM_LEVEL:
    return "ODRIVE_ERROR_SYSTEM_LEVEL";
    case ODRIVE_ERROR_TIMING_ERROR:
    return "ODRIVE_ERROR_TIMING_ERROR";
    case ODRIVE_ERROR_MISSING_ESTIMATE:
    return "ODRIVE_ERROR_MISSING_ESTIMATE";
    case ODRIVE_ERROR_BAD_CONFIG:
    return "ODRIVE_ERROR_BAD_CONFIG";
    case ODRIVE_ERROR_DRV_FAULT:
    return "ODRIVE_ERROR_DRV_FAULT";
    case ODRIVE_ERROR_MISSING_INPUT:
    return "ODRIVE_ERROR_MISSING_INPUT";
    case ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE:
    return "ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE";
    case ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE:
    return "ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE";
    case ODRIVE_ERROR_DC_BUS_OVER_CURRENT:
    return "ODRIVE_ERROR_DC_BUS_OVER_CURRENT";
    case ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT:
    return "ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT";
    case ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION:
    return "ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION";
    case ODRIVE_ERROR_MOTOR_OVER_TEMP:
    return "ODRIVE_ERROR_MOTOR_OVER_TEMP";
    case ODRIVE_ERROR_INVERTER_OVER_TEMP:
    return "ODRIVE_ERROR_INVERTER_OVER_TEMP";
    case ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION:
    return "ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION";
    case ODRIVE_ERROR_POSITION_LIMIT_VIOLATION:
    return "ODRIVE_ERROR_POSITION_LIMIT_VIOLATION";
    case ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED:
    return "ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED";
    case ODRIVE_ERROR_ESTOP_REQUESTED:
    return "ODRIVE_ERROR_ESTOP_REQUESTED";
    case ODRIVE_ERROR_SPINOUT_DETECTED:
    return "ODRIVE_ERROR_SPINOUT_DETECTED";
    case ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED:
    return "ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED";
    case ODRIVE_ERROR_THERMISTOR_DISCONNECTED:
    return "ODRIVE_ERROR_THERMISTOR_DISCONNECTED";
    case ODRIVE_ERROR_CALIBRATION_ERROR:
    return "ODRIVE_ERROR_CALIBRATION_ERROR";
    default:
    return "ODRIVE_ERROR_UNKNOWN";
}
}
