#ifndef ODriveTeensyCAN_h
#define ODriveTeensyCAN_h

#include "Arduino.h"
#include <FlexCAN_T4.h>
#include <string>

// HeartbeatMsg_t struct defintion
struct HeartbeatMsg_t {
    uint32_t Axis_Error = 0;
    uint8_t Axis_State = 0;
    uint8_t Procedure_Result = 0;
    uint8_t Trajectory_Done_Flag = 0;

    // Member function that takes a CAN_message_t (or could just do a raw buf[8])
    void parseMessage(const CAN_message_t &inMsg) {
        memcpy(&(Axis_Error), &inMsg.buf[0], 4);
        Axis_State = inMsg.buf[4];
        Procedure_Result = inMsg.buf[5];
        Trajectory_Done_Flag = inMsg.buf[6];

    }
};

struct EncoderEstimatesMsg_t {
	float Pos_Estimate = 0;
	float Vel_Estimate = 0;
	
	void parseMessage(const CAN_message_t &inMsg) {
        memcpy(&(Pos_Estimate), &inMsg.buf[0], 4);
        memcpy(&(Vel_Estimate), &inMsg.buf[4], 4);
	}
};

struct IqMsg_t {
	float Iq_Setpoint = 0;
	float Iq_Measured = 0;
	
	void parseMessage(const CAN_message_t &inMsg) {
        memcpy(&(Iq_Setpoint), &inMsg.buf[0], 4);
        memcpy(&(Iq_Measured), &inMsg.buf[4], 4);
	}
};

struct TemperatureMsg_t {
	float FET_Temperature = 0;
	float Motor_Temperature = 0;
	
	void parseMessage(const CAN_message_t &inMsg) {
        memcpy(&(FET_Temperature), &inMsg.buf[0], 4);
        memcpy(&(Motor_Temperature), &inMsg.buf[4], 4);
	}
};

struct ErrorMsg_t {
    uint32_t Active_Errors = 0;
    uint32_t Disarm_Reason = 0;

    void parseMessage(const CAN_message_t &inMsg) {
        memcpy(&(Active_Errors), &inMsg.buf[0], 4);
        memcpy(&(Disarm_Reason), &inMsg.buf[4], 4);
    }
};

struct BusViMsg_t {
    float Bus_Voltage = 0;
    float Bus_Current = 0;

    void parseMessage(const CAN_message_t &inMsg) {
        memcpy(&(Bus_Voltage), &inMsg.buf[0], 4);
        memcpy(&(Bus_Current), &inMsg.buf[4], 4);
    }
};

struct TorqueMsg_t {
    float Torque_Target = 0;
    float Torque_Estimate = 0;

    void parseMessage(const CAN_message_t &inMsg) {
        memcpy(&(Torque_Target), &inMsg.buf[0], 4);
        memcpy(&(Torque_Estimate), &inMsg.buf[4], 4);
    }
};

class ODriveTeensyCAN {
public:
    enum AxisState_t {
        kAxisStateUndefined = 0,           //<! will fall through to idle
        kAxisStateIdle = 1,                //<! disable PWM and do nothing
        kAxisStateStartupSequence = 2, //<! the actual sequence is defined by the config.startup_... flags
        kAxisStateFullCalibrationSequence = 3,   //<! run all calibration procedures, then idle
        kAxisStateMotorCalibration = 4,   //<! run motor calibration
        kAxisStateSensorlessControl = 5,  //<! run sensorless control
        kAxisStateEncoderIndexSearch = 6, //<! run encoder index search
        kAxisStateEncoderOffsetCalibration = 7, //<! run encoder offset calibration
        kAxisStateClosedLoopControl = 8  //<! run closed loop control
    };
	
	enum ControlMode_t {
		kVoltageControl = 0,
		kTorqueControl = 1,
		kVelocityControl = 2,
		kPositionControl = 3
	};
	
	enum InputMode_t {
		kInactive = 0,
		kPassthrough = 1,
		kVelRamp = 2,
		kPosFilter = 3,
		kMixChannels = 4,
		kTrapTraj = 5,
		kTorqueRamp = 6,
		kMirror = 7,
		kTuning = 8
	};

    enum CommandId_t {
        kCmdIdGetVersion = 0x000,
        kCmdIdHeartbeat = 0x001,
        kCmdIdEstop = 0x002,
        kCmdIdGetError = 0x003,
        kCmdIdRxSdo = 0x004,
        kCmdIdTxSdo = 0x005,
        kCmdIdSetAxisNodeID = 0x006,
        kCmdIdSetAxisState = 0x007,
        kCmdIdGetEncoderEstimates = 0x009,
        kCmdIdSetControllerMode = 0x00B,
        kCmdIdSetInputPos = 0x00C,
        kCmdIdSetInputVel = 0x00D,
        kCmdIdSetInputTorque = 0x00E,
		kCmdIdSetLimits = 0x00F,
        kCmdIdSetTrajVelLimit = 0x011,
        kCmdIdSetTrajAccelLimits = 0x012,
        kCmdIdSetTrajInertia = 0x013,
        kCmdIdGetIq = 0x014,
        kCmdIdGetTemperature = 0x015,               // changed from get sensorless estimates to get temp
        kCmdIdReboot = 0x016,
        kCmdIdGetBusVoltageCurrent = 0x017,
        kCmdIdClearErrors = 0x018,
		kCmdIdSetAbsolutePosition = 0x019,
		kCmdIdSetPosGain = 0x01A,
		kCmdIdSetVelGains = 0x01B,
		kCmdIdGetTorques = 0x01C,                   // changed from get adc voltage to get torques
        kCmdIdEnterDFUMode = 0x01F,
    };

    enum ODriveError {
        ODRIVE_ERROR_NONE                        = 0x00000000,
        ODRIVE_ERROR_INITIALIZING                = 0x00000001,
        ODRIVE_ERROR_SYSTEM_LEVEL                = 0x00000002,
        ODRIVE_ERROR_TIMING_ERROR                = 0x00000004,
        ODRIVE_ERROR_MISSING_ESTIMATE            = 0x00000008,
        ODRIVE_ERROR_BAD_CONFIG                  = 0x00000010,
        ODRIVE_ERROR_DRV_FAULT                   = 0x00000020,
        ODRIVE_ERROR_MISSING_INPUT               = 0x00000040,
        ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE         = 0x00000100,
        ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE        = 0x00000200,
        ODRIVE_ERROR_DC_BUS_OVER_CURRENT         = 0x00000400,
        ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT   = 0x00000800,
        ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION     = 0x00001000,
        ODRIVE_ERROR_MOTOR_OVER_TEMP             = 0x00002000,
        ODRIVE_ERROR_INVERTER_OVER_TEMP          = 0x00004000,
        ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION    = 0x00008000,
        ODRIVE_ERROR_POSITION_LIMIT_VIOLATION    = 0x00010000,
        ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED      = 0x01000000,
        ODRIVE_ERROR_ESTOP_REQUESTED             = 0x02000000,
        ODRIVE_ERROR_SPINOUT_DETECTED            = 0x04000000,
        ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED     = 0x08000000,
        ODRIVE_ERROR_THERMISTOR_DISCONNECTED     = 0x10000000,
        ODRIVE_ERROR_CALIBRATION_ERROR           = 0x40000000,
    };

    ODriveTeensyCAN(int CANBaudRate);
	
	int CANBaudRate = 250000;  //250,000 is odrive default

    void sendMessage(int axis_id, int cmd_id, bool remote_transmission_request, int length, byte *signal_bytes);
	
	bool ReadMsg(CAN_message_t& inMsg);
	
	// Heartbeat
	void Heartbeat(HeartbeatMsg_t &returnVals, CAN_message_t &inMsg);

    // Setters
	void SetAxisNodeId(int axis_id, int node_id);
	void SetControllerModes(int axis_id, int control_mode, int input_mode);
    void SetPosition(int axis_id, float position);
    void SetPosition(int axis_id, float position, float velocity_feedforward);
    void SetPosition(int axis_id, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(int axis_id, float velocity);
    void SetVelocity(int axis_id, float velocity, float current_feedforward);
    void SetTorque(int axis_id, float torque);
	void SetLimits(int axis_id, float velocity_limit, float current_limit);
	void SetTrajVelLimit(int axis_id, float traj_vel_limit);
	void SetTrajAccelLimits(int axis_id, float traj_accel_limit, float traj_decel_limit);
	void SetTrajInertia(int axis_id, float traj_inertia);
	void SetAbsolutePosition(int axis_id, float abs_position);
	void SetPositionGain(int axis_id, float position_gain);
	void SetVelocityGains(int axis_id, float velocity_gain, float velocity_integrator_gain);

    // Getters
    void GetPositionVelocity(int axis_id);
    void GetPositionVelocityResponse(EncoderEstimatesMsg_t &returnVal, CAN_message_t &inMsg);
	void GetIq(int axis_id);
	void GetIqResponse(IqMsg_t &returnVal, CAN_message_t &inMsg);
	void GetTemperature(int axis_id);
	void GetTemperatureResponse(TemperatureMsg_t &returnVal, CAN_message_t &inMsg);
    void GetError(int axis_id);
	void GetErrorResponse(ErrorMsg_t &returnVal, CAN_message_t &inMsg);
	void GetBusVoltageCurrent(int axis_id);
	void GetBusVoltageCurrentResponse(BusViMsg_t &returnVal, CAN_message_t &inMsg);
	void GetTorques(int axis_id);
	void GetTorquesResponse(TorqueMsg_t &returnVal, CAN_message_t &inMsg);
	
	// Other functions
	void Estop(int axis_id);
	void RebootOdrive(int axis_id);  //Can be sent to either axis
	void ClearErrors(int axis_id);

    // State helper
    bool RunState(int axis_id, int requested_state);

    std::string error_to_string(uint32_t err);

};

#endif