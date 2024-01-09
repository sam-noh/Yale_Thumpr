#include "state_estimation.h"
#include "..\..\include\joint_defs.h"
#include "..\motor_control\motor_control.h"
#include "..\locomotion\locomotion.h"

// hardware interrupt encoders
std::vector<Encoder> encoders = {Encoder(ENC_1_A, ENC_1_B),   // medial body; front right leg
                                 Encoder(ENC_2_A, ENC_2_B),   // medial body; front left leg
                                 Encoder(ENC_3_A, ENC_3_B),   // medial body; rear right leg
                                 Encoder(ENC_4_A, ENC_4_B),   // medial body; rear left leg
                                 Encoder(ENC_5_A, ENC_5_B),   // lateral body; front right leg
                                 Encoder(ENC_6_A, ENC_6_B),   // lateral body; rear right leg
                                 Encoder(ENC_7_A, ENC_7_B),   // lateral body; front left leg
                                 Encoder(ENC_8_A, ENC_8_B)    // lateral body; rear left leg
                                 }; 

// moving average filter for motor torques
std::vector<MovingAvgFilter> motor_torque_filters = {MovingAvgFilter(kTorqueFilterLength),
                                                     MovingAvgFilter(kTorqueFilterLength),
                                                     MovingAvgFilter(kTorqueFilterLength),
                                                     MovingAvgFilter(kTorqueFilterLength),
                                                     MovingAvgFilter(kTorqueFilterLength),
                                                     MovingAvgFilter(kTorqueFilterLength)};

TwoAxisJoystick two_axis_joystick;                                   // 2-axis joystick with select button
BNO08X_IMU bno08x_imu;                                        // BNO08x 9-DoF IMU
ACS711EX_Sensor acs711ex_sensor;                              // ACS711EX current sensor

std::vector<float> q = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};      // see JointID for joint indices
std::vector<float> q_dot = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // see JointID for joint indices
float z_body_local = 0;                                     // height of body above local terrain
std::vector<float> rpy_medial = {0, 0, 0};                  // medial body orientation estimated by IMU

bool stop_signal = false;

// read the joystick XY analog voltages and the select button and normalize
void TwoAxisJoystick::readJoystick() {
  x_raw = analogRead(XOUT);
  y_raw = analogRead(YOUT);
  checkStopButton();
  normalizeJoystick();
}

// normalize joystick position based on deadzones and calibrated center
void TwoAxisJoystick::normalizeJoystick() {
  x_norm = max(abs(x_raw - kJoystickXCenter) - kJoystickXDeadZone / 2, 0) / kJoystickXMax;
  y_norm = max(abs(y_raw - kJoystickYCenter) - kJoystickYDeadZone / 2, 0) / kJoystickYMax;

  int dir_1 = (x_raw > kJoystickXCenter) - (x_raw < kJoystickXCenter);
  int dir_2 = (y_raw > kJoystickYCenter) - (y_raw < kJoystickYCenter);
  if (fabs(x_norm) < 0.01) {
    dir_1 = 1;
  }
  if (fabs(y_norm) < 0.01) {
    dir_2 = 1;
  }

  x_norm = dir_1 * x_norm;
  y_norm = dir_2 * y_norm;
}

void TwoAxisJoystick::checkStopButton() {
  joystick_select = digitalRead(SEL);
  
  if (!joystick_select) {
    uint8_t hold_count = 0;
    for(uint8_t i = 0; i < 5; i++) {
      joystick_select = digitalRead(SEL);
      if (!joystick_select) {
        ++hold_count;
      }
    }
    if (hold_count > 3) {
      SERIAL_USB.println("Stop button pressed.");
      stop_signal = true;
    }

    SERIAL_USB.println("End.");
    digitalWrite(LED, HIGH);
  }
}

// initialize BNO08x IMU and set desired reports
void BNO08X_IMU::initIMU(TwoWire *wire) {
  wire->begin();
  if (!bno08x.begin_I2C(0x4A, wire, 0)) {      // begin_I2C(uint8_t i2c_addr = BNO08x_I2CADDR_DEFAULT, TwoWire *wire = &Wire, int32_t sensor_id = 0);
    digitalWrite(LED, HIGH);
    snprintf(sent_data, sizeof(sent_data), "Couldn't find BNO08x chip.\n");
    transmitMsg();
    while (true) {}
  } else {
    bno08x.wasReset();
    snprintf(sent_data, sizeof(sent_data), "BNO08x successfully connected.\n\n");
    transmitMsg();
  }

  snprintf(sent_data, sizeof(sent_data), "Setting desired reports.\n\n");
  transmitMsg();
  if (! bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    snprintf(sent_data, sizeof(sent_data), "Could not enable rotation vector.\n\n");
    transmitMsg();
    while (true) {}
  }

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    snprintf(sent_data, sizeof(sent_data), "Could not enable gyroscope.\n\n");
    transmitMsg();
    while (true) {}
  }

  snprintf(sent_data, sizeof(sent_data), "BNO08x initialized.\n---------------------------------------------\n\n");
  transmitMsg();
}

// fetch IMU data updates and raise stop flag is IMU is disconnected
void BNO08X_IMU::readIMU() {
  if (bno08x.wasReset()) {  // if the sensor was reset, pose estimation has drifted. stop the robot
    snprintf(sent_data, sizeof(sent_data), "IMU was reset.\n");
    transmitMsg();
    stop_signal = true;

    if (! bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      snprintf(sent_data, sizeof(sent_data), "IMU could not be reenabled.\n");
      transmitMsg();
    }
  }

  if (bno08x.getSensorEvent(&imu_sensor_value)) {
    switch (imu_sensor_value.sensorId) {
      case SH2_ROTATION_VECTOR:
        quat[0] = imu_sensor_value.un.rotationVector.real;
        quat[1] = imu_sensor_value.un.rotationVector.i;
        quat[2] = imu_sensor_value.un.rotationVector.j;
        quat[3] = imu_sensor_value.un.rotationVector.k;
        quat_accuracy = imu_sensor_value.un.rotationVector.accuracy;
        quat2rpy();
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        gyro[0] = imu_sensor_value.un.gyroscope.x;
        gyro[1] = imu_sensor_value.un.gyroscope.y;
        gyro[2] = imu_sensor_value.un.gyroscope.z;
        break;
    }
  }
}

// convert quaternion to roll pitch yaw
void BNO08X_IMU::quat2rpy() {
  float sinr_cosp = 2 * (quat[0] * quat[1] + quat[2] * quat[3]);
  float cosr_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
  rpy_medial[0] = atan2(sinr_cosp, cosr_cosp) * 180 / PI;

  float sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
  if (abs(sinp) >= 1) {
    rpy_medial[1] = 90;
    if (sinp < 0) {
      rpy_medial[1] *= -1;
    }
  } else {
    rpy_medial[1] = asin(sinp) * 180 / PI;
  }

  float siny_cosp = 2 * (quat[0] * quat[3] + quat[1] * quat[2]);
  float cosy_cosp = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]);
  rpy_medial[2] = atan2(siny_cosp, cosy_cosp) * 180 / PI;
}

// update current sensor estimate
void ACS711EX_Sensor::readCurrentSensor() {
  ACS711EX_fault = digitalRead(ACS711EX_FAULT);
  if (!ACS711EX_fault) {
    snprintf(sent_data, sizeof(sent_data), "Battery output current exceeded 31A.\n");
    transmitMsg();
    stop_signal = true;
  } else {
    ACS711EX_Vout = analogRead(ACS711EX_OUT);
    current_measured = (3.3 * ACS711EX_Vout / 1023 - 3.3 / 2) * 1000 / k_mVPerAmpere; // convert counts to voltage, then to current
  }
}

MovingAvgFilter::MovingAvgFilter(uint8_t size) {
  std::queue<float> tmp( ( std::deque<float>(size) ) );
  filter_queue = tmp;
}

void MovingAvgFilter::updateFilter(float val) {
    queue_sum -= filter_queue.front();
    filter_queue.pop();
    filter_queue.push(val);
    queue_sum += val;
    filtered_value = queue_sum / kTorqueFilterLength;
}

// update robot states and sensor feedback
void updateStates() {
  uint32_t t_current = millis();

  if (t_current - t_last_state_update >= k_dtStateUpdate) {
    t_last_state_update = t_current;
    readJointEncoders();
    two_axis_joystick.readJoystick();
    bno08x_imu.readIMU();
    acs711ex_sensor.readCurrentSensor();
    updateMotorTorqueFilters();
    updateKinematics();
  }
}

// update robot joint positions and velocities from encoder feedback
void readJointEncoders() {
  // assume that this function runs fast enough that
  // the same time period can be used to calculate all joints' velocity estimates
  uint32_t t_current = millis();
  uint8_t dt = t_current - t_last_encoder_update;  
  t_last_encoder_update = t_current;

  float q_prev;

  // read leg encoders and update position and velocity estimates
  for (uint8_t i = 0; i < kNumOfLegs; i++) {
    q_prev = q[i];
    q[i] = kTxRatioLegDrive * ((float)encoders[i].read() / k_CPR_AMT102_V);
    q_dot[i] = (q[i] - q_prev) / dt;
  }

  // update translation and yaw joint position and velocity estimates
  q[kJointTranslate] = motors[kMotorTranslate].states_.q;
  q_dot[kJointTranslate] = motors[kMotorTranslate].states_.q_dot;

  q[kJointYaw] = motors[kMotorYaw].states_.q;
  q_dot[kJointYaw] = motors[kMotorYaw].states_.q_dot;
}

// call the updateFilter() function for all the moving average filters
void updateMotorTorqueFilters() {
  for (uint8_t i = 0; i < kNumOfActuators; i++) {
    motor_torque_filters[i].updateFilter(motors[i].states_.tau);
  }
}

// update robot body pose based on kinematics
void updateKinematics() {
  uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;
  z_body_local = (motors[stance * 2].states_.q + motors[stance * 2 + 1].states_.q) / 2 - gait_phase * kBodyZOffset;
}
