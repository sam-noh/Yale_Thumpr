#include "state_estimation.h"

#include <algorithm>
#include "..\..\include\joint_defs.h"
#include "..\mcu_util\mcu_util.h"
#include "..\motor_control\motor_control.h"
#include "..\locomotion\locomotion.h"

// time variables
elapsedMicros dt_last_leg_encoder_update = 0;     // time since last encoder sampling in microseconds 
uint32_t t_last_IMU_update = 0;                   // timestamp in milliseconds at last IMU sampling
uint32_t t_last_contact_update = 0;               // timestamp in milliseconds at last leg ground contact update
uint32_t t_last_power_update = 0;                 // timestamp in milliseconds at last power update
uint32_t t_last_motor_torque_filter_update = 0;   // timestamp in milliseconds at last motor torque filter update
uint32_t t_last_kinematics_update = 0;            // timestamp in milliseconds at last kinematics update

// hardware interrupt encoders
Encoder encoders[] = {Encoder(ENC_1_A, ENC_1_B),  // medial body; front right leg
                      Encoder(ENC_2_A, ENC_2_B),  // medial body; front left leg
                      Encoder(ENC_3_A, ENC_3_B),  // medial body; rear right leg
                      Encoder(ENC_4_A, ENC_4_B),  // medial body; rear left leg
                      Encoder(ENC_6_A, ENC_6_B),  // lateral body; front right leg
                      Encoder(ENC_8_A, ENC_8_B),  // lateral body; rear right leg
                      Encoder(ENC_5_A, ENC_5_B),  // lateral body; front left leg
                      Encoder(ENC_7_A, ENC_7_B),  // lateral body; rear left leg
                     }; 

// moving average filter for motor torques
std::vector<MovingAvgFilter> motor_torque_filters = {MovingAvgFilter(kTorqueFilterLength),
                                                     MovingAvgFilter(kTorqueFilterLength),
                                                     MovingAvgFilter(kTorqueFilterLength),
                                                     MovingAvgFilter(kTorqueFilterLength),
                                                     MovingAvgFilter(kTorqueFilterLength),
                                                     MovingAvgFilter(kTorqueFilterLength)};

TwoAxisJoystick two_axis_joystick;              // 2-axis joystick with select button
BNO08X_IMU bno08x_imu;                          // BNO08x 9-DoF IMU
ACS711EX_Sensor acs711ex_sensor;                // ACS711EX current sensor
float battery_voltage = 0;                      // current battery voltage in voltage
float battery_current = 0;                      // current battery current in ampere
float battery_power = 0;                        // current battery power in watt

std::vector<float> q = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};      // see JointID for joint indices
std::vector<float> q_prev = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // see JointID for joint indices
std::vector<float> q_dot = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // see JointID for joint indices
float z_body_local = 0;                                     // height of body above local terrain


std::vector<float> rpy_lateral_0 = {0, 0, 0};               // lateral body roll pitch yaw after homing
std::vector<float> rpy_lateral = {0, 0, 0};                 // lateral body roll pitch yaw relative to rpy_lateral_0
std::vector<float> omega_lateral = {0, 0, 0};               // lateral body angular velocity with respect to body frame axes

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
      snprintf(sent_data, sizeof(sent_data), "Stop button pressed.");
      writeToSerial();
      stop_signal = true;
    }

    digitalWrite(LED, HIGH);

    snprintf(sent_data, sizeof(sent_data), "End.");
    writeToSerial();
  }
}

// initialize BNO08x IMU and set desired reports
void BNO08X_IMU::initIMU(TwoWire *wire) {
  #ifdef ENABLE_IMU

  uint32_t t_current = millis();
  bool success = false;

  // BNO08X IMU power-on & initialization timeout
  snprintf(sent_data, sizeof(sent_data), "Waiting for BNO08x chip.\n");
  writeToSerial();
  while (!success && millis() - t_current < 1000) {
    if (bno08x.begin_I2C(0x4A, wire, 0)) {  // begin_I2C(uint8_t i2c_addr = BNO08x_I2CADDR_DEFAULT, TwoWire *wire = &Wire, int32_t sensor_id = 0);
      success = true;
      bno08x.wasReset();                    // do NOT delete this line; necessary to clear the reset flag after power on
    }
  }

  if (!success) {      
    digitalWrite(LED, HIGH);

    snprintf(sent_data, sizeof(sent_data), "Couldn't find BNO08x chip.\n");
    writeToSerial();

    while (true) {}

  } else {
    bno08x.wasReset();                    // do NOT delete this line; necessary to clear the reset flag after power on
    snprintf(sent_data, sizeof(sent_data), "BNO08x successfully connected.\n\n");
    writeToSerial();
  }

  snprintf(sent_data, sizeof(sent_data), "Setting desired reports.\n\n");
  writeToSerial();
  if (! bno08x.enableReport(SH2_ROTATION_VECTOR, 2000)) {
    snprintf(sent_data, sizeof(sent_data), "Could not enable rotation vector.\n\n");
    writeToSerial();
    while (true) {}
  }

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 2000)) {
    snprintf(sent_data, sizeof(sent_data), "Could not enable gyroscope.\n\n");
    writeToSerial();
    while (true) {}
  }

  snprintf(sent_data, sizeof(sent_data), "BNO08x initialized.\n---------------------------------------------\n\n");
  writeToSerial();

  #endif
}

// fetch IMU data updates and raise stop flag is IMU is disconnected
void BNO08X_IMU::readIMU() {
  if (bno08x.wasReset()) {  // if the sensor was reset, pose estimation has drifted. stop the robot
    digitalWrite(LED, HIGH);

    snprintf(sent_data, sizeof(sent_data), "ERROR: IMU was reset.\n\n");
    writeToSerial();

    #ifdef ENABLE_SD_CARD
    writeToCard(sent_data);
    #endif

    stop_signal = true;

    if (! bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      snprintf(sent_data, sizeof(sent_data), "IMU could not be reenabled.\n");
      writeToSerial();
      
      #ifdef ENABLE_SD_CARD
      writeToCard(sent_data);
      #endif
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
  rpy[0] = atan2(sinr_cosp, cosr_cosp) * 180 / PI;

  float sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
  if (abs(sinp) >= 1) {
    rpy[1] = 90;
    if (sinp < 0) {
      rpy[1] *= -1;
    }
  } else {
    rpy[1] = asin(sinp) * 180 / PI;
  }

  float siny_cosp = 2 * (quat[0] * quat[3] + quat[1] * quat[2]);
  float cosy_cosp = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]);
  rpy[2] = atan2(siny_cosp, cosy_cosp) * 180 / PI;
}

// return roll pitch yaw estimate transformed to robot body frame
std::vector<float> BNO08X_IMU::getRPY() {
  std::vector<float> tmp = {0, 0, 0};
  tmp[0] = rpy[1];
  tmp[1] = -rpy[0];
  tmp[2] = rpy[2];
  return tmp;
}

// return angular velcotiy estimates transformed to robot body frame
std::vector<float> BNO08X_IMU::getOmega() {
  std::vector<float> tmp = {0, 0, 0};
  tmp[0] = RAD2DEG*gyro[1];
  tmp[1] = RAD2DEG*-gyro[0];
  tmp[2] = RAD2DEG*gyro[2];
  return tmp;
}

// update current sensor estimate
void ACS711EX_Sensor::readCurrentSensor() {
  ACS711EX_fault = digitalRead(ACS711EX_FAULT);
  if (!ACS711EX_fault) {
    snprintf(sent_data, sizeof(sent_data), "Battery output current exceeded 31A.\n");
    writeToSerial();
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

// determine the initial body pose by averaging the IMU roll pitch yaw readings
void zeroIMUReading() {
  snprintf(sent_data, sizeof(sent_data), "Zeroing IMU pose estimate...\n\n");
  writeToSerial();
  
  uint32_t t_last = millis();
  uint8_t i = 0;
  uint8_t j = 0;
  std::vector<float> rpy_sums(3, 0);
  std::vector<float> rpy;
  float dump = 0.3;   // initial readings are sometimes all zeros

  while(i < kIMUAvgSize) {
    handleODriveCANMsg();
    if (millis() - t_last > k_dtIMUUpdate) {
      t_last = millis();

      #ifdef ENABLE_IMU
      bno08x_imu.readIMU();
      rpy = bno08x_imu.getRPY();
      #else
      parseTeensySerial();
      rpy = input_rpy;
      #endif

      if (i > dump*kIMUAvgSize) {
        rpy_sums[0] += rpy[0];
        rpy_sums[1] += rpy[1];
        rpy_sums[2] += rpy[2];
        ++j;
      }
      ++i;
    }
  }

  rpy_lateral_0[0] = rpy_sums[0]/j;
  rpy_lateral_0[1] = rpy_sums[1]/j;
  rpy_lateral_0[2] = rpy_sums[2]/j;

  snprintf(sent_data, sizeof(sent_data), "IMU pose estimate zeroed.\nroll_0 = %.2f\tpitch_0 = %.2f\tyaw_0 = %.2f\n\n", rpy_lateral_0[0], rpy_lateral_0[1], rpy_lateral_0[2]);
  writeToSerial();

  // if the IMU is still sending only zeros, stop code
  if (fabs(rpy_lateral_0[0]) < ESP && fabs(rpy_lateral_0[1]) < ESP && fabs(rpy_lateral_0[2]) < ESP) {
    digitalWrite(LED, HIGH);

    snprintf(sent_data, sizeof(sent_data), "ERROR: IMU is only sending zeros.\n\n");
    writeToSerial();

    #ifdef ENABLE_SD_CARD
    writeToCard(sent_data);
    #endif

    stop_signal = true;
  }
}

// update robot states and sensor feedback
void updateStates() {
  readJointEncoders();
  updateIMUEstimate();
  updateContactState();
  updatePowerMeasurement();
  updateMotorTorqueFilters();
  updateKinematics();
}

// update robot joint positions and velocities from encoder feedback
void readJointEncoders() {

  // this entire function runs in <= 1 microsecond
  // so the same time period can be used to calculate all joints' velocity estimates
  if (dt_last_leg_encoder_update >= k_dtLegEncoderUpdate) {

    // read leg encoders and update position and velocity estimates
    for (uint8_t i = 0; i < kNumOfLegs; i++) {
      q[i] = kTxRatioLegDrive * (kDirectionWorm[i]*(float)encoders[i].read() / k_CPR_AMT102_V);
      q_dot[i] = (q[i] - q_prev[i]) / ((float)dt_last_leg_encoder_update / 1e6);
      q_prev[i] = q[i];
    }

    dt_last_leg_encoder_update = 0;

    // update translation and yaw joint position and velocity estimates
    q[JointID::kJointTranslate] = motors[MotorID::kMotorTranslate].states_.q;
    q_dot[JointID::kJointTranslate] = motors[MotorID::kMotorTranslate].states_.q_dot;

    q[JointID::kJointYaw] = motors[MotorID::kMotorYaw].states_.q;
    q_dot[JointID::kJointYaw] = motors[MotorID::kMotorYaw].states_.q_dot;
  }
}

// fetch IMU reading and update the body orientation relative to the initial orientation
void updateIMUEstimate() {
  uint32_t t_current = millis();
  if (t_current - t_last_IMU_update >= k_dtIMUUpdate) {
    t_last_IMU_update = t_current;

    #ifdef ENABLE_IMU
    bno08x_imu.readIMU();
    rpy_lateral = bno08x_imu.getRPY();
    omega_lateral = bno08x_imu.getOmega();
    #else
    rpy_lateral = input_rpy;
    omega_lateral = input_omega;
    #endif

    std::transform(rpy_lateral.begin(), rpy_lateral.end(), rpy_lateral_0.begin(), rpy_lateral.begin(), std::minus<float>());
  }
  
}

// estimate total power consumption by motors
void updatePowerMeasurement() {
  uint32_t t_current = millis();
  if (t_current - t_last_power_update >= k_dtPowerUpdate) {
    t_last_power_update = t_current;

    battery_voltage = 0;
    battery_current = 0;
    battery_power = 0;

    for(uint8_t axis_id = 0; axis_id < kNumOfActuators; ++axis_id) {
      battery_voltage += motors[axis_id].states_.bus_voltage;
      battery_current += motors[axis_id].states_.bus_current;
      battery_power += motors[axis_id].states_.bus_voltage*motors[axis_id].states_.bus_current;
    }

    battery_voltage /= kNumOfActuators;
  }
}

// estimates the contact state of each swing leg motors
void updateContactState() {
  if (actuation_phase == ActuationPhases::kTouchDown) {   // only check contact state of the swing leg motors during touchdown phase

    uint32_t t_current = millis();
    if (t_current - t_last_contact_update >= k_dtContactUpdate) {
      t_last_contact_update = t_current;

      // use either leg encoder velocity or motor velocity for contact estimation
      #ifdef USE_LEG_CONTACT
      inContact[gait_phase * 2] = fabs(q_dot[gait_phase*4]) < kQdotContact && fabs(q_dot[gait_phase*4 + 1]) < kQdotContact      // if the leg velocities are below a threshold
                                  && motors[gait_phase * 2].states_.q + dz_body_local - q_leg_swing[0] > kDqStartContact;       // AND the actuator position is past some inital displacement

      inContact[gait_phase * 2 + 1] = fabs(q_dot[gait_phase*4 + 2]) < kQdotContact && fabs(q_dot[gait_phase*4 + 3]) < kQdotContact
                                      && motors[gait_phase * 2 + 1].states_.q + dz_body_local - q_leg_swing[1] > kDqStartContact;

      #else
      inContact[gait_phase * 2] = fabs(motors[gait_phase * 2].states_.q_dot) < kQdotContact                                  // if the actuator velocity is below a threshold
                                  && motors[gait_phase * 2].states_.q + dz_body_local - q_leg_swing[0] > kDqStartContact;    // AND the actuator position is past some inital displacement

      inContact[gait_phase * 2 + 1] = fabs(motors[gait_phase * 2 + 1].states_.q_dot) < kQdotContact
                                      && motors[gait_phase * 2 + 1].states_.q + dz_body_local - q_leg_swing[1] > kDqStartContact;
      #endif
    }
  }
}

// call the updateFilter() function for all the moving average filters
void updateMotorTorqueFilters() {
  uint32_t t_current = millis();
  if (t_current - t_last_motor_torque_filter_update >= k_dtMotorTorqueFilterUpdate) {
    t_last_motor_torque_filter_update = t_current;

    for (uint8_t i = 0; i < kNumOfActuators; i++) {
      motor_torque_filters[i].updateFilter(motors[i].states_.torque);
    }
  }
}

// update robot body pose based on kinematics
void updateKinematics() {
  uint32_t t_current = millis();
  if (t_current - t_last_kinematics_update >= k_dtKinematicsUpdate) {
    t_last_kinematics_update = t_current;

    uint8_t stance = (gait_phase + 1) % kNumOfGaitPhases;
    z_body_local = (motors[stance * 2].states_.q + motors[stance * 2 + 1].states_.q) / 2 - gait_phase * kBodyZOffset;
  }
}