#include "state_estimation.h"

#include <algorithm>
#include "../../include/joint_defs.h"
#include "../mcu_util/mcu_util.h"
#include "../motor_control/motor_control.h"
#include "../locomotion/locomotion.h"

// time variables
elapsedMicros dt_last_pos_update = 0;                     // time since last leg position sampling in microseconds
elapsedMicros dt_last_vel_update = 0;                     // time since last leg velocity sampling in microseconds
elapsedMicros dt_last_accel_update = 0;                   // time since last leg acceleration sampling in microseconds
uint32_t t_last_IMU_update = 0;                           // timestamp in milliseconds at last IMU sampling
std::vector<uint32_t> t_last_contact_update{0, 0, 0, 0};  // timestamp in milliseconds at last leg motor ground contact update
uint32_t t_last_power_update = 0;                         // timestamp in milliseconds at last power update
uint32_t t_last_motor_torque_filter_update = 0;           // timestamp in milliseconds at last motor torque filter update
uint32_t t_last_kinematics_update = 0;                    // timestamp in milliseconds at last kinematics update

// hardware interrupt encoders
Encoder encoders[] = {Encoder(ENC_2_A, ENC_2_B),  // medial body; front right leg
                      Encoder(ENC_1_A, ENC_1_B),  // medial body; front left leg
                      Encoder(ENC_4_A, ENC_4_B),  // medial body; rear right leg
                      Encoder(ENC_3_A, ENC_3_B),  // medial body; rear left leg
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

// moving average filter for joint velocity
std::vector<MovingAvgFilter> q_dot_filters = {MovingAvgFilter(kLegVelFilterLength),
                                               MovingAvgFilter(kLegVelFilterLength),
                                               MovingAvgFilter(kLegVelFilterLength),
                                               MovingAvgFilter(kLegVelFilterLength),
                                               MovingAvgFilter(kLegVelFilterLength),
                                               MovingAvgFilter(kLegVelFilterLength),
                                               MovingAvgFilter(kLegVelFilterLength),
                                               MovingAvgFilter(kLegVelFilterLength),
                                               MovingAvgFilter(kLegVelFilterLength),
                                               MovingAvgFilter(kLegVelFilterLength)};

// moving average filter for joint acceleration
std::vector<MovingAvgFilter> q_ddot_filters = {MovingAvgFilter(kLegAccelFilterLength),
                                               MovingAvgFilter(kLegAccelFilterLength),
                                               MovingAvgFilter(kLegAccelFilterLength),
                                               MovingAvgFilter(kLegAccelFilterLength),
                                               MovingAvgFilter(kLegAccelFilterLength),
                                               MovingAvgFilter(kLegAccelFilterLength),
                                               MovingAvgFilter(kLegAccelFilterLength),
                                               MovingAvgFilter(kLegAccelFilterLength),
                                               MovingAvgFilter(kLegAccelFilterLength),
                                               MovingAvgFilter(kLegAccelFilterLength)};

std::vector<float> q(kNumOfJoints, 0);                // joint position from encoders
std::vector<float> q_prev(kNumOfJoints, 0);           // q at last velocity update
std::vector<float> q_dot(kNumOfJoints, 0);            // unfiltered joint velocity from numerical difference
std::vector<float> q_dot_prev(kNumOfJoints, 0);       // q_dot at last acceleration update
std::vector<float> q_ddot(kNumOfJoints, 0);           // unfiltered joint acceleration from numerical difference
float z_body_local = 0;                               // height of body above local terrain
float dist_traveled = 0;                              // distance traveled estimated with translational joint displacement

std::vector<float> rpy_lateral_0 = {-4.2, 0.45, 0};   // lateral body roll pitch yaw after homing
std::vector<float> rpy_lateral = {0, 0, 0};           // lateral body roll pitch yaw relative to rpy_lateral_0
std::vector<float> omega_lateral = {0, 0, 0};         // lateral body angular velocity with respect to body frame axes

// moving average filter for body angular velocity
std::vector<MovingAvgFilter> omega_filters = {MovingAvgFilter(kGyroFilterLength),
                                              MovingAvgFilter(kGyroFilterLength),
                                              MovingAvgFilter(kGyroFilterLength)};

std::vector<float> q_leg_init(kNumOfLegs/2, kQLegMin);          // leg motor position at the start of leg touchdown/contact detection
std::vector<int> isInContact = {false, false, false, false};    // true if the motor's current exceeds the threshold during touchdown; stays true until legs lift
std::vector<int> isDecelerated(kNumOfLegs, false);              // true if a leg's deceleration has exceeded a threshold during touchdown; reset after each cycle
std::vector<float> q_dot_max(kNumOfLegs, 0);                    // maximum leg velocity reached during leg touchdown; used for contact detection; reset after each cycle
float terrain_pitch = 0;                                        // pitch of the terrain in degrees; take care of the sign and the direction of locomotion

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
  if (fabs(x_norm) < EPS) {
    dir_1 = 1;
  }
  if (fabs(y_norm) < EPS) {
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

MovingAvgFilter::MovingAvgFilter(uint8_t _size) {
  std::queue<float> tmp( ( std::deque<float>(_size) ) );
  size = _size;
  filter_queue = tmp;
}

void MovingAvgFilter::updateFilter(float val) {
    queue_sum -= filter_queue.front();
    filter_queue.pop();
    filter_queue.push(val);
    queue_sum += val;
    filtered_value = queue_sum / size;
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
  if (fabs(rpy_lateral_0[0]) < EPS && fabs(rpy_lateral_0[1]) < EPS && fabs(rpy_lateral_0[2]) < EPS) {
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
  updateJointEstimates();
  updateIMUEstimate();
  updateBodyLegContactState(gait_phase);
  updatePowerMeasurement();
  updateMotorTorqueFilters();
  updateKinematics();
  estimateTerrainSlope();
}

// update robot joint position, velocity, accelerations
void updateJointEstimates() {

  // update joint position
  if (dt_last_pos_update >= k_dtPosUpdate) {

    // read leg encoders and update position estimates
    for (uint8_t i = 0; i < kNumOfLegs; i++) {
      q[i] = kTxRatioLegDrive * (kDirectionWorm[i]*(float)(encoders[i].read()) / k_CPR_AMT102_V);
    }

    // update translation and yaw joint position estimates
    q[JointID::kJointTranslate] = motors[MotorID::kMotorTranslate].states_.q;
    q[JointID::kJointYaw] = motors[MotorID::kMotorYaw].states_.q;

    dt_last_pos_update = 0;
  }

  // update joint velocity
  // this entire function runs in <= 1 microsecond
  // so the same time period can be used to calculate all joints' velocity estimates
  if (dt_last_vel_update >= k_dtVelUpdate) {

    // update velocity estimates
    for (uint8_t i = 0; i < kNumOfLegs; i++) {
      q_dot[i] = (q[i] - q_prev[i]) / ((float)dt_last_vel_update / 1e6);
      q_prev[i] = q[i];
    }

    // update distance traveled by summing the translational joint displacement
    // distance traveled tracks the medial body since it is the outer frame in fore-aft
    if (gait_phase == GaitPhases::kMedialSwing) {
      float dq_trans = fabs(q[JointID::kJointTranslate] - q_prev[JointID::kJointTranslate]);
      if (dq_trans < 1e-1) dq_trans = 0;
      dist_traveled += dq_trans;
    }

    // update translation and yaw joint velocity estimates
    q_dot[JointID::kJointTranslate] = motors[MotorID::kMotorTranslate].states_.q_dot;
    q_dot[JointID::kJointYaw] = motors[MotorID::kMotorYaw].states_.q_dot;
    q_prev[JointID::kJointTranslate] = q[JointID::kJointTranslate];
    q_prev[JointID::kJointYaw] = q[JointID::kJointYaw];

    updateVelFilters();

    dt_last_vel_update = 0;
  }

  // update joint acceleration
  // this entire function runs in <= 1 microsecond
  // so the same time period can be used to calculate all joints' acceleration estimates
  if (dt_last_accel_update >= k_dtAccelUpdate) {

    // update acceleration estimates
    for (uint8_t i = 0; i < kNumOfLegs; i++) {
      q_ddot[i] = (q_dot_filters[i].filtered_value - q_dot_prev[i]) / ((float)dt_last_accel_update / 1e6);
      q_dot_prev[i] = q_dot_filters[i].filtered_value;
    }

    // update translation and yaw joint acceleration estimates
    q_ddot[JointID::kJointTranslate] = (motors[MotorID::kMotorTranslate].states_.q_dot - q_dot_prev[JointID::kJointTranslate]) / ((float)dt_last_accel_update / 1e6);
    q_ddot[JointID::kJointYaw] = (motors[MotorID::kMotorYaw].states_.q_dot - q_dot_prev[JointID::kJointYaw]) / ((float)dt_last_accel_update / 1e6);
    q_dot_prev[JointID::kJointTranslate] = q_dot[JointID::kJointTranslate];
    q_dot_prev[JointID::kJointYaw] = q_dot[JointID::kJointYaw];

    updateAccelFilters();

    dt_last_accel_update = 0;
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
    parseTeensySerial();
    rpy_lateral = input_rpy;
    omega_lateral = input_omega;
    #endif

    std::transform(rpy_lateral.begin(), rpy_lateral.end(), rpy_lateral_0.begin(), rpy_lateral.begin(), std::minus<float>());  // subtract IMU mounting offset
    updateOmegaFilters();                                                                                                     // filter angular velocity estimates
  }
  
}

// call the updateFilter() function for body angular velocity filters
// update frequency tracks sampling rate
void updateOmegaFilters() {
  for (uint8_t i = 0; i < omega_filters.size(); i++) {
    omega_filters[i].updateFilter(omega_lateral[i]);
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

    for(uint8_t idx_motor = 0; idx_motor < kNumOfActuators; ++idx_motor) {
      battery_voltage += motors[idx_motor].states_.bus_voltage;
      battery_current += motors[idx_motor].states_.bus_current;
      battery_power += motors[idx_motor].states_.bus_voltage*motors[idx_motor].states_.bus_current;
    }

    battery_voltage /= kNumOfActuators;
  }
}

// estimates the contact state of the given leg motor
void updateLegMotorContactState(uint8_t idx_motor) {
  uint32_t t_current = millis();
  if (t_current - t_last_contact_update[idx_motor] >= k_dtContactUpdate) {
    t_last_contact_update[idx_motor] = t_current;

    // for each leg in touchdown
    for (uint8_t idx_leg = idx_motor*2; idx_leg < idx_motor*2 + 2; ++idx_leg) {

      if (motors[idx_motor].states_.q - q_leg_init[idx_motor] > kDqLegMotorStartContact  // if the actuator has moved some amount
          && isNotStuck(idx_motor)) {                                                     // AND the legs are not stuck

        // contact detection using velocity reduction percentage
        #ifdef USE_SCALED_VELOCITY_THRESHOLD

        // update max leg velocity during touchdown
        if (q_dot_filters[idx_leg].filtered_value > q_dot_max[idx_leg]) {           // if the current leg velocity exceeds the historical max value
          q_dot_max[idx_leg] = q_dot_filters[idx_leg].filtered_value;
        }

        // check if the leg velocity has fallen below the threshold
        isDecelerated[idx_leg] = (q_dot_filters[idx_leg].filtered_value < kQdotLegPercentContact*q_dot_max[idx_leg]);  // don't latch; condition should persist with true contact

        #endif
        
        // contact detection checking for max deceleration
        #ifdef USE_DECELERATION_THRESHOLD

        isDecelerated[idx_leg] = isDecelerated[idx_leg] || (q_ddot_filters[idx_leg].filtered_value < kQddotLegContact);

        #endif

        // contact detection checking for stopped legs
        // this should always be on
        #ifdef USE_VELOCITY_THRESHOLD

        isDecelerated[idx_leg] = isDecelerated[idx_leg] || (q_dot[idx_leg] < kQdotLegContact);

        #endif
      }

      // if the leg cannot move kDqLegMotorStartContact after kDtTouchdown,
      // assume it is in contact
      if (t_current - t_start_contact > kDtTouchdown
          ) {
        isDecelerated[idx_leg] = isDecelerated[idx_leg] || q_dot[idx_leg] < kQdotLegContact;
      }
    }

    // combine leg contact states to determine motor contact state
    isInContact[idx_motor] = isDecelerated[idx_motor*2] && isDecelerated[idx_motor*2 + 1];
  }
}

// estimates the contact state of each swing leg motor
void updateBodyLegContactState(uint8_t idx_body) {
  if (actuation_phase == ActuationPhases::kTouchDown
      && (motion_primitive == ReactiveBehaviors::kNone || motion_primitive == ReactiveBehaviors::kStancePosition)
      ) {   // only check contact state of the swing leg motors during touchdown phase
    updateLegMotorContactState(idx_body*2);
    updateLegMotorContactState(idx_body*2 + 1);

  }
}

void resetLegMotorContactState(uint8_t idx_motor) {
  isInContact[idx_motor] = false;
  isDecelerated[idx_motor*2] = false;
  isDecelerated[idx_motor*2 + 1] = false;
  q_dot_max[idx_motor*2] = 0;
  q_dot_max[idx_motor*2 + 1] = 0;
}

void resetBodyLegContactState(uint8_t idx_body) {
  resetLegMotorContactState(idx_body*2);
  resetLegMotorContactState(idx_body*2 + 1);
}

// call the updateFilter() function for motor torque filters
void updateMotorTorqueFilters() {
  uint32_t t_current = millis();
  if (t_current - t_last_motor_torque_filter_update >= k_dtMotorTorqueFilterUpdate) {
    t_last_motor_torque_filter_update = t_current;

    for (uint8_t i = 0; i < kNumOfActuators; i++) {
      motor_torque_filters[i].updateFilter(motors[i].states_.torque);
    }
  }
}

// call the updateFilter() function for joint velocity filters
// update frequency tracks sampling rate
void updateVelFilters() {
  for (uint8_t i = 0; i < kNumOfJoints; i++) {
    q_dot_filters[i].updateFilter(q_dot[i]);
  }
}

// call the updateFilter() function for joint acceleration filters
// update frequency tracks sampling rate
void updateAccelFilters() {
  for (uint8_t i = 0; i < kNumOfJoints; i++) {
    q_ddot_filters[i].updateFilter(q_ddot[i]);
  }
}

// update robot body pose based on kinematics
void updateKinematics() {
  uint32_t t_current = millis();
  if (t_current - t_last_kinematics_update >= k_dtKinematicsUpdate) {
    t_last_kinematics_update = t_current;

    // update local body height
    if (isInContact[GaitPhases::kLateralSwing * 2] && isInContact[GaitPhases::kLateralSwing * 2 + 1]) {
      z_body_local = (q[4] + q[5] + q[6] + q[7])/4;
    } 
    // walking on stairs requires this to be disabled since height estimation can vary depending on whether medial or later legs are standing
    // TO DO: improve body height estimation across both stances 
    else if (isInContact[GaitPhases::kMedialSwing * 2] && isInContact[GaitPhases::kMedialSwing * 2 + 1]) {
      z_body_local = (q[0] + q[1] + q[2] + q[3])/4;
    }
    
  }
}

// returns true if the two legs for the corresponding motor are away from the minimum joint limit by a margin
// returning false does NOT necessarily mean the legs are stuck; they may still be moving away from the joint limit
bool isNotStuck(uint8_t idx_motor) {
  return (q[idx_motor*2] > kQLegUnstuck) && (q[idx_motor*2 + 1] > kQLegUnstuck);
}
bool isNearLimitLeg(uint8_t idx_motor) {
  return (q[idx_motor*2] < kQLegNearLimit) || (q[idx_motor*2 + 1] < kQLegNearLimit);
}

// estimates the terrain slope based on current ground contacts
// using 2D simplification until vector support is added
// update terrain slope only upon ground contact
void estimateTerrainSlope() {
  if (actuation_phase == ActuationPhases::kTouchDown
      && isInContact[gait_phase*2] && isInContact[gait_phase*2 + 1]
      ) {
    float q_front, q_rear = 0;

    // if medial legs touched down
    if (gait_phase == GaitPhases::kMedialSwing) {
      q_front = motors[MotorID::kMotorMedialFront].states_.q;
      q_rear  = motors[MotorID::kMotorMedialRear].states_.q;

    // if lateral legs touched down
    } else {
      q_front = (q[kJointLateralFrontRight] + q[kJointLateralFrontLeft])/2;
      q_rear = (q[kJointLateralRearRight] + q[kJointLateralRearLeft])/2;
    }

    float x_front = (stance_length[gait_phase]/2)*cos(DEG2RAD*rpy_lateral[1]) - q_front*sin(DEG2RAD*rpy_lateral[1]);    
    float x_rear = (-stance_length[gait_phase]/2)*cos(DEG2RAD*rpy_lateral[1]) - q_rear*sin(DEG2RAD*rpy_lateral[1]);
    float dx = x_front - x_rear;

    float z_front = (-stance_length[gait_phase]/2)*sin(DEG2RAD*rpy_lateral[1]) - q_front*cos(DEG2RAD*rpy_lateral[1]);    
    float z_rear = (stance_length[gait_phase]/2)*sin(DEG2RAD*rpy_lateral[1]) - q_rear*cos(DEG2RAD*rpy_lateral[1]);
    float dz = z_front - z_rear;
    
    terrain_pitch = RAD2DEG*atan2(dz, dx);
  }
}