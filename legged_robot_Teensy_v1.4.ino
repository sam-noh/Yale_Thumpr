////////////////////////////////////////////////////////////////////////////////////////////////
// includes
#include "include\joint_defs.h"

#include "src\mcu_util\mcu_util.h"
#include "src\state_estimation\state_estimation.h"
#include "src\motor_control\motor_control.h"
#include "src\locomotion\locomotion.h"

////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  initTeensy();
  initSDCard();
  bno08x_imu.initIMU(&WIRE_IMU);
  initActuators();

  homeLeggedRobot();     // actuator position after homing is assumed to be the new zero position
  standUp();
}

void loop() {
  while (!stop_signal) {
    updateStates();
    handleODriveCANMsg();
    updateCmdVector();
    updateGait();
    updateMotorCommands();
    transmitData();
  }

//  stopActuators();    // only use if the legs are actually non-backdrviable; if not, the robot will drop when the motors turn off
  data_file.close();
  delay(10000000);
}

void transmitData() {
  uint32_t t_current = millis();
  if (t_current - t_last_print >= k_dtPrint) {
    t_last_print = t_current;
    snprintf(sent_data, sizeof(sent_data), "%.3f\t", (float)t_current / 1000);
    transmitMsg();
    writeToCard(sent_data);

    Serial.print("a: ");
    for(uint8_t i = 0; i < 6; i++) {
      if(fabs(motors[i].states_.q) < 1e-2 && motors[i].states_.q < 0) {
        motors[i].states_.q = 0;
      }
    }
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motors[0].states_.q, motors[1].states_.q, motors[2].states_.q, motors[3].states_.q, motors[4].states_.q, motors[5].states_.q); // motor positions
    transmitMsg();
    writeToCard(sent_data);

    Serial.print("a_d: ");
    for(uint8_t i = 0; i < 6; i++) {
      if(fabs(motors[i].states_.q_d) < 1e-2 && motors[i].states_.q_d < 0) {
        motors[i].states_.q_d = 0;
      }
    }
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motors[0].states_.q_d, motors[1].states_.q_d, motors[2].states_.q_d, motors[3].states_.q_d, motors[4].states_.q_d, motors[5].states_.q_d);   // motor setpoints
    transmitMsg();
    writeToCard(sent_data);

    //    Serial.print("q: ");
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9]);   // joint positions
    //    transmitMsg();
    writeToCard(sent_data);

    Serial.printf("local body height: %.2f\t", z_body_local);

    Serial.print("tau: ");
    for(uint8_t i = 0; i < 6; i++) {
      if(fabs(motor_torque_filters[i].filtered_value) < 1e-2 && motor_torque_filters[i].filtered_value < 0) {
        motor_torque_filters[i].filtered_value = 0;
      }
    }
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motor_torque_filters[0].filtered_value, motor_torque_filters[1].filtered_value, motor_torque_filters[2].filtered_value,
             motor_torque_filters[3].filtered_value, motor_torque_filters[4].filtered_value, motor_torque_filters[5].filtered_value);   // motor torque
    transmitMsg();
    writeToCard(sent_data);

    Serial.print("inContact: ");
    snprintf(sent_data, sizeof(sent_data), "%d\t%d\t%d\t%d\t",
             inContact[0], inContact[1], inContact[2], inContact[3]);
    transmitMsg();
    writeToCard(sent_data);

    Serial.print("states: ");
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%d\t%d\t%lu\n",
             acs711ex_sensor.current_measured, rpy_medial[0], rpy_medial[1], rpy_medial[2], gait_phase, actuation_phase, gait_cycles);    // roll pitch yaw, gait variables
    transmitMsg();
    writeToCard(sent_data);
  }
}

