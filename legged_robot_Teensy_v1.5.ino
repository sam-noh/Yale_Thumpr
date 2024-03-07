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
  initActuators();
  
  homeLeggedRobot();
  zeroIMUReading();
  standUp();
}

void loop() {
  while (!stop_signal) {
    handleODriveCANMsg();
    parseJetsonSerial();
    parseTeensySerial();
    updateTrajectory();
    updateStates();
    updateGait();
    updateMotorCommands();
    sendTelemetry();
  }

  stopActuators();    // only use if the legs are actually non-backdrviable; if not, the robot will drop when the motors turn off
  closeDataFile();
  delay(10000000);
}

void sendTelemetry() {
  uint32_t t_current = millis();
  if (t_current - t_last_print >= k_dtPrint) {
    t_last_print = t_current;

    // current time
    snprintf(sent_data, sizeof(sent_data), "%.3f\t", (float)t_current / 1000);
    writeToSerial();
    writeToCard(sent_data);

    // actuator position
    for(uint8_t i = 0; i < 6; i++) {
      if(fabs(motors[i].states_.q) < ESP && motors[i].states_.q < 0) {
        motors[i].states_.q = 0;
      }
    }
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motors[0].states_.q, motors[1].states_.q, motors[2].states_.q, motors[3].states_.q, motors[4].states_.q, motors[5].states_.q);

    #ifdef DEBUG_ACTUATOR_POSITION
    SERIAL_USB.print("actuator q (mm): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // actuator position setpoint
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motors[0].states_.q_d, motors[1].states_.q_d, motors[2].states_.q_d, motors[3].states_.q_d, motors[4].states_.q_d, motors[5].states_.q_d);

    #ifdef DEBUG_ACTUATOR_SETPOINT
    SERIAL_USB.print("actuator q_d (mm): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // motor torque
    for(uint8_t i = 0; i < 6; i++) {
      if(fabs(motor_torque_filters[i].filtered_value) < ESP && motor_torque_filters[i].filtered_value < 0) {
        motor_torque_filters[i].filtered_value = 0;
      }
    }
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             motor_torque_filters[0].filtered_value, motor_torque_filters[1].filtered_value, motor_torque_filters[2].filtered_value,
             motor_torque_filters[3].filtered_value, motor_torque_filters[4].filtered_value, motor_torque_filters[5].filtered_value);

    #ifdef DEBUG_ACTUATOR_TORQUE
    SERIAL_USB.print("motor torque (Nm): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // motor temp
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t",
             motors[0].states_.motor_temp, motors[1].states_.motor_temp, motors[2].states_.motor_temp, motors[3].states_.motor_temp);

    #ifdef DEBUG_ACTUATOR_TEMP
    SERIAL_USB.print("actuator temp (C): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // leg positions
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7]);

    #ifdef DEBUG_LEG_POSITION
    SERIAL_USB.print("leg q (mm): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // leg velocities
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t",
             q_dot[0], q_dot[1], q_dot[2], q_dot[3], q_dot[4], q_dot[5], q_dot[6], q_dot[7]);

    #ifdef DEBUG_LEG_VELOCITY
    SERIAL_USB.print("leg q_dot (mm/s): ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // ground contacts
    snprintf(sent_data, sizeof(sent_data), "%d\t%d\t%d\t%d\t",
             inContact[0], inContact[1], inContact[2], inContact[3]);

    #ifdef DEBUG_CONTACT
    SERIAL_USB.print("contacts: ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // battery voltage, current, power for motors
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t", battery_voltage, battery_current, battery_power);

    #ifdef DEBUG_POWER
    SERIAL_USB.print("battery V, A, P: ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // IMU pose estimation
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t", rpy_lateral[0], rpy_lateral[1], rpy_lateral[2]);

    #ifdef DEBUG_RPY
    SERIAL_USB.print("RPY: ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // IMU angular velocity
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t", omega_lateral[0], omega_lateral[1], omega_lateral[2]);

    #ifdef DEBUG_OMEGA
    SERIAL_USB.print("omega: ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    // robot trajectory parameters
    snprintf(sent_data, sizeof(sent_data), "%.2f\t%.2f\t%.2f\t%.2f\t", cmd_vector[0], cmd_vector[2], q_leg_stance, leg_swing_percent);

    #ifdef DEBUG_TRAJECTORY
    SERIAL_USB.print("trajectory: ");
    writeToSerial();

    #endif
    writeToCard(sent_data);

    // gait variables
    snprintf(sent_data, sizeof(sent_data), "%d\t%d\t%lu\t", gait_phase, actuation_phase, gait_cycles);

    #ifdef DEBUG_GAIT
    SERIAL_USB.print("gait, actuation, cycles: ");
    writeToSerial();
    #endif
    writeToCard(sent_data);

    snprintf(sent_data, sizeof(sent_data), "\n");
    writeToSerial();
    writeToCard(sent_data);
  }
}

