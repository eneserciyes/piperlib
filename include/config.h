#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

class ControllerConfig {
public:
  std::string interface_name;
  std::string urdf_path;
  std::array<double, MOTOR_DOF> default_kp;
  std::array<double, MOTOR_DOF> default_kd;
  std::array<double, MOTOR_DOF> joint_vel_max;
  std::array<double, MOTOR_DOF> joint_acc_max;
  int over_current_cnt_max;
  double controller_freq_hz;
  bool gravity_compensation;
  bool gripper_on;

  ControllerConfig(
      std::string interface_name = "can0",
      std::string urdf_path = "../urdf/piper_description.urdf",
      std::array<double, MOTOR_DOF> default_kp = {10.0, 10.0, 10.0, 10.0, 10.0,
                                                  10.0},
      std::array<double, MOTOR_DOF> default_kd = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8},
      std::array<double, MOTOR_DOF> joint_vel_max = {3.14, 3.40, 3.14, 3.92,
                                                     3.92, 3.92},
      std::array<double, MOTOR_DOF> joint_acc_max = {3.0, 3.0, 3.0, 3.0, 3.0,
                                                     3.0},
      int over_current_cnt_max = 20, double controller_freq_hz = 500.0,
      bool gravity_compensation = true, bool gripper_on = false)
      : interface_name(interface_name), urdf_path(urdf_path),
        default_kp(default_kp), default_kd(default_kd),
        joint_vel_max(joint_vel_max), joint_acc_max(joint_acc_max),
        over_current_cnt_max(over_current_cnt_max),
        controller_freq_hz(controller_freq_hz),
        gravity_compensation(gravity_compensation), gripper_on(gripper_on) {}
};

#endif // CONFIG_H
