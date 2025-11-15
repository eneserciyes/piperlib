#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H
#include "common.h"
#include "config.h"
#include "piper_interface.h"
#include "solver.h"

#include <ruckig/ruckig.hpp>
#include <spdlog/spdlog.h>
#include <thread>

class PiperController {
public:
  PiperController(ControllerConfig controller_config);
  ~PiperController();

  void resetToHome();
  bool start();
  void stop();
  bool isRunning() const;
  JointState getCurrentState();
  std::array<double, MOTOR_DOF> getCurrentTarget();
  void setTarget(
      const std::array<double, MOTOR_DOF> &new_target_pos,
      const float new_target_gripper_pos,
      const std::array<double, MOTOR_DOF> &new_target_vel = {0.0, 0.0, 0.0, 0.0,
                                                             0.0, 0.0},
      const std::array<double, MOTOR_DOF> &new_target_acc = {0.0, 0.0, 0.0, 0.0,
                                                             0.0, 0.0});

private:
  const ControllerConfig controller_config_;
  ruckig::Ruckig<MOTOR_DOF> otg_;
  InverseDynamicsSolver solver_;

  Gain gain_;
  std::array<double, MOTOR_DOF> target_position_;
  std::array<double, MOTOR_DOF> target_velocity_;
  std::array<double, MOTOR_DOF> target_acceleration_;
  float target_gripper_pos_;

  PiperInterface piper_interface_;
  std::thread control_loop_thread_;
  std::mutex target_mutex_;
  std::atomic<bool> control_loop_running_{false};
  std::atomic<bool> should_stop_{false};
  std::atomic<bool> new_target_flag_{false};

  std::chrono::microseconds start_time_us_;
  int over_current_cnt_ = 0;

  void controlLoop();
  void checkJointStateSanity(const JointState &joint_state);
  void driverProtection();
};

#endif
