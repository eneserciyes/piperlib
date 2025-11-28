#include "controller_base.h"
#include "utils.h"

PiperController::PiperController(ControllerConfig controller_config)
    : controller_config_(controller_config),
      piper_interface_(controller_config.interface_name, controller_config.gripper_on),
      solver_(controller_config.urdf_path),
      otg_(1.0 / controller_config.controller_freq_hz),
      gain_(controller_config.default_kp, controller_config.default_kd) {
  start_time_us_ = get_time_us();
  target_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  target_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  target_acceleration_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  target_gripper_pos_ = 1.0f;
  minimum_duration_ = 0.0f;
}

PiperController::~PiperController() { stop(); }

void PiperController::resetToHome() {
  setTarget(
      controller_config_.home_position,
      1.0f,
      3.0f // minimum duration
  );

  // wait until homed
  while (true) {
    auto current_state = getCurrentState();
    bool is_close = true;
    for (size_t i = 0; i < MOTOR_DOF; ++i) {
      if (std::abs(current_state.pos[i] - controller_config_.home_position[i]) > 0.05) {
        is_close = false;
        break;
      }
    }

    if (is_close) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  spdlog::info("Arm reset to home successfully");
}

bool PiperController::start() {
  if (control_loop_running_.load()) {
    spdlog::warn("Controller already running");
    return false;
  }

  reset_arm(piper_interface_);
  reset_gripper(piper_interface_);

  should_stop_.store(false);
  control_loop_running_.store(true);
  control_loop_thread_ = std::thread(&PiperController::controlLoop, this);

  return true;
}

void PiperController::stop() {
  if (!control_loop_running_.load()) {
    return;
  }

  should_stop_.store(true);

  if (control_loop_thread_.joinable()) {
    control_loop_thread_.join();
  }

  control_loop_running_.store(false);
  piper_interface_.set_to_damping_mode();
  spdlog::info("PiperController set to damping mode");

  std::cout << "Press Enter to disable arm..." << std::endl;
  std::cin.get();

  disable_arm(piper_interface_);
  disable_gripper(piper_interface_);
}

void PiperController::setTarget(
    const std::array<double, MOTOR_DOF> &new_target_pos,
    const float new_target_gripper_pos,
    const float minimum_duration,
    const std::array<double, MOTOR_DOF> &new_target_vel,
    const std::array<double, MOTOR_DOF> &new_target_acc) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_position_ = new_target_pos;
  target_velocity_ = new_target_vel;
  target_acceleration_ = new_target_acc;
  target_gripper_pos_ = new_target_gripper_pos;
  minimum_duration_ = minimum_duration;
  new_target_flag_.store(true);
  spdlog::debug("Target set to: [{}]", ::join(new_target_pos));
}

bool PiperController::isRunning() const { return control_loop_running_.load(); }

std::array<double, MOTOR_DOF> PiperController::getCurrentTarget() {
  std::lock_guard<std::mutex> lock(target_mutex_);
  return target_position_;
}

JointState PiperController::getCurrentState() {
  return piper_interface_.get_current_state();
}

void PiperController::driverProtection() {
  bool over_current = false;
  for (int i = 0; i < MOTOR_DOF; ++i) {
    DriverStatus driver_status = piper_interface_.get_driver_status(i);
    if (driver_status.driver_overcurrent) {
      over_current = true;
      spdlog::warn("Over current detected once on joint {}", i);
      break;
    }
    if (driver_status.driver_error_status) {
      piper_interface_.set_emergency_stop(EmergencyStop::STOP);
      throw std::runtime_error("Driver error detected on joint " +
                               std::to_string(i) +
                               ". Please restart the program.");
    }
    if (driver_status.driver_overheating) {
      piper_interface_.set_emergency_stop(EmergencyStop::STOP);
      throw std::runtime_error("Driver overheating detected on joint " +
                               std::to_string(i) +
                               ". Please restart the program.");
    }
    if (driver_status.collision_status) {
      piper_interface_.set_emergency_stop(EmergencyStop::STOP);
      throw std::runtime_error("Collision detected on joint " +
                               std::to_string(i) +
                               ". Please restart the program.");
    }
    if (driver_status.stall_status) {
      piper_interface_.set_emergency_stop(EmergencyStop::STOP);
      throw std::runtime_error("Stall detected on joint " + std::to_string(i) +
                               ". Please restart the program.");
    }
  }

  if (over_current) {
    over_current_cnt_++;
    if (over_current_cnt_ > controller_config_.over_current_cnt_max) {
      piper_interface_.set_emergency_stop(EmergencyStop::STOP);
      throw std::runtime_error("Over current detected too many times. Please "
                               "restart the program.");
    }
  } else {
    over_current_cnt_ = 0;
  }
}

void PiperController::controlLoop() {
  spdlog::info("Starting Piper controller control loop at {}Hz",
               controller_config_.controller_freq_hz);

  ruckig::InputParameter<MOTOR_DOF> input;
  ruckig::OutputParameter<MOTOR_DOF> output;

  JointState initial_joint_state = piper_interface_.get_current_state();
  input.current_position = initial_joint_state.pos;
  input.current_velocity = initial_joint_state.vel;
  input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    input.target_position = target_position_;
    input.target_velocity = target_velocity_;
    input.target_acceleration = target_acceleration_;
  }

  input.max_velocity = controller_config_.joint_vel_max;
  input.max_acceleration = controller_config_.joint_acc_max;

  JointState output_joint_cmd;

  RateLimiter rate_limiter(controller_config_.controller_freq_hz);
  while (!should_stop_.load()) {
    if (new_target_flag_.exchange(false)) {
      std::lock_guard<std::mutex> lock(target_mutex_);
      input.target_position = target_position_;
      input.target_velocity = target_velocity_;
      input.target_acceleration = target_acceleration_;
      input.minimum_duration = minimum_duration_;
      trajectory_active_.store(true);
      spdlog::debug("New target received: [{}]", ::join(input.target_position));
    }
    if (trajectory_active_.load()) {
      ruckig::Result result = otg_.update(input, output);

      if (result == ruckig::Result::Working) {
        output_joint_cmd.pos = output.new_position;
        output_joint_cmd.vel = output.new_velocity;

        if (controller_config_.gravity_compensation) {
          std::array<double, MOTOR_DOF> gravity_compensation =
              solver_.inverse_dynamics(output_joint_cmd.pos,
                                       output_joint_cmd.vel,
                                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
          output_joint_cmd.torque = gravity_compensation;
        } else {
          output_joint_cmd.torque = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }

        piper_interface_.set_joint_pos_vel_torque(output_joint_cmd, gain_);
        output.pass_to_input(input);
      } else if (result == ruckig::Result::Finished) {
        spdlog::debug("Trajectory completed");
        trajectory_active_.store(false);
      }
    } else {
      if (controller_config_.gravity_compensation) {
        auto current_joint_state = piper_interface_.get_current_state();
        std::array<double, MOTOR_DOF> gravity_compensation =
            solver_.inverse_dynamics(current_joint_state.pos,
                                     current_joint_state.vel,
                                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        output_joint_cmd.pos = current_joint_state.pos;
        output_joint_cmd.vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        output_joint_cmd.torque = gravity_compensation;
        piper_interface_.set_joint_pos_vel_torque(output_joint_cmd, gain_);
      }
    }

    if (controller_config_.gripper_on) {
      piper_interface_.set_gripper(target_gripper_pos_, 0.1f, GripperCode::ENABLE);
    }

    // TODO: check if joint state is within limits
    driverProtection();
    rate_limiter.wait();
  }
  spdlog::info("Control loop stopped");
}
