#include "piper_interface.h"
#include "solver.h"
#include "utils.h"
#include <signal.h>
#include <spdlog/spdlog.h>

static bool running = true;

void signal_handler(int signal) { running = false; }

int main(int argc, char *argv[]) {
  signal(SIGINT, signal_handler);

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <urdf_file_path>" << std::endl;
    return 1;
  }



  std::string urdf_path = argv[1];
  InverseDynamicsSolver solver(urdf_path);
  PiperInterface piper_interface("can0");
  if (piper_interface.get_arm_status() == ArmStatus::EMERGENCY_STOP) {
    spdlog::error("Arm status is emergency stop, first resume emergency stop!");
    exit(1);
  }

  // reset arm
  reset_arm(piper_interface);
  reset_gripper(piper_interface);

  RateLimiter rate_limiter(200);

  JointState output_joint_state_;
  Gain grav_comp_gain =
      Gain({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  while (running) {
    JointState joint_state = piper_interface.get_current_state();
    std::array<double, MOTOR_DOF> joint_torque = solver.inverse_dynamics(
        joint_state.pos, joint_state.vel, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    output_joint_state_.pos = joint_state.pos;
    output_joint_state_.vel = joint_state.vel;
    output_joint_state_.torque = joint_torque;

    piper_interface.set_joint_pos_vel_torque(output_joint_state_, grav_comp_gain);
    rate_limiter.wait();
  }
  piper_interface.set_to_damping_mode();

  spdlog::info("Press Enter to disable arm...");
  std::cin.get();
  piper_interface.disable_arm();
  piper_interface.disable_gripper();
  return 0;
}