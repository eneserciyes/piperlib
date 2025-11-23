#include "piper_interface.h"
#include "solver.h"
#include "utils.h"
#include <signal.h>
#include <spdlog/spdlog.h>
#include <iostream>
#include <string>

static bool running = true;

void signal_handler(int signal) { running = false; }

void print_usage(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [options]\n"
              << "Options:\n"
              << "  -i, --interface <name>  CAN interface name (default: can_left)\n"
              << "  -u, --urdf <path>       Path to URDF file (default: ../urdf/piper_no_gripper_description.urdf)\n"
              << "  -g, --gripper           Enable gripper (default: false)\n"
              << "  -h, --help              Show this help message\n";
}

int main(int argc, char *argv[]) {
  signal(SIGINT, signal_handler);

  // Default configuration
  std::string interface_name = "can_left";
  std::string urdf_path = "../urdf/piper_no_gripper_description.urdf";
  bool gripper_on = false;

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
        print_usage(argv[0]);
        return 0;
    } else if (arg == "-i" || arg == "--interface") {
        if (i + 1 < argc) {
            interface_name = argv[++i];
        } else {
            std::cerr << "Error: --interface requires an argument\n";
            return 1;
        }
    } else if (arg == "-u" || arg == "--urdf") {
        if (i + 1 < argc) {
            urdf_path = argv[++i];
        } else {
            std::cerr << "Error: --urdf requires an argument\n";
            return 1;
        }
    } else if (arg == "-g" || arg == "--gripper") {
        gripper_on = true;
    } else {
        std::cerr << "Unknown argument: " << arg << "\n";
        print_usage(argv[0]);
        return 1;
    }
  }

  InverseDynamicsSolver solver(urdf_path);
  PiperInterface piper_interface(interface_name, gripper_on);
  
  if (piper_interface.get_arm_status() == ArmStatus::EMERGENCY_STOP) {
    spdlog::error("Arm status is emergency stop, first resume emergency stop!");
    exit(1);
  }

  // reset arm
  reset_arm(piper_interface);
  if (gripper_on) {
    reset_gripper(piper_interface);
  }

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
  if (gripper_on) {
    piper_interface.disable_gripper();
  }
  return 0;
}
