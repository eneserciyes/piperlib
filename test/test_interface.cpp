#include "catch2/catch_test_macros.hpp"
#include "piper_interface.h"

#include <cmath>
#include <spdlog/spdlog.h>

TEST_CASE("PiperInterface tests", "[piper]") {
  PiperInterface piper_interface("can0");

  if (piper_interface.get_arm_status() == ArmStatus::EMERGENCY_STOP) {
    spdlog::error("Arm status is emergency stop, first resume emergency stop!");
    exit(1);
  }

  SECTION("Get current state") {
    JointState state = piper_interface.get_current_state();
    spdlog::info("✓ Successfully retrieved current state");

    for (int i = 0; i < MOTOR_DOF; i++) {
      REQUIRE(state.pos[i] >= POS_MIN);
      REQUIRE(state.pos[i] <= POS_MAX);

      REQUIRE(state.vel[i] >= VEL_MIN);
      REQUIRE(state.vel[i] <= VEL_MAX);

      REQUIRE(state.torque[i] >= T_MIN);
      REQUIRE(state.torque[i] <= T_MAX);


      REQUIRE(std::isfinite(state.pos[i]));
      REQUIRE(std::isfinite(state.vel[i]));
      REQUIRE(std::isfinite(state.torque[i]));

      spdlog::info("✓ Motor {} pos={:.3f}, vel={:.3f}, tau={:.3f}", i,
                   state.pos[i], state.vel[i], state.torque[i]);
    }

    REQUIRE(std::isfinite(state.gripper_pos));
    spdlog::info("✓ Gripper pos={:.3f}", state.gripper_pos);
    spdlog::info("✓ Gripper effort={:.3f}", state.gripper_effort);
    spdlog::info("✓ All motor states are within reasonable bounds");
  }

  SECTION("Arm status") {
    ArmStatus arm_status = piper_interface.get_arm_status();
    spdlog::info("Arm status is {}", static_cast<uint8_t>(arm_status));
    REQUIRE(arm_status == ArmStatus::NORMAL);
  }

  SECTION("Arm enable/disable") {
    piper_interface.enable_arm();
    REQUIRE(piper_interface.is_arm_enabled());

    piper_interface.disable_arm();
    REQUIRE(!piper_interface.is_arm_enabled());

    spdlog::info("✓ Motor enable and disable test PASSED");
  }

  SECTION("Control mode switching") {
    // test position-velocity mode
    piper_interface.enable_arm();
    piper_interface.set_arm_mode(ControlMode::CAN_COMMAND, MoveMode::JOINT, 0x30, ArmController::POSITION_VELOCITY);

    REQUIRE(piper_interface.get_control_mode() == ControlMode::CAN_COMMAND);
    REQUIRE(piper_interface.get_move_mode() == MoveMode::JOINT);
    spdlog::info("✓ Set to joint mode successfully!");

    // test MIT mode
    piper_interface.set_arm_mode(ControlMode::CAN_COMMAND, MoveMode::MIT, 0x00, ArmController::MIT);
    REQUIRE(piper_interface.get_control_mode() == ControlMode::CAN_COMMAND);
    REQUIRE(piper_interface.get_move_mode() == MoveMode::MIT);
    spdlog::info("✓ Set to MIT mode successfully!");

    piper_interface.disable_arm();
  }


  SECTION("Gripper position"){
    piper_interface.enable_gripper();
    if(!piper_interface.is_gripper_enabled()) {
      spdlog::error("Gripper cannot be enabled, trying to disable and enable again");
      piper_interface.disable_gripper();
      piper_interface.enable_gripper();
    }
    REQUIRE(piper_interface.is_gripper_enabled());
    double pos = piper_interface.get_current_state().gripper_pos;
    spdlog::info("Gripper pos: {:.3f}", pos);
    REQUIRE(pos >= -0.05);
    REQUIRE(pos <= 1.15);

    piper_interface.disable_gripper();
    REQUIRE(!piper_interface.is_gripper_enabled());

  }


  // SECTION("Motor control") {
  //   piper_interface.enable_motors();
  //   spdlog::info("Gripper pos: {}", piper_interface.get_current_state().gripper_pos);
  //   piper_interface.set_ctrl_mode(0x01, 0x04, 0x00, 0xAD);

  //   JointState initial_joint_state = piper_interface.get_current_state();
  //   spdlog::info("Initial joint state: {}", initial_joint_state.gripper_pos);

  //   JointState desired_joint_state = JointState(
  //       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  //       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 1.0);

  //   Gain gain = Gain({10.0, 10.0, 10.0, 10.0, 10.0, 10.0},
  //                    {0.8, 0.8, 0.8, 0.8, 0.8, 0.8});

  //   float control_freq = 200.0;
  //   float time_to_reach_pos = 1.0; // seconds
  //   int total_steps = int(time_to_reach_pos * control_freq);

  //   JointState ref_joint_state;
  //   for (int i = 0; i < MOTOR_DOF; i++) {
  //     ref_joint_state.vel[i] =
  //         (desired_joint_state.pos[i] - initial_joint_state.pos[i]) /
  //         time_to_reach_pos;
  //   }
  //   ref_joint_state.gripper_pos = desired_joint_state.gripper_pos;
  //   ref_joint_state.torque = {0.0,   0.0, 0.0, 0.0,
  //                             -0.85, 0.0}; // Gravity compensation for joint 5

  //   spdlog::info("Arm is going to move. Press Enter to continue...");
  //   std::cin.get();
  //   RateLimiter rate_limiter(control_freq);
  //   for (int i = 0; i < total_steps; i++) {
  //     for (int j = 0; j < MOTOR_DOF; j++) {
  //       ref_joint_state.pos[j] =
  //           initial_joint_state.pos[j] +
  //           (desired_joint_state.pos[j] - initial_joint_state.pos[j]) *
  //               (float(i) / total_steps);

  //       ref_joint_state.gripper_pos =
  //           initial_joint_state.gripper_pos +
  //           (desired_joint_state.gripper_pos - initial_joint_state.gripper_pos) *
  //               (float(i) / total_steps);
  //     }
  //     piper_interface.set_motor_ctrl(ref_joint_state, gain);
  //     piper_interface.set_gripper_ctrl(ref_joint_state.gripper_pos, 0.2f);
  //     rate_limiter.wait();
  //   }

  //   sleep_ms(1000); // wait for the arm to settle

  //   JointState final_joint_state = piper_interface.get_current_state();
  //   for (int i = 0; i < MOTOR_DOF; i++) {
  //     float diff =
  //         std::abs(final_joint_state.pos[i] - desired_joint_state.pos[i]);
  //     CHECK(diff < 0.05f);
  //     spdlog::info("✓ Motor {} pos={}, pos_des={}, diff={:.3f}", i,
  //                  final_joint_state.pos[i], desired_joint_state.pos[i], diff);
  //   }

  //   CHECK(std::abs(final_joint_state.gripper_pos - desired_joint_state.gripper_pos) < 0.01);
  //   spdlog::info("✓ Gripper pos={}, pos_des={}, diff={:.3f}", final_joint_state.gripper_pos, desired_joint_state.gripper_pos, std::abs(final_joint_state.gripper_pos - desired_joint_state.gripper_pos));

  //   piper_interface.disable_gripper();
  //   piper_interface.emergency_stop();

  //   spdlog::info("✓ Motor control test PASSED");
  // }
}
