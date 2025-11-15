#include "piper_interface.h"
#include "common.h"
#include "spdlog/spdlog.h"
#include <cstdint>
#include <stdexcept>

#define COMMUNICATION_DELAY 10 // 10us

/********************************
        Helper functions
********************************/

inline uint8_t can_data_to_uint8_t(const uint8_t *data) {
  uint8_t value;
  std::memcpy(&value, data, sizeof(uint8_t));
  return value;
}
inline int16_t can_data_to_int16_t(const uint8_t *data) {
  int16_t value;
  std::memcpy(&value, data, sizeof(int16_t));
  return be16toh(value);
}

inline uint16_t can_data_to_uint16_t(const uint8_t *data) {
  uint16_t value;
  std::memcpy(&value, data, sizeof(uint16_t));
  return be16toh(value);
}

inline uint64_t can_data_to_uint64_t(const uint8_t *data) {
  uint64_t value;
  std::memcpy(&value, data, sizeof(uint64_t));
  return be64toh(value);
}

inline int32_t can_data_to_int32_t(const uint8_t *data) {
  int32_t value;
  std::memcpy(&value, data, sizeof(int32_t));
  return be32toh(value);
}

inline int float_to_int(const float val, const float min, const float max,
                        const int bits) {
  float span = max - min;
  return static_cast<int>((val - min) * ((1 << bits) - 1) / span);
}


/**
* Helper functions
*/

void disable_arm(PiperInterface &piper_interface, float timeout_sec) {
  // disable arm first
  auto start_time = get_time_ms();
  while (true) {
    piper_interface.set_emergency_stop(EmergencyStop::RESUME);
    sleep_ms(100);
    if (
      piper_interface.get_control_mode() == ControlMode::STANDBY 
      && piper_interface.get_arm_status() == ArmStatus::NORMAL
    )
    {
      break;
    }
    if ((get_time_ms() - start_time).count() > timeout_sec * 1000) {
      spdlog::error("Failed to disable arm");
      throw std::runtime_error("Failed to disable arm");
    }
    sleep_ms(200);
  }
}

void enable_arm(PiperInterface &piper_interface, float timeout_sec) {
  // enable arm
  auto start_time = get_time_ms();
  while (true) {
    piper_interface.enable_arm();
    sleep_ms(100);
    if (piper_interface.is_arm_enabled()) {
      break;
    }
    if ((get_time_ms() - start_time).count() > timeout_sec * 1000) {
      spdlog::error("Failed to enable arm");
      throw std::runtime_error("Failed to enable arm");
    }
    sleep_ms(200);
  }

  // set arm mode
  piper_interface.set_arm_mode(ControlMode::CAN_COMMAND, MoveMode::MIT, 100, ArmController::MIT);
  sleep_ms(100);

  if (piper_interface.get_arm_status() != ArmStatus::NORMAL || piper_interface.get_control_mode() != ControlMode::CAN_COMMAND || piper_interface.get_move_mode() != MoveMode::MIT) {
    spdlog::error("Failed to set arm mode");
    throw std::runtime_error("Failed to set arm mode");
  }
}

void reset_arm(PiperInterface &piper_interface, float timeout_sec) {
  disable_arm(piper_interface, timeout_sec);
  enable_arm(piper_interface, timeout_sec);
  spdlog::info("Arm reset successfully");
}

void disable_gripper(PiperInterface &piper_interface, float timeout_sec) {
  auto start_time = get_time_ms();
  // disable gripper
  while (true) {
    piper_interface.disable_gripper();
    sleep_ms(100);
    if (!piper_interface.is_gripper_enabled()) {
      break;
    }
    if ((get_time_ms() - start_time).count() > timeout_sec * 1000) {
      spdlog::error("Failed to disable gripper");
      throw std::runtime_error("Failed to disable gripper");
    }
    sleep_ms(200);
  }
  spdlog::info("Gripper disabled successfully");
}

void enable_gripper(PiperInterface &piper_interface, float timeout_sec) {
  auto start_time = get_time_ms();
  // enable gripper
  while (true) {
    piper_interface.enable_gripper();
    sleep_ms(100);
    if (piper_interface.is_gripper_enabled()) {
      break;
    }
    if ((get_time_ms() - start_time).count() > timeout_sec * 1000) {
      spdlog::error("Failed to enable gripper");
      throw std::runtime_error("Failed to enable gripper");
    }
    sleep_ms(200);
  }
  spdlog::info("Gripper enabled successfully");
}

void reset_gripper(PiperInterface &piper_interface, float timeout_sec) {
  disable_gripper(piper_interface, timeout_sec);
  enable_gripper(piper_interface, timeout_sec);
  spdlog::info("Gripper reset successfully");
}

/********************************
        PiperInterface
********************************/

PiperInterface::PiperInterface(std::string interface_name) {
  socketcan_ = new SocketCAN();
  socketcan_->open(interface_name.c_str());
  socketcan_->start_receiver_thread();
  socketcan_->reception_handler = [this](can_frame_t *frame) {
    can_receive_frame(frame);
  };
  sleep_ms(100);
  if (!can_connection_status_) {
    spdlog::error("Failed to connect to Piper arm. Check the connection and "
                  "power supply.");
    throw std::runtime_error("Failed to connect to Piper arm");
  }
}

void PiperInterface::transmit(can_frame_t &frame) {
  if (socketcan_->is_open()) {
    frame.can_dlc = 8;
    socketcan_->transmit(&frame);
  } else {
    spdlog::error("Cannot transmit CAN frame: CAN bus not open");
  }
}

void PiperInterface::can_receive_frame(const can_frame_t *frame) {
  can_connection_status_ = true;
  if (frame->can_id >= 0x251 && frame->can_id <= 0x256) { // high speed feedback
    int motor_id = frame->can_id - 0x251;
    qvel_[motor_id].store(can_data_to_int16_t(frame->data) / 1000.0f);
    tau_[motor_id].store(can_data_to_int16_t(frame->data + 2) / 1000.0f); // TODO: use KT to estimate torque from current
    qpos_[motor_id].store(can_data_to_int32_t(frame->data + 4) / 1000.0f);
  } else if (frame->can_id >= 0x261 &&
             frame->can_id <= 0x266) { // low speed feedback
    int motor_id = frame->can_id - 0x261;
    driver_status_[motor_id].store(DriverStatus(frame->data[5]));
  } else if (frame->can_id == 0x2A1) {
    control_mode_.store(ControlMode(can_data_to_uint8_t(frame->data)));
    arm_status_.store(ArmStatus(can_data_to_uint8_t(frame->data + 1)));
    move_mode_.store(MoveMode(can_data_to_uint8_t(frame->data + 2)));
  } else if (frame->can_id == 0x2A8) {
    gripper_pos_.store(can_data_to_int32_t(frame->data) / (GRIPPER_ANGLE_MAX * 1000.0f * 1000.0f));
    gripper_effort_.store(can_data_to_uint16_t(frame->data + 4) / 1000.0f);
    gripper_status_.store(GripperStatus(can_data_to_uint8_t(frame->data + 6)));
  }
}

// Public functions

bool PiperInterface::is_arm_enabled() {
  for (int i = 0; i < MOTOR_DOF; i++) {
    if (!driver_status_[i].load().driver_enable_status) {
      return false;
    }
  }
  return true;
}

bool PiperInterface::is_gripper_enabled() {
  return gripper_status_.load().driver_enable_status;
}

void PiperInterface::enable_arm() {
  can_frame_t frame;
  frame.can_id = 0x471;
  frame.data[0] = 0x07;
  frame.data[1] = 0x02;
  std::fill(std::begin(frame.data) + 2, std::end(frame.data), 0);
  transmit(frame);
  sleep_ms(200);
}

void PiperInterface::disable_arm() {
  can_frame_t frame;
  frame.can_id = 0x471;
  frame.data[0] = 0x07;
  frame.data[1] = 0x01;
  std::fill(std::begin(frame.data) + 2, std::end(frame.data), 0);
  transmit(frame);
  sleep_ms(200);
}

void PiperInterface::enable_gripper() {
  set_gripper(0.0f, 0.0f, GripperCode::ENABLE); // enable
  sleep_ms(200);
}

void PiperInterface::disable_gripper() {
  set_gripper(0.0f, 0.0f, GripperCode::DISABLE_AND_CLEAR_ERROR); // disable and clear error
  sleep_ms(200);
}

void PiperInterface::set_emergency_stop(EmergencyStop emergency_stop) {
  can_frame_t frame;
  frame.can_id = 0x150;
  frame.data[0] = static_cast<uint8_t>(emergency_stop);
  std::fill(std::begin(frame.data) + 1, std::end(frame.data), 0);
  transmit(frame);
  sleep_ms(200);
}

/*
 * This function is intended to be called once to switch control modes.
 * It has a sleep, don't call in main control loop.
 */
void PiperInterface::set_arm_mode(ControlMode ctrl_mode, MoveMode move_mode,
                                   uint8_t speed_rate, ArmController arm_controller) {
  if (!is_arm_enabled()) {
    spdlog::warn("Cannot call set_arm_mode without enabling arm!");
    return;
  }
  can_frame_t frame;
  frame.can_id = 0x151;
  frame.data[0] = static_cast<uint8_t>(ctrl_mode);
  frame.data[1] = static_cast<uint8_t>(move_mode);
  frame.data[2] = speed_rate;
  frame.data[3] = static_cast<uint8_t>(arm_controller);
  std::fill(std::begin(frame.data) + 4, std::end(frame.data), 0);
  transmit(frame);
  sleep_ms(200);
}

void PiperInterface::set_to_damping_mode() {
  JointState joint_state;
  Gain gain;
  gain.kp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  gain.kd = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8};

  for (int i = 0; i < 10; i++) {
    set_joint_pos_vel_torque(joint_state, gain);
    sleep_ms(10);
  }
}

/*
 * This code first converts the float values to integers of varying bit
 * widths, and packs them into a CAN frame for each motor in the array.
 * pos (16-bit)|spd (12-bit)|kp (12-bit)|kd (12-bit)|t (8-bit)|crc (4-bit)
 * crc is calculated by XORing the first 7 bytes of the CAN frame. A simple
 * checksum.
 */
void PiperInterface::set_joint_pos_vel_torque(const JointState &joint_state,
                                    const Gain &gain) {
  if (get_move_mode() != MoveMode::MIT) {
    spdlog::error("Cannot call set_joint_pos_vel_torque in non-MIT mode!");
    return;
  }

  for (uint8_t motor_id = 0; motor_id < MOTOR_DOF; ++motor_id) {
    // Check bounds for all parameters
    if (joint_state.pos[motor_id] < POS_MIN ||
        joint_state.pos[motor_id] > POS_MAX) {
      spdlog::error("Warning: Position {} out of bounds [{}, {}] for motor {}",
                    joint_state.pos[motor_id], POS_MIN, POS_MAX, motor_id);
      continue;
    }
    if (joint_state.vel[motor_id] < VEL_MIN ||
        joint_state.vel[motor_id] > VEL_MAX) {
      spdlog::error("Warning: Velocity {} out of bounds [{}, {}] for motor {}",
                    joint_state.vel[motor_id], VEL_MIN, VEL_MAX, motor_id);
      continue;
    }
    if (gain.kp[motor_id] < KP_MIN || gain.kp[motor_id] > KP_MAX) {
      spdlog::error("Warning: Kp {} out of bounds [{}, {}] for motor {}",
                    gain.kp[motor_id], KP_MIN, KP_MAX, motor_id);
      continue;
    }
    if (gain.kd[motor_id] < KD_MIN || gain.kd[motor_id] > KD_MAX) {
      spdlog::error("Warning: Kd {} out of bounds [{}, {}] for motor {}",
                    gain.kd[motor_id], KD_MIN, KD_MAX, motor_id);
      continue;
    }
    if (joint_state.torque[motor_id] < T_MIN ||
        joint_state.torque[motor_id] > T_MAX) {
      spdlog::error("Warning: Torque {} out of bounds [{}, {}] for motor {}",
                    joint_state.torque[motor_id], T_MIN, T_MAX, motor_id);
      continue;
    }

    can_frame_t frame;
    frame.can_id = 0x15A + motor_id;

    int pos_ref = float_to_int(joint_state.pos[motor_id], POS_MIN, POS_MAX, 16);
    int spd_ref = float_to_int(joint_state.vel[motor_id], VEL_MIN, VEL_MAX, 12);
    int kp_int = float_to_int(gain.kp[motor_id], KP_MIN, KP_MAX, 12);
    int kd_int = float_to_int(gain.kd[motor_id], KD_MIN, KD_MAX, 12);
    float kt = (motor_id < 3 ? 1 / 4.0 : 1 / 0.8);
    int t_ref = float_to_int(joint_state.torque[motor_id] * kt, T_MIN, T_MAX, 8);

    frame.data[0] = (pos_ref >> 8) & 0xFF; // High byte
    frame.data[1] = pos_ref & 0xFF;        // Low byte
    frame.data[2] = (spd_ref >> 4) & 0xFF;
    frame.data[3] = (((spd_ref & 0xF) << 4) & 0xF0) | ((kp_int >> 8) & 0x0F);
    frame.data[4] = kp_int & 0xFF;
    frame.data[5] = (kd_int >> 4) & 0xFF;
    frame.data[6] = (((kd_int & 0xF) << 4) & 0xF0) | ((t_ref >> 4) & 0x0F);

    uint8_t crc =
        (frame.data[0] ^ frame.data[1] ^ frame.data[2] ^ frame.data[3] ^
         frame.data[4] ^ frame.data[5] ^ frame.data[6]) &
        0x0F;
    frame.data[7] = ((t_ref << 4) & 0xF0) | crc;

    transmit(frame);
    sleep_us(COMMUNICATION_DELAY);
  }
}

// Even when the status_code is disable, the gripper moves to the position.
void PiperInterface::set_gripper(float position, float effort, GripperCode status_code) {
  int32_t pos_ref = static_cast<int32_t>((position * 0.07) * 1000 * 1000); // 0-1 to m to mm to micrometer
  uint16_t effort_ref = static_cast<uint16_t>(effort * 5000); // 0-1 to Nm to mNm

  can_frame_t frame;
  frame.can_id = 0x159;
  frame.data[0] = (pos_ref >> 24) & 0xFF;
  frame.data[1] = (pos_ref >> 16) & 0xFF;
  frame.data[2] = (pos_ref >> 8) & 0xFF;
  frame.data[3] = pos_ref & 0xFF;
  frame.data[4] = (effort_ref >> 8) & 0xFF;
  frame.data[5] = effort_ref & 0xFF;
  frame.data[6] = static_cast<uint8_t>(status_code);
  frame.data[7] = 0;
  transmit(frame);
  sleep_us(COMMUNICATION_DELAY);

}

JointState PiperInterface::get_current_state() {
  JointState state;
  state.timestamp = get_time_ms().count();
  for (int i = 0; i < MOTOR_DOF; i++) {
    state.pos[i] = qpos_[i].load();
    state.vel[i] = qvel_[i].load();
    state.torque[i] = tau_[i].load();
  }
  state.gripper_pos = gripper_pos_.load();
  state.gripper_effort = gripper_effort_.load();
  return state;
}
