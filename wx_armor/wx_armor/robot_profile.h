#pragma once

#include <chrono>
#include <cstdlib>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace horizon::wx_armor {

enum class OpMode : int {
  PWM = 0,
  POSITION = 1,
  EXT_POSITION = 2,
  CURRENT_BASED_POSITION = 3,
  LINEAR_POSITION = 4,
  VELOCITY = 5,
  CURRENT = 6,
};

enum class MoveMode : int {
  VELOCITY_BASED = 0,
  TIME_BASED = 1,
};

struct MotorInfo {
  // Dynamixel ID of the motor
  uint8_t id = 0;

  std::string name = "unknown";

  OpMode op_mode = OpMode::POSITION;

  MoveMode movement = MoveMode::TIME_BASED;
  std::chrono::milliseconds movement_vel{0};
  std::chrono::milliseconds movement_acc{0};
};

struct RegistryKV {
  uint8_t motor_id;
  std::string key;
  int32_t value;

  RegistryKV(uint8_t motor_id_, const std::string &key_, int32_t value_)
      : motor_id(motor_id_), key(key_), value(value_) {}
};

struct RobotProfile {
  std::vector<MotorInfo> motors{};
  std::vector<RegistryKV> eeprom{};
  // The concept of "Joints" is a subset of all motors. It is a subset
  // because some of the joints can have more than one motors (called
  // "shadow" motors) coverting them.
  std::vector<uint8_t> joint_ids{};
  std::vector<std::string> joint_names{};

  RobotProfile() = default;
  RobotProfile(RobotProfile &&) = default;
  RobotProfile &operator=(RobotProfile &) = default;

  inline const MotorInfo *motor(const std::string name) const {
    for (const MotorInfo &entry : motors) {
      if (entry.name == name) {
        return &entry;
      }
    }
    return nullptr;
  }
};

}  // namespace horizon::wx_armor

namespace YAML {

template <>
struct convert<horizon::wx_armor::RobotProfile> {
  static bool decode(const Node &node,
                     horizon::wx_armor::RobotProfile &profile);
};

}  // namespace YAML
