#pragma once

#include <chrono>
#include <cstdlib>
#include <vector>

#include "spdlog/spdlog.h"
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
                     horizon::wx_armor::RobotProfile &profile) {
    // Fill in model information.
    for (const auto &child : node["motors"]) {
      YAML::Node info = child.second;
      uint8_t motor_id = info["ID"].as<uint8_t>();
      profile.motors.emplace_back(horizon::wx_armor::MotorInfo{
          .id = motor_id,
          .name = child.first.as<std::string>(),
          // By default, set the operation mode to position control.
          .op_mode = horizon::wx_armor::OpMode::POSITION,
          .movement = horizon::wx_armor::MoveMode::VELOCITY_BASED,
      });

      // Now, populate the register table.
      for (const auto &kv : info) {
        std::string key = kv.first.as<std::string>();
        // Ignore "ID" and "Baud_Rate" as they are not valid register
        // names on the motor's internal EEPROM register table.
        if (key == "ID" || key == "Baud_Rate") continue;
        profile.eeprom.emplace_back(motor_id, key, kv.second.as<int32_t>());
      }
    }

    // Fill in joint IDs.
    for (const auto &child : node["joint_order"]) {
      std::string name = child.as<std::string>();
      const horizon::wx_armor::MotorInfo *motor = profile.motor(name);
      if (motor == nullptr) {
        spdlog::critical(
            "Malformed motor config. Cannot find a motor named '{}', which "
            "appears in 'joint_order'",
            name);
      }
      profile.joint_ids.emplace_back(motor->id);
      profile.joint_names.emplace_back(motor->name);
    }

    return true;
  }
};

}  // namespace YAML
