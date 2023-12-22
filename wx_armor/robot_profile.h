#pragma once

#include <chrono>
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

class EEPROMRegisterTable {
 public:
  struct Entry {
    uint8_t motor_id;
    std::string key;
    int32_t value;
  };

  EEPROMRegisterTable() = default;

  inline void Add(uint8_t motor_id, const std::string &key, int32_t value) {
    entries_.emplace_back(Entry{
        .motor_id = motor_id,
        .key = key,
        .value = value,
    });
  }

 private:
  std::vector<Entry> entries_;
};

struct RobotProfile {
  std::vector<MotorInfo> motors{};
  EEPROMRegisterTable eeprom{};

  RobotProfile() = default;
  RobotProfile(RobotProfile &&) = default;
  RobotProfile &operator=(RobotProfile &) = default;
};

}  // namespace horizon::wx_armor

namespace YAML {

template <>
struct convert<horizon::wx_armor::RobotProfile> {
  static bool decode(const Node &node,
                     horizon::wx_armor::RobotProfile &profile) {
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
        profile.eeprom.Add(motor_id, key, kv.second.as<int32_t>());
      }
    }
    return true;
  }
};

}  // namespace YAML
