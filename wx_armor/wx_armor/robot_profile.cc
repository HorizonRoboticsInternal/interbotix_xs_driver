#include "wx_armor/robot_profile.h"

#include "spdlog/spdlog.h"

namespace YAML {

bool convert<horizon::wx_armor::RobotProfile>::decode(
    const Node &node, horizon::wx_armor::RobotProfile &profile) {
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

}  // namespace YAML
