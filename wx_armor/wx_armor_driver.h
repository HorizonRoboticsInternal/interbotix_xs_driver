#pragma once

#include <chrono>
#include <filesystem>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "wx_armor/robot_profile.h"
#include "yaml-cpp/yaml.h"

namespace horizon::wx_armor {

class WxArmorDriver {
 public:
  struct MotorInfo {
    // Dynamixel ID of the motor
    uint8_t id;

    // The operating mode of the motor.
    std::string mode;

    std::string profile_type;
    std::chrono::milliseconds profile_vel;
    std::chrono::milliseconds profile_acc;
  };

  WxArmorDriver(const std::string &usb_port,
                std::filesystem::path motor_config_path);

 private:
  // ┌──────────────────┐
  // │ The motor handle │
  // └──────────────────┘

  DynamixelWorkbench dxl_wb_;

  // ┌───────────────┐
  // │ Info (static) │
  // └───────────────┘
  RobotProfile profile_;
  std::vector<MotorInfo> motor_states_;

  // ┌──────────────────┐
  // │ States (mutable) │
  // └──────────────────┘
};

}  // namespace horizon::wx_armor
