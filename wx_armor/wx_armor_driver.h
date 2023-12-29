#pragma once

#include <filesystem>
#include <memory>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "wx_armor/io.h"
#include "wx_armor/robot_profile.h"
#include "yaml-cpp/yaml.h"

namespace horizon::wx_armor {

class WxArmorDriver {
 public:
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
  // │ Mutables         │
  // └──────────────────┘

  // Note that each call to `JointStateReader::ReadTo` takes around 12ms to
  // finish, under the default baud rate 1000000.
  std::unique_ptr<JointStateReader> reader_ = nullptr;
  std::vector<float> position_;
  std::vector<float> velocity_;
  std::vector<float> current_;
};

}  // namespace horizon::wx_armor
