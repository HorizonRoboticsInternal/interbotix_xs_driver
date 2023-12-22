#pragma once

#include <filesystem>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
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

  struct OnboardReadAddress {
    ControlItem position;
    ControlItem velocity;
    ControlItem current;
  };

  struct OnboardGoalAddress {
    ControlItem position;

    // TODO(breakds): Implement the following when other types of OpMode (i.e.
    // other than Position Control) are needed.

    // ControlItem velocity;
    // ControlItem current;
    // ControlItem pwm;
  };

  OnboardReadAddress reading_;
  OnboardGoalAddress writing_;

  // ┌──────────────────┐
  // │ Mutables         │
  // └──────────────────┘
};

}  // namespace horizon::wx_armor
