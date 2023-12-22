#pragma once

#include "yaml-cpp/yaml.h"

#include <filesystem>

namespace wx_armor {

class WxArmorDriver {
 public:
  WxArmorDriver(std::filesystem::path motor_config_path,
                std::filesystem::path mode_config_path);

 private:
  YAML::Node motor_configs_;
  YAML::Node mode_configs_;
};

}  // namespace wx_armor
