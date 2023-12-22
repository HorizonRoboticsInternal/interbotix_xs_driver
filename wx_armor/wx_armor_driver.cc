#include "wx_armor/wx_armor_driver.h"

#include <cstdlib>

#include "spdlog/spdlog.h"

namespace fs = std::filesystem;

namespace wx_armor {
namespace {

// Load the configuration file, which should be in YAML format. Upon failure,
// crash the program.
auto LoadConfigOrDie(fs::path config_path) -> YAML::Node {
  try {
    return YAML::LoadFile(config_path.c_str());
  } catch (YAML::BadFile &error) {
    spdlog::critical("Failed to load the config file {}, due to {}",
                     config_path.string(),
                     error.what());
    std::abort();
  }
}

}  // namespace

WxArmorDriver::WxArmorDriver(fs::path motor_config_path,
                             fs::path mode_config_path) {
  motor_configs_ = LoadConfigOrDie(motor_config_path);
  mode_configs_ = LoadConfigOrDie(mode_config_path);
}

}  // namespace wx_armor
