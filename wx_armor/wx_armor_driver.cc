#include "wx_armor/wx_armor_driver.h"

#include <cstdlib>

#include "spdlog/spdlog.h"

namespace fs = std::filesystem;

namespace horizon::wx_armor {
namespace {

static constexpr uint32_t DEFAULT_BAUDRATE = 1'000'000;

// Load the configuration file, which should be in YAML format. Upon failure,
// crash the program.
auto LoadConfigOrDie(fs::path config_path) -> YAML::Node {
  try {
    YAML::Node node = YAML::LoadFile(config_path.c_str());
    if (node.IsNull()) {
      spdlog::critical("Failed to read config file {} as it is empty.",
                       config_path.string());
      std::abort();
    }
    return node;
  } catch (YAML::BadFile &error) {
    spdlog::critical("Failed to load the config file {}, due to {}",
                     config_path.string(),
                     error.what());
    std::abort();
  }
}

}  // namespace

WxArmorDriver::WxArmorDriver(const std::string &usb_port,
                             fs::path motor_config_path)
    : profile_(LoadConfigOrDie(motor_config_path).as<RobotProfile>()) {
  // Now, initialize the handle, connecting to the specified usb port. It
  // returns false if the initialization fails.
  if (dxl_wb_.init(usb_port.c_str(), DEFAULT_BAUDRATE)) {
    spdlog::info("Successfully connected to {}", usb_port);
  } else {
    spdlog::critical("Failed to connect to port {}", usb_port);
    std::abort();
  }
}

}  // namespace horizon::wx_armor
