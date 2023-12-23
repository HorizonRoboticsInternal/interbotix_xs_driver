#include "wx_armor/wx_armor_driver.h"

#include <chrono>
#include <cstdlib>
#include <thread>

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

auto PingMotors(DynamixelWorkbench *dxl_wb,
                const RobotProfile &profile,
                int num_trials = 3,
                std::chrono::milliseconds sleep_between_trials =
                    std::chrono::milliseconds(200)) -> bool {
  const char *log;

  std::set<uint8_t> success{};

  for (int i = 0; i < num_trials; ++i) {
    for (const MotorInfo motor : profile.motors) {
      if (success.count(motor.id) > 0) continue;
      if (dxl_wb->ping(motor.id, &log)) {
        success.emplace(motor.id);
        std::string model_name = dxl_wb->getModelName(motor.id);
        spdlog::info("Found DYNAMIXEL Motor ID: {}, Model: {}, Name: {}",
                     motor.id,
                     model_name,
                     motor.name);
        if (model_name == "XL-320") {
          spdlog::warn(
              "Model XL-320's current reading is not supported by this "
              "driver, because its effort is load (%) based.");
        }
      } else {
        spdlog::error(
            "FAILED to ping Motor ID: {}, Name: {}", motor.id, motor.name);
        break;
      }
    }
    if (success.size() == profile.motors.size()) {
      return true;
    } else if (i + 1 < num_trials) {
      spdlog::warn("Found only {} / {} motors. Will wait for {} ms and retry.",
                   success.size(),
                   profile.motors.size(),
                   sleep_between_trials.count());
      std::this_thread::sleep_for(sleep_between_trials);
    }
  }
  return false;
}

}  // namespace

WxArmorDriver::WxArmorDriver(const std::string &usb_port,
                             fs::path motor_config_path)
    : profile_(LoadConfigOrDie(motor_config_path).as<RobotProfile>()),
      position_(profile_.joint_ids.size()),
      velocity_(profile_.joint_ids.size()),
      current_(profile_.joint_ids.size()) {
  // Now, initialize the handle, connecting to the specified usb port. It
  // returns false if the initialization fails.
  if (dxl_wb_.init(usb_port.c_str(), DEFAULT_BAUDRATE)) {
    spdlog::info("Successfully connected to {}", usb_port);
  } else {
    spdlog::critical("Failed to connect to port {}", usb_port);
    std::abort();
  }

  if (dxl_wb_.getProtocolVersion() != 2.0) {
    spdlog::critical("Requires protocol 2.0, but got {:.1f}",
                     dxl_wb_.getProtocolVersion());
  }

  // For WindowX 250s, we are expecting
  //
  // Found DYNAMIXEL Motor ID: 1, Model: XM430-W350, Name: waist
  // Found DYNAMIXEL Motor ID: 2, Model: XM430-W350, Name: shoulder
  // Found DYNAMIXEL Motor ID: 3, Model: XM430-W350, Name: shoulder_shadow
  // Found DYNAMIXEL Motor ID: 4, Model: XM430-W350, Name: elbow
  // Found DYNAMIXEL Motor ID: 5, Model: XM430-W350, Name: elbow_shadow
  // Found DYNAMIXEL Motor ID: 6, Model: XM430-W350, Name: forearm_roll
  // Found DYNAMIXEL Motor ID: 7, Model: XM430-W350, Name: wrist_angle
  // Found DYNAMIXEL Motor ID: 8, Model: XL430-W250, Name: wrist_rotate
  // Found DYNAMIXEL Motor ID: 9, Model: XL430-W250, Name: gripper
  if (!PingMotors(&dxl_wb_, profile_)) {
    spdlog::critical("Could not find all specified motors.");
    std::abort();
  }

  reader_ = std::make_unique<JointStateReader>(&dxl_wb_, profile_.joint_ids);

  for (int i = 0; i < 10; ++i) {
    spdlog::info("Start Read.");
    reader_->ReadTo(&position_, &velocity_, &current_);
    spdlog::info("Positions: {}, {}, {}, {}, {}, {}, {}",
                 position_[0],
                 position_[1],
                 position_[2],
                 position_[3],
                 position_[4],
                 position_[5],
                 position_[6]);
  }

  // The EEPROM on the motors has a lifespan about 100,000 write
  // cycles. However, reading from eeprom does not affect it lifespan
  //
  // TODO(breakds): We should only write to it if we find discrepancies, i.e.
  // write-on-change.
}

}  // namespace horizon::wx_armor
