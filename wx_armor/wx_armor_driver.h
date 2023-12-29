#pragma once

#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "wx_armor/robot_profile.h"
#include "yaml-cpp/yaml.h"

namespace horizon::wx_armor {

struct SensorData {
  // ┌──────────────────┐
  // │ Per Joint        │
  // └──────────────────┘
  std::vector<double> pos{};  // joint position
  std::vector<double> vel{};  // joint velocity
  std::vector<double> crt{};  // motor electric current

  // The timestamp at which the sensor data is requested, which is approximately
  // when the measurement is taken.
  int64_t timestamp = 0;
};

class WxArmorDriver {
 public:
  WxArmorDriver(const std::string &usb_port,
                std::filesystem::path motor_config_path);

  void FetchSensorData();

 private:
  ControlItem AddItemToRead(const std::string &name);
  void InitReadHandler();

  // ┌──────────────────┐
  // │ The motor handle │
  // └──────────────────┘

  DynamixelWorkbench dxl_wb_;

  // ┌───────────────┐
  // │ Info (static) │
  // └───────────────┘
  RobotProfile profile_;

  // ┌──────────────────┐
  // │ Read             │
  // └──────────────────┘

  // Note that each call to `JointStateReader::ReadTo` takes around 12ms to
  // finish, under the default baud rate 1000000.
  std::mutex read_handler_mutex_;
  uint8_t read_handler_index_ = 0;

  std::mutex latest_reading_mutex_;
  SensorData latest_reading_;

  // A ControlItem is Dynamixel SDK's terminology of describing the address
  // where the corresponding information is stored on each motor. Each address
  // (i.e. ControlItem) consists of a starting address and a length, where the
  // unit is of Bytes.
  ControlItem read_position_address_;
  ControlItem read_velocity_address_;
  ControlItem read_current_address_;

  // Union address interval of the above 3. Note that `read_end_` address is
  // not inclusive (open interval). We need this so that we can issue command
  // to the motors to read such data resides within the address interval.
  uint16_t read_start_ = std::numeric_limits<uint16_t>::max();
  uint16_t read_end_ = 0;
};

}  // namespace horizon::wx_armor
