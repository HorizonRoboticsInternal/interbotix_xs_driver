#pragma once

#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "nlohmann/json.hpp"
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

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(SensorData, pos, vel, crt, timestamp);
};

class WxArmorDriver {
 public:
  WxArmorDriver(const std::string &usb_port,
                std::filesystem::path motor_config_path);

  // Blocking. Each call to this should take around 2ms.
  void FetchSensorData();

  nlohmann::json SensorDataToJson() const;

  void SetPosition(const std::vector<double> &position);

  void StartLoop();

 private:
  ControlItem AddItemToRead(const std::string &name);

  void InitReadHandler();

  void InitWriteHandler();

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

  std::mutex read_handler_mutex_;
  uint8_t read_handler_index_ = 0;

  mutable std::mutex latest_reading_mutex_;
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

  std::jthread read_loop_thread_{};

  // ┌──────────────────┐
  // │ Write            │
  // └──────────────────┘

  std::mutex write_handler_mutex_;

  // Unlike write, in the future we may have write handler for each of the
  // operation mode (e.g. position control, velocity control, pwm control).
  // Therefore we will need to store handler index for each of them.
  uint8_t write_position_handler_index_;

  // Similar to how we read, we also write to identical addresses on each
  // motor.
  ControlItem write_position_address_;
};

}  // namespace horizon::wx_armor
