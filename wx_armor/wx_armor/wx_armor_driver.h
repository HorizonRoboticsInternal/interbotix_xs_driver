/**
 * @file wx_armor_driver.h
 * @brief Enhanced Driver for Dynamixel Motor-Based Robots, Modified from
 * Interbotix Stock Driver.
 *
 * This file contains the WxArmorDriver class, an enhanced version of the
 * original stock driver provided by Interbotix for Dynamixel motor-based
 * robots. While it maintains the fundamental structure and initialization logic
 * of the original driver, several key improvements and modifications have been
 * made to enhance its functionality and reliability.
 *
 * Key Enhancements and Differences:
 * 1. **Smart Reboot and EEPROM Writing**:
 *    - The initialization procedure follows the original driver's logic but
 * incorporates a 'smart reboot' feature. This feature checks for motors in an
 * error state during initialization and reboots them if necessary.
 *    - The EEPROM writing has been optimized to only update values that differ
 * from their existing state. This selective writing approach helps in extending
 * the lifespan of the EEPROM.
 *
 * 2. **Calibration Consistency**:
 *    - The calibration process for master motors and their corresponding shadow
 * motors remains consistent with the original stock driver, ensuring
 * compatibility and reliability in multi-motor joint configurations.
 *
 * 3. **Thread-Safety with Locks**:
 *    - Thread-safety has been significantly improved by adding locks
 * (`io_mutex_`) to prevent simultaneous use of the read and write handlers.
 * This enhancement ensures that the driver can be safely used in multi-threaded
 * applications without risking data corruption or race conditions.
 *
 * @note These enhancements aim to provide a more robust, efficient, and
 * thread-safe driver while maintaining the core functionalities and
 * compatibility with the original Interbotix driver.
 */

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

/**
 * @struct SensorData
 * @brief Represents sensor readings of the robot's joints.
 *
 * This structure encapsulates the sensor data related to the robot's joints,
 * including positions, velocities, and electric currents of the motors. It also
 * includes a timestamp to indicate the time at which these readings were taken
 * or requested *
 */
struct SensorData {
  // ┌──────────────────┐
  // │ Per Joint        │
  // └──────────────────┘

  std::vector<float> pos{};  // joint position
  std::vector<float> vel{};  // joint velocity
  std::vector<float> crt{};  // motor electric current of this joint

  // ┌──────────────────┐
  // │ Metadata         │
  // └──────────────────┘

  // The timestamp at which the sensor data is requested, which is approximately
  // when the measurement is taken. Since it is an approximation with around 2ms
  // error, it is only expected to be used as a reference.
  int64_t timestamp = 0;

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(SensorData, pos, vel, crt, timestamp);
};

/**
 * @class WxArmorDriver
 * @brief Driver class for controlling and interfacing with Dynamixel
 * motor-based robots.
 *
 * This class provides the necessary functionalities to control a robot powered
 * by Dynamixel motors. It includes methods for initializing the robot, fetching
 * sensor data, setting joint positions, and enabling/disabling motor torque.
 * The driver communicates with the motors through a specified USB port and
 * follows configurations defined in a motor configuration file.
 *
 * Usage:
 * @code
 *   WxArmorDriver driver("/dev/ttyUSB0", "path/to/motor_config.yaml");
 *   driver.TorqueOn();
 *   driver.SetPosition({1.0, 1.5, 1.2});
 *   driver.FetchSensorData();
 *   auto sensorJson = driver.SensorDataToJson();
 * @endcode
 *
 * @note This driver is specific to robots with Dynamixel motors and requires
 * the Dynamixel SDK for low-level communications.
 */
class WxArmorDriver {
 public:
  /**
   * @brief Constructs a WxArmorDriver object.
   * @param usb_port The USB port through which the robot is connected.
   * @param motor_config_path Filesystem path to the motor configuration file.
   * @param flash_eeprom Flag to indicate whether to flash the EEPROM on
   * construction.
   */
  WxArmorDriver(const std::string &usb_port,
                std::filesystem::path motor_config_path,
                bool flash_eeprom = false);

  ~WxArmorDriver();

  /**
   * @brief Fetches the latest sensor data from the robot and caches it.
   * @details This method is blocking and typically takes around 2ms to
   * complete.
   */
  SensorData Read();

  /**
   * @brief Sets the position of the robot's joints.
   * @param position A vector of floats representing the desired joint
   * positions.
   */
  void SetPosition(const std::vector<float> &position);

  /**
   * @brief Activates the torque in the robot's motors.
   *
   * @details When this method is called, the motors of the robot's joints start
   * generating torque based on the provided commands. This enables the robot to
   * maintain or move to the specified positions. It's essential to call this
   * method before attempting to move the robot, as it 'energizes' the joints,
   * preparing them for active motion. TorqueOn() will be called automatically
   * upon construction of this driver class.
   *
   * @note when the robot is torque back on, it will not remember the previous
   * position command and goto there. It will remain the position when it is
   * torqued on.
   */
  void TorqueOn();

  /**
   * @brief Deactivates the torque in the robot's motors.
   *
   * @details Calling this method will cause the motors of the robot's joints to
   * stop producing torque. As a result, the robot becomes 'soft' and will not
   * resist external forces. This state is useful for safely handling the robot,
   * performing maintenance, or when the robot is in a powered-down or idle
   * state.
   */
  void TorqueOff();

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

  std::mutex io_mutex_;  // protects IO from dxl_wb_.

  uint8_t read_handler_index_ = 0;

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

  // ┌──────────────────┐
  // │ Write            │
  // └──────────────────┘

  // Unlike write, in the future we may have write handler for each of the
  // operation mode (e.g. position control, velocity control, pwm control).
  // Therefore we will need to store handler index for each of them.
  uint8_t write_position_handler_index_;

  // Similar to how we read, we also write to identical addresses on each
  // motor.
  ControlItem write_position_address_;
};

}  // namespace horizon::wx_armor
