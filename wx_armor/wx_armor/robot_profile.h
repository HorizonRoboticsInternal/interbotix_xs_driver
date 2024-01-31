/* This library provides RobotProfile - a class that holds the information about
 * a robot assembled based on and powered by Dynamixel motors.
 *
 * Key concepts are:
 *
 * Joint: Such a robot consists of joints. Each joint is usually 1D rotary where
 *        the position of the joint is a radian describing the angle between the
 *        two parts that this joints connects.
 *
 * Motor: Each joint may be powered by a single Dynamixel Motor or multiple
 *        Dynamixel motors. In the latter case, the joint is usually powered by
 *        multiple identical Dynamixel Motors whose positions are always
 *        identical to each other. One of the motors will be called the master
 *        motor, and the rest of them are called the shadow motors. A typical
 *        multi-motor setup involves one master motor and one shadow motor, such
 *        as in WidowX 250s case. To read the current position of such a joint,
 *        the user only need to read the position of the master motor.
 *
 * EEPROM and RAM: On each Dynamixel motor, there are two storage areas, based
 *        on EEPROM and RAM respectively. EEPROM area stores the data that are
 *        not frequently changed such as parameters for the firmware, where the
 *        RAM area is used for I/O on motor sensor reading and command sending.
 */

#pragma once

#include <chrono>
#include <cstdlib>
#include <map>
#include <string_view>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace horizon::wx_armor {

// Enum for the operating mode of the underlying Dynamixel motors. This
// determines how the underlying controller works and what kind of commands the
// motor accepts.
enum class OpMode : int {
  CURRENT = 0,                 // Electric current controller
  VELOCITY = 1,                // Velocity controller
  POSITION = 3,                // Position controller
  CURRENT_BASED_POSITION = 5,  // Current based position controller
  PWM = 16,                    // Pulse-width modulation controller
  TORQUE = 100,                // Torque controller
};

// Convert the OpMode enum to a string e.g. for debugging purpose.
std::string_view OpModeName(OpMode mode);

// Stores the metadata of a Dynamixel motor in the robot. Each motor in the
// robot is associated with an "ID". When communicating with the motors, the
// APIs use the id to specify which motor they talk to.
struct MotorInfo {
  // Dynamixel ID of the motor
  uint8_t id = 0;

  // Name of the motor.
  std::string name = "unknown";

  // If the motor is powering the joint together with other shadow motors (which
  // makes this motor a "master" motor), this is a list of IDs of the
  // corresponding shadow motors. Otherwise if the motor is powering the joint
  // alone, this list is empty.
  std::vector<uint8_t> shadow_motor_ids{};

  // The following information are currently not used at this moment. Having
  // them here just to be consistent with dynamixel.
  OpMode op_mode = OpMode::POSITION;
};

// The EERPROM area of each motor is logically organized as a a few key/value
// pairs, similar to a registry. This class is used to represent such a
// key/value pair, together with their associated motor ID.
struct RegistryKV {
  uint8_t motor_id;
  std::string key;
  int32_t value;

  RegistryKV(uint8_t motor_id_, const std::string &key_, int32_t value_)
      : motor_id(motor_id_), key(key_), value(value_) {}
};

// This class stores the full information about a Dynamixel motor based robot.
// From here the user should be able to access
//
// 1. Information about the joints of the robot.
// 2. Information about the motors of the robot.
// 3. Information about the EEPROM registry of the robot.
//
// Such information is usually stored in a yaml file. When parsed, the yaml file
// becomes a RobotProfile instance.
struct RobotProfile {
  std::vector<MotorInfo> motors{};
  std::vector<uint8_t> joint_ids{};
  std::vector<std::string> joint_names{};
  std::vector<RegistryKV> eeprom{};

  RobotProfile() = default;
  RobotProfile(RobotProfile &&) = default;
  RobotProfile &operator=(RobotProfile &) = default;

  // API to access a motor by its name. Returns nullptr if such motor does not
  // exist in this robot.
  inline const MotorInfo *motor(const std::string name) const {
    for (const MotorInfo &entry : motors) {
      if (entry.name == name) {
        return &entry;
      }
    }
    return nullptr;
  }
};

}  // namespace horizon::wx_armor

namespace YAML {

// Specialization of YAML::convert for RobotPRofile. so that we can parse yaml
// into RobotProfile.
template <>
struct convert<horizon::wx_armor::RobotProfile> {
  static bool decode(const Node &node,
                     horizon::wx_armor::RobotProfile &profile);
};

}  // namespace YAML
