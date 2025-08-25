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
#include <optional>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "nlohmann/json.hpp"
#include "wx_armor/robot_profile.h"
#include "yaml-cpp/yaml.h"

namespace horizon::wx_armor
{

/**
 * @struct SensorData
 * @brief Represents sensor readings of the robot's joints.
 *
 * This structure encapsulates the sensor data related to the robot's joints,
 * including positions, velocities, and electric currents of the motors. It also
 * includes a timestamp to indicate the time at which these readings were taken
 * or requested *
 */
struct SensorData
{
    // ┌──────────────────┐
    // │ Per Joint        │
    // └──────────────────┘

    std::vector<float> pos{};    // joint position
    std::vector<float> vel{};    // joint velocity
    std::vector<float> crt{};    // motor electric current of this joint
    std::vector<int32_t> err{};  // per joint error codes

    // ┌──────────────────┐
    // │ Metadata         │
    // └──────────────────┘

    // The timestamp at which the sensor data is requested, which is
    // approximately when the measurement is taken. Since it is an approximation
    // with around 2ms error, it is only expected to be used as a reference.
    int64_t timestamp = 0;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(SensorData, pos, vel, crt, err, timestamp);
};

/**
 * @struct PIDGain
 * @brief Holds the PID gain values for a specific motor or a group of motors.
 *
 * This structure is used to configure the PID gains (Proportional, Integral,
 * Derivative) for motors in a robot. It supports setting gains by specifying
 * motor names or applying the same gains to all motors by using "all" as the
 * motor name.
 */
struct PIDGain
{
    //  The name of the motor. Use "all" to apply to all motors.
    std::string name = "";
    // The proportional gain for the PID controller.
    int32_t p = 0;
    // The integral gain for the PID controller.
    int32_t i = 0;
    // The derivative gain for the PID controller.
    int32_t d = 0;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(PIDGain, name, p, i, d);
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
 * @note The WxArmorDriver should never be used by applications directly, as it
 * does not provide any safety protections. Instead, it should only be invoked
 * through the WxArmorWebController class as of now.
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
class WxArmorDriver
{
  public:
    /**
     * @brief Constructs a WxArmorDriver object.
     * @param usb_port The USB port through which the robot is connected.
     * @param motor_config_path Filesystem path to the motor configuration file.
     * @param flash_eeprom Flag to indicate whether to flash the EEPROM on
     * construction.
     */
    WxArmorDriver(const std::string& usb_port, std::filesystem::path motor_config_path,
                  bool flash_eeprom = false);

    ~WxArmorDriver();

    /**
     * @brief Read the latest sensor data from the robot and returns it.
     * @details This method is blocking and typically takes around 2ms to
     * complete.
     *
     * @return The read sensor data if the read is successful. Or std::nullopt
     * if the read fails.
     */
    std::optional<SensorData> Read();

    /**
     * @brief Reads the safety velocity limits from RobotProfile and returns it.
     *
     * @return An array containing the safety velocity limits for each motor
     *         [rad/s].
     */
    std::vector<float> GetSafetyVelocityLimits();

    /**
     * @brief Reads the safety current limits from RobotProfile and returns it.
     *
     * @return An array containing the safety current limits for each motor
     * [mA].
     */
    std::vector<float> GetSafetyCurrentLimits();

    /**
     * @brief Sets the position of the robot's joints, with a desired moving and
     * acceleration time.
     * @details If acc_time is zero, constant velocity is used.
     * @param position A vector of floats representing the desired joint
     * positions.
     * @param moving_time A float in seconds
     * @param acc_time A float in seconds
     */
    void SetPosition(const std::vector<float>& position, float moving_time, float acc_time = 0.0);

    /**
     * @brief Activates the torque in the robot's motors.
     *
     * @details When this method is called, the motors of the robot's joints
     * start generating torque based on the provided commands. This enables the
     * robot to maintain or move to the specified positions. It's essential to
     * call this method before attempting to move the robot, as it 'energizes'
     * the joints, preparing them for active motion. TorqueOn() will be called
     * automatically upon construction of this driver class.
     *
     * @note when the robot is torque back on, it will not remember the previous
     * position command and goto there. It will remain the position when it is
     * torqued on.
     */
    void TorqueOn();

    /**
     * @brief Deactivates the torque in the robot's motors.
     *
     * @details Calling this method will cause the motors of the robot's joints
     * to stop producing torque. As a result, the robot becomes 'soft' and will
     * not resist external forces. This state is useful for safely handling the
     * robot, performing maintenance, or when the robot is in a powered-down or
     * idle state.
     */
    void TorqueOff();

    /**
     * @brief Sets the PID gains for one or more motors based on the provided
     * configurations.
     *
     * This function iterates through a list of PIDGain configurations,
     * applying each configuration to the specified motor or all motors.
     * If a motor's name does not match any in the profile or if "all"
     * is specified but some motors cannot be configured, an error is
     * logged. The function aims for flexible and efficient PID
     * configuration across different motors in a robot using Dynamixel
     * actuators.
     *
     * @param gain_cfgs A vector of PIDGain structures containing the gain
     * configurations for the motors. Each structure specifies a motor name and
     * the PID gains to apply. If the name is "all", the gains are applied to
     * all motors.
     *
     * Usage Examples:
     *     - To set PID gains for a specific motor: [{"name": "waist", "p":
     *       800, "i": 0, "d": 3}]
     *     - To set PID gains for all motors, with a different configuration for
     *       one motor: [{"name": "all", "p": 800, "i": 0, "d": 3}, {"name":
     *       "wrist", "p": 400, "i": 0, "d": 0}]
     */
    void SetPID(const std::vector<PIDGain>& gain_cfgs);

    /**
     * @brief Returns the current safety violation mode status.
     *
     * @return A bool describing whether a safety violation is currently
     * triggered and not reset.
     */
    bool SafetyViolationTriggered();

    /**
     * @brief Sets off safety violation mode.
     *
     * @details For functionality to return to normal, ResetSafetyViolation()
     * must be called.
     */
    void TriggerSafetyViolationMode();

    /**
     * @brief Resets the safety violation status back to false.
     *
     * @note Currently, this gets called only when a new client connection
     * is made after all previous connections are closed.
     */
    void ResetSafetyViolationMode();

    /**
     * @brief Resets the gripper delta counter to 0
     */
    void ResetGripperClosingCounter() {
        closing_iters_ = 0;
    }

    /**
     * @brief Returns the profile of the robot.
     */
    const RobotProfile& Profile() const {
        return profile_;
    }

    /**
     * @brief Reboot any motors in an error state.
     */
    void RebootMotors();

  private:
    ControlItem AddItemToRead(const std::string& name);

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
    ControlItem read_error_address_;

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

    // Index to the write handler that writes both the position and velocity and
    // acceleration profile, and the corresponding register addresses.
    uint8_t write_position_and_profile_handler_index_;

    // Flag that gets triggered when safety violations such as
    // velocity limits are violated.
    std::atomic_bool safety_violation_{false};

    // A tracker for how many iterations the gripper has been closing for
    // without an open command.
    std::atomic<uint32_t> closing_iters_{0};

    // The current gripper position to be updated by Read().
    // Used for delta control when gripper is closing.
    std::atomic<double> gripper_position_{1.0};
};

// Helper function to read the environment variable.
// Used by the Driver() function below.
template <typename T>
T GetEnv(const char* name, T default_value) {
    const char* text = std::getenv(name);

    if (text == nullptr) {
        return default_value;
    }

    if constexpr (std::is_same_v<T, std::string>) {
        return std::string(text);
    }
    else if constexpr (std::is_integral_v<T>) {
        return static_cast<T>(std::stoi(text));
    }
    else if constexpr (std::is_same_v<T, std::filesystem::path>) {
        return std::filesystem::path(text);
    }

    std::abort();
}

/**
 * @brief Helper function for issuing a command a decelerate to zero command to the driver.
 *
 * @param curr_reading The current sensor reading.
 * @param dt The current control time step.
 * @param deceleration_time The time to decelerate over.
 */
void SlowDownToStop(const SensorData& curr_reading, float dt = 0.5, float deceleration_time = 1.5);

/**
 * @brief Getter for the driver.
 *
 * Can be used by any downstream logic and threads.
 *
 * @return A pointer to the driver.
 */
WxArmorDriver* Driver();

}  // namespace horizon::wx_armor
