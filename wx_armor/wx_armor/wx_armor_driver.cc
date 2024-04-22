#include "wx_armor/wx_armor_driver.h"

#include <cassert>
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

// The driver is a long running service. There are cases that when the driver
// starts running (either because of power on or restarted from previous
// failure) the USB cable connecting the arm (U2D2) and the runnig host (usually
// a Raspberry Pi) isn't plugged in yet. In this case the driver will be waiting
// for the port to get ready (i.e. for the cable to be plugged in).
void WaitUntilPortAvailable(DynamixelWorkbench *dxl_wb,
                            const std::string usb_port) {
  spdlog::info("Still waiting for USB port {} to be available ...", usb_port);
  while (true) {
    // Note that for WidowX 250s, if we do not use the default Baudrate 1000000,
    // the communication between the driver and the robotic arm WILL NOT work.
    bool success = dxl_wb->init(usb_port.c_str(), DEFAULT_BAUDRATE);
    if (success) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  spdlog::info("Successfully connected to {}", usb_port);
}

// This makes sure that all the motors defined in the robot profile are
// reachable. Returns `false` if not all of them are reachable.
auto PingMotors(DynamixelWorkbench *dxl_wb,
                const RobotProfile &profile,
                int num_trials = 3,
                std::chrono::milliseconds sleep_between_trials =
                    std::chrono::milliseconds(200)) -> bool {
  const char *log;

  std::set<uint8_t> success{};

  for (int i = 0; i < num_trials; ++i) {
    for (const MotorInfo &motor : profile.motors) {
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

void FlashEEPROM(DynamixelWorkbench *dxl_wb, const RobotProfile &profile) {
  const char *log;

  size_t num_failed = 0;
  for (const RegistryKV &kv : profile.eeprom) {
    int32_t current_value = 0;

    // Hacky fix for now... Probably a better way to handle this
    // Safety_Velocity_Limit is our own custom param. Don't write it to EEPROM
    if (kv.key == "Safety_Velocity_Limit") {
      continue;
    }
    dxl_wb->itemRead(kv.motor_id, kv.key.c_str(), &current_value);

    // The EEPROM on the motors has a limited number of writes during its
    // lifespan. Only write when there is a discrepancy between the intended
    // value and the current value.
    if (current_value == kv.value) {
      continue;
    }

    if (!dxl_wb->itemWrite(kv.motor_id, kv.key.c_str(), kv.value, &log)) {
      spdlog::error(
          "Failed to flash EEPROM for key value pair ({}, {}) on motor ID = "
          "{}: {}",
          kv.key,
          kv.value,
          kv.motor_id,
          log);
      ++num_failed;
    } else {
      spdlog::info("Value '{}' of Motor {} is set to {}.",
                   kv.key,
                   kv.motor_id,
                   kv.value);
    }
  }

  if (num_failed > 0) {
    spdlog::critical(
        "Failed to flash all registry key value pairs to EEPROM (failure "
        "{} / {}).",
        num_failed,
        profile.eeprom.size());
    std::abort();
  } else {
    spdlog::info(
        "Successfully flashed all registry key value pairs to EEPROM.");
  }
}

// Calibrate the shadow motors so that their 0 position is identical to the 0
// position of their corresponding master motor. This is done by setting the
// homing offset of the shadow motors.
void CalibrateShadowOrDie(DynamixelWorkbench *dxl_wb,
                          const RobotProfile &profile) {
  for (const MotorInfo &motor : profile.motors) {
    // Skip the motor that does not have any shadow motors to calibrate.
    if (motor.shadow_motor_ids.empty()) {
      continue;
    }

    int32_t master_position = 0;
    dxl_wb->itemRead(motor.id, "Present_Position", &master_position);

    for (int32_t shadow_id : motor.shadow_motor_ids) {
      // In order to read the baseline shadow position without homing
      // offset, first set the homing offset to 0.
      if (!dxl_wb->itemWrite(shadow_id, "Homing_Offset", 0)) {
        spdlog::critical("Failed to reset homing offset of motor {} to 0.",
                         shadow_id);
        std::abort();
      }

      int32_t shadow_position = 0;
      int32_t shadow_drive_mode = 0;
      dxl_wb->itemRead(shadow_id, "Present_Position", &shadow_position);
      dxl_wb->itemRead(shadow_id, "Drive_Mode", &shadow_drive_mode);

      // Now we are going to look at the 1st bit of the drive mode, which
      // determines whether the motor is in Normal Mode or Reverse Mode.
      //
      // drive_mode & 1 == 0, Normal Mode, position⇧ = rotate CCW
      //
      // drive_mode & 1 == 1, Reverse Mode, position⇧ = rotate CW
      int32_t homing_offset = ((shadow_drive_mode & 1) == 1)
                                  ? shadow_position - master_position
                                  : master_position - shadow_position;
      if (!dxl_wb->itemWrite(shadow_id, "Homing_Offset", homing_offset)) {
        spdlog::critical(
            "Failed to write homing offset of motor {} during shadow motor "
            "calibration",
            shadow_id);
        std::abort();
      } else {
        spdlog::info(
            "Successfully calibrated motor {} and {} with homing offset = {}",
            motor.id,
            shadow_id,
            homing_offset);
      }
    }
  }
}

// Set the current limit of each motor. The current limit effectively determines
// the maximum torque each motor can generate. This can protect the gears inside
// the motor when there is a huge external force being applied (e.g. robot
// hitting the floor).
//
// If the current limit is set to 0, it means that there is no current limit. In
// this case all the motors will work under "position control" operation mode.
//
// If the current limit is a non-zero integer, it means that we want to apply
// the given current limit. In this case all the motors (except for the
// XL-series motors) will work under "current-based position control" operation
// mode, which can be thought of as "position control" operation with a current
// limit. The XL-series motors will still be under "position control" mode.
//
// The WidowX 250s robotic arm has 7 XM-series motors and 2-XL series motors.
void SetCurrentLimit(DynamixelWorkbench *dxl_wb,
                     RobotProfile *profile,
                     int32_t current_limit) {
  const char *log = nullptr;

  for (MotorInfo &motor : profile->motors) {
    std::string model_name = dxl_wb->getModelName(motor.id);

    // If the current limit (effectively torque limit) is set to a non-zero
    // value, we should use current-based position controller as the operation
    // mode, so that we can set current limit to the motor.
    //
    // There is one exception. If the motor is an XL series motor, it does
    // support current-based position controller operation mode. In this case,
    // we are still going to use the default position controller operation mode.
    if (current_limit == 0 || model_name.substr(0, 2) == "XL") {
      motor.op_mode = OpMode::POSITION;
      if (!dxl_wb->setPositionControlMode(motor.id, &log)) {
        spdlog::critical(
            "Failed to set OpMode to POSITION on motor {} (id = {}): {}",
            motor.name,
            motor.id,
            log);
        std::abort();
      }
    } else {
      motor.op_mode = OpMode::CURRENT_BASED_POSITION;
      if (!dxl_wb->currentBasedPositionMode(motor.id, current_limit, &log)) {
        spdlog::critical(
            "Failed to set OpMode to POSITION on motor {} (id = {}): {}",
            motor.name,
            motor.id,
            log);
        std::abort();
      }
    }
    spdlog::info("Successfully set OpMode to {} on motor {} (id = {})",
                 OpModeName(motor.op_mode),
                 motor.name,
                 motor.id);
  }
}

// Check all the motors and see whether there are motors in error state. If so,
// that motor is rebooted. This function is called at initialization.
void RebootMotorIfInErrorState(DynamixelWorkbench *dxl_wb,
                               const RobotProfile &profile) {
  const char *log;
  int32_t value = 0;
  for (const MotorInfo &motor : profile.motors) {
    bool success =
        dxl_wb->itemRead(motor.id, "Hardware_Error_Status", &value, &log);

    if (success && value == 0) {
      continue;
    } else if (dxl_wb->reboot(motor.id, &log)) {
      spdlog::info("Motor {} '{}' was in error state, and is rebooted.",
                   motor.id,
                   motor.name);
    } else {
      spdlog::critical(
          "Motor {} '{}' was in error state, but fail to reboot it: {}",
          motor.id,
          motor.name,
          log);
      std::abort();
    }
  }
}

}  // namespace

WxArmorDriver::WxArmorDriver(const std::string &usb_port,
                             fs::path motor_config_path,
                             bool flash_eeprom,
                             int32_t current_limit,
                             bool gripper_use_pwm_control)
    : profile_(LoadConfigOrDie(motor_config_path).as<RobotProfile>()) {
  WaitUntilPortAvailable(&dxl_wb_, usb_port);

  if (dxl_wb_.getProtocolVersion() != 2.0) {
    spdlog::critical("Requires protocol 2.0, but got {:.1f}",
                     dxl_wb_.getProtocolVersion());
  }

  // For WindowX 250s, we are expecting 9 motors
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

  // Torque off so that we can write EEPROM, calibrate shadow motors and so on.
  TorqueOff();

  RebootMotorIfInErrorState(&dxl_wb_, profile_);
  if (flash_eeprom) {
    // Note that FlashEEPROM performs "write-on-diff", meaning that it will not
    // write if the desired value and current value are the same. This can
    // effectively extend the lifespan of the EEPROM because it has limited
    // number of writes.
    FlashEEPROM(&dxl_wb_, profile_);
  }
  CalibrateShadowOrDie(&dxl_wb_, profile_);
  SetCurrentLimit(&dxl_wb_, &profile_, current_limit);

  if (gripper_use_pwm_control) {
    UsePWMControlOnGripper();
  }

  // Now torque back on.
  TorqueOn();

  InitReadHandler();
  InitWriteHandler();
}

WxArmorDriver::~WxArmorDriver() {}

std::optional<SensorData> WxArmorDriver::Read() {
  std::vector<int32_t> buffer(profile_.joint_ids.size());
  const uint8_t num_joints = static_cast<uint8_t>(buffer.size());

  SensorData result = SensorData{
      .pos = std::vector<float>(num_joints),
      .vel = std::vector<float>(num_joints),
      .crt = std::vector<float>(num_joints),
  };

  std::unique_lock<std::mutex> handler_lock{io_mutex_};
  const char *log;

  if (!dxl_wb_.syncRead(
          read_handler_index_, profile_.joint_ids.data(), num_joints, &log)) {
    spdlog::warn("Failed to syncRead: {}", log);
    return std::nullopt;
  }

  // We use the time here as the timestamp for the latest reading. This is,
  // however, an approximation. It won't be much off because the syncRead above
  // typically takes 2ms to complete.
  result.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();

  // 1. Extract Position

  if (!dxl_wb_.getSyncReadData(read_handler_index_,
                               profile_.joint_ids.data(),
                               num_joints,
                               read_position_address_.address,
                               read_position_address_.data_length,
                               buffer.data(),
                               &log)) {
    spdlog::critical("Cannot getSyncReadData (position): {}", log);
    std::abort();
  }

  for (size_t i = 0; i < profile_.joint_ids.size(); ++i) {
    result.pos[i] =
        dxl_wb_.convertValue2Radian(profile_.joint_ids[i], buffer[i]);
  }

  // 2. Extract Velocity

  if (!dxl_wb_.getSyncReadData(read_handler_index_,
                               profile_.joint_ids.data(),
                               num_joints,
                               read_velocity_address_.address,
                               read_velocity_address_.data_length,
                               buffer.data(),
                               &log)) {
    spdlog::critical("Cannot getSyncReadData (velocity): {}", log);
    std::abort();
  }

  for (size_t i = 0; i < profile_.joint_ids.size(); ++i) {
    result.vel[i] =
        dxl_wb_.convertValue2Velocity(profile_.joint_ids[i], buffer[i]);
  }

  // 3. Extract Current

  if (!dxl_wb_.getSyncReadData(read_handler_index_,
                               profile_.joint_ids.data(),
                               num_joints,
                               read_current_address_.address,
                               read_current_address_.data_length,
                               buffer.data(),
                               &log)) {
    spdlog::critical("Cannot getSyncReadData (current): {}", log);
    std::abort();
  }

  for (size_t i = 0; i < profile_.joint_ids.size(); ++i) {
    result.crt[i] =
        dxl_wb_.convertValue2Current(profile_.joint_ids[i], buffer[i]);
  }

  return std::move(result);
}

std::vector<float> WxArmorDriver::GetSafetyVelocityLimits() {
  std::vector<float> safety_velocity_limits;

  int counter = 0;
  for (const auto &motor : profile_.motors) {
    // Skip the shadowed motors, which have IDs 3 and 5.
    if (counter != 3 && counter != 5) {
      safety_velocity_limits.push_back(motor.safety_vel_limit);
    }
    counter++;
  }
  return safety_velocity_limits;
}

namespace {

// This class manages a buffer that is used to store the integer commands that
// will be sent to the motor. Following Dynamixel's convention, the buffer is
// organized with DWORD (i.e. 4 bytes) as the unit, where each element is an
// int32. Currently for each motor we are going to write 5 DWORDs, see below.
class CommandBuffer {
 public:
  // According to Dynamixel:
  //
  // Goal PWM         (2 Bytes)     Default =  885 (PWM limit)
  // Goal Current     (2 Bytes)     Default = 1193 (Current Limit)
  // Goal Velocity    (4 Bytes)     Default = 131 (Via experiment)
  // Profile Acc      (4 bytes)     Default = 0
  // Profile Vel      (4 Bytes)     Default = 0
  // Goal Position    (4 Bytes)     Default = 0
  static constexpr int NUM_DWORDS = 5;
  static constexpr int16_t PWM_LIMIT = 885;
  static constexpr int16_t CURRENT_LIMIT = 1193;
  // The Goal PWM and Goal Current share the 4 bytes, with Goal PWM on the low
  // bits and Goal Current on the high bits.
  static constexpr int32_t DEFAULT_PWM_CURRENT =
      (CURRENT_LIMIT << 16) | PWM_LIMIT;
  static constexpr int32_t DEFAULT_VELOCITY = 131;
  // The unit that converts torque in N·m to the integer current representation
  // used in Dyanmixel.
  static constexpr float NEWTON_METER_TO_INT_CURRENT = CURRENT_LIMIT * 0.666667;
  // The unit that converts torque in N·m to the integer pwm representation used
  // in Dyanmixel.
  static constexpr float NEWTON_METER_TO_INT_PWM = PWM_LIMIT * 0.5;

  CommandBuffer(int num_joints) : buffer_(num_joints * NUM_DWORDS, 0) {
    // Intialize the Goal PWM and Current. We need to initialize them because
    // even in other Operation Mode such as position control, those values are
    // used as limits.
    for (size_t i = 0; i < buffer_.size(); i += NUM_DWORDS) {
      buffer_[i] = DEFAULT_PWM_CURRENT;
      buffer_[i + 1] = DEFAULT_VELOCITY;
    }

    buffer_ptr_ = buffer_.data();
  }

  inline void WritePosition(int32_t int_pos,
                            int32_t moving_time_ms,
                            int32_t acc_time_ms) {
    buffer_ptr_[2] = acc_time_ms;
    buffer_ptr_[3] = moving_time_ms;
    buffer_ptr_[4] = int_pos;
    buffer_ptr_ += NUM_DWORDS;
  }

  inline void WriteCurrent(float tau) {
    int16_t int_current =
        static_cast<int16_t>(std::round(NEWTON_METER_TO_INT_CURRENT * tau));
    buffer_ptr_[0] = (int_current << 16) | PWM_LIMIT;
    buffer_ptr_ += NUM_DWORDS;
  }

  inline void WritePWM(float tau) {
    int16_t int_pwm =
        static_cast<int16_t>(std::round(NEWTON_METER_TO_INT_PWM * tau));
    // We need to put the 2 bytes (signed) integer PWM in the position of the
    // lower 2 bytes at buffer's [0]. This is how we do it.
    uint16_t unsigned_pwm = static_cast<uint16_t>(int_pwm);
    buffer_ptr_[0] = 0 | unsigned_pwm;
    buffer_ptr_ += NUM_DWORDS;
  }

  inline void WriteVelocity(int32_t int_velocity) {
    buffer_ptr_[2] = int_velocity;
    buffer_ptr_ += NUM_DWORDS;
  }

  inline int32_t *data() {
    // Returns the content of the buffer. Useful when writing to the motors.
    return buffer_.data();
  }

 private:
  std::vector<int32_t> buffer_{};
  // Each time a Write* method is called, the buffer_ptr_ will increment by the
  // number of DWORDs per motor, so that the next Write* method invocation will
  // write to the area corresponding to the next motor.
  int32_t *buffer_ptr_ = nullptr;
};

}  // namespace

void WxArmorDriver::SendCommand(const std::vector<float> &targets,
                                float moving_time,
                                float acc_time) {
  assert(moving_time / 2 >= acc_time &&
         "Acceleration time cannot be more than half the moving time.");

  int32_t moving_time_ms = static_cast<int32_t>(moving_time * 1000.0);
  int32_t acc_time_ms = static_cast<int32_t>(acc_time * 1000.0);
  const uint8_t num_joints = static_cast<uint8_t>(profile_.joint_ids.size());
  CommandBuffer buffer{num_joints};
  const float *raw_target = targets.data();

  for (const MotorInfo *motor : profile_.joint_motors) {
    switch (motor->op_mode) {
      case OpMode::POSITION:
      case OpMode::CURRENT_BASED_POSITION:
        buffer.WritePosition(
            dxl_wb_.convertRadian2Value(motor->id, *raw_target),
            moving_time_ms,
            acc_time_ms);
        break;
      case OpMode::CURRENT:
        buffer.WriteCurrent(*raw_target);
        break;
      case OpMode::PWM:
        buffer.WritePWM(*raw_target);
        break;
      case OpMode::VELOCITY:
        buffer.WriteVelocity(
            dxl_wb_.convertVelocity2Value(motor->id, *raw_target));
        break;
      default:
        spdlog::critical("Operation Mode {} is not supported",
                         OpModeName(motor->op_mode));
    }
    ++raw_target;
  }

  const char *log = nullptr;
  std::unique_lock<std::mutex> lock{io_mutex_};

  // NOTE: The number of data for each motor (= 1) in this call to syncWrite()
  // means that each motor will take one int32_t value from int_command.data().
  bool success = dxl_wb_.syncWrite(
      write_handler_index_,
      profile_.joint_ids.data(),
      num_joints,
      buffer.data(),
      CommandBuffer::NUM_DWORDS, /* number of words (4 bytes) per motor */
      &log);

  lock.unlock();
  if (!success) {
    spdlog::error("Cannot write position command: {}", log);
  }
}

void WxArmorDriver::TorqueOn() {
  std::lock_guard<std::mutex> lock{io_mutex_};
  for (const MotorInfo &motor : profile_.motors) {
    dxl_wb_.torqueOn(motor.id);
  }
}

void WxArmorDriver::TorqueOff() {
  std::lock_guard<std::mutex> lock{io_mutex_};
  for (const MotorInfo &motor : profile_.motors) {
    dxl_wb_.torqueOff(motor.id);
  }
}

void WxArmorDriver::SetPID(const std::vector<PIDGain> &gain_cfgs) {
  std::lock_guard<std::mutex> lock{io_mutex_};

  auto SetPIDSingleMotor =
      [this](const MotorInfo &motor, int32_t p, int32_t i, int32_t d) -> void {
    if (motor.op_mode != OpMode::POSITION &&
        motor.op_mode != OpMode::CURRENT_BASED_POSITION) {
      spdlog::warn(
          "Motor '{}' is currently in {} control mode, and will ignore PID "
          "setting.",
          motor.name,
          OpModeName(motor.op_mode));
      return;
    }
    const char *log;
    if (!dxl_wb_.itemWrite(motor.id, "Position_P_Gain", p, &log)) {
      spdlog::error("Failed To set P Gain of motor {}: {}", motor.id, log);
    }
    if (!dxl_wb_.itemWrite(motor.id, "Position_I_Gain", i, &log)) {
      spdlog::error("Failed To set I Gain of motor {}: {}", motor.id, log);
    }
    if (!dxl_wb_.itemWrite(motor.id, "Position_D_Gain", d, &log)) {
      spdlog::error("Failed To set D Gain of motor {}: {}", motor.id, log);
    }
    spdlog::info(
        "Successfully set PID = ({}, {}, {}) for motor {}", p, i, d, motor.id);
  };

  for (const PIDGain &gain_cfg : gain_cfgs) {
    if (gain_cfg.name == "all") {
      for (const MotorInfo &motor : profile_.motors) {
        SetPIDSingleMotor(motor, gain_cfg.p, gain_cfg.i, gain_cfg.d);
      }
    } else if (const MotorInfo *motor = profile_.motor(gain_cfg.name)) {
      SetPIDSingleMotor(*motor, gain_cfg.p, gain_cfg.i, gain_cfg.d);
    } else {
      spdlog::error("Cannot set PID: motor '{}' not found", gain_cfg.name);
    }
  }
}

void WxArmorDriver::UsePWMControlOnGripper() {
  if (!profile_.UpdateMotorByName("gripper", [this](MotorInfo *motor) {
        const char *log;
        if (!dxl_wb_.setPWMControlMode(motor->id, &log)) {
          spdlog::error("Cannot set gripper control to PWM control: {}", log);
        } else {
          motor->op_mode = OpMode::PWM;
          spdlog::info("Gripper is now updated to use PWM control");
        }
      })) {
    spdlog::error("Cannot find gripper motor.");
  }
}

bool WxArmorDriver::SafetyViolationTriggered() {
  return safety_violation_.load();
}

void WxArmorDriver::TriggerSafetyViolationMode() { safety_violation_ = true; }

void WxArmorDriver::ResetSafetyViolationMode() { safety_violation_ = false; }

ControlItem WxArmorDriver::AddItemToRead(const std::string &name) {
  // Here we assume that the data allocation on all the motors are
  // identical. Therefore, we can just read the address of the motor ID and
  // call it a day.
  const ControlItem *address =
      dxl_wb_.getItemInfo(profile_.joint_ids.front(), name.c_str());

  if (address == nullptr) {
    spdlog::critical("Cannot find onboard item '{}' to read.", name);
    std::abort();
  } else {
    spdlog::info("Register '{}' reader at (address = {}, length = {})",
                 name,
                 address->address,
                 address->data_length);
  }

  read_start_ = std::min(read_start_, address->address);
  read_end_ =
      std::max(read_end_,
               static_cast<uint16_t>(address->address + address->data_length));

  return *address;
}

void WxArmorDriver::InitReadHandler() {
  read_position_address_ = AddItemToRead("Present_Position");
  read_velocity_address_ = AddItemToRead("Present_Velocity");
  read_current_address_ = AddItemToRead("Present_Current");

  read_handler_index_ = dxl_wb_.getTheNumberOfSyncReadHandler();
  if (!dxl_wb_.addSyncReadHandler(read_start_, read_end_ - read_start_)) {
    spdlog::critical("Failed to add sync read handler.");
    std::abort();
  } else {
    spdlog::info("Registered sync read handler (address = {}, length = {})",
                 read_start_,
                 read_end_ - read_start_);
  }
}

void WxArmorDriver::InitWriteHandler() {
  const char *prev_item_name = nullptr;

  auto ExtendWriteAddressRange = [this, &prev_item_name](const char *name) {
    const ControlItem *item =
        dxl_wb_.getItemInfo(profile_.joint_ids.front(), name);

    if (prev_item_name == nullptr) {
      write_address_.address = item->address;
      write_address_.data_length = item->data_length;
    } else {
      // Otherwise, extend the current write address (range). However we will
      // first verify that the next item is an immediate follower.
      if (write_address_.address + write_address_.data_length !=
          item->address) {
        spdlog::critical(
            "Expecting the address of '{}' to immediately follow "
            "the address of '{}'. However, getting {} instead of {}",
            name,
            prev_item_name,
            item->address,
            write_address_.address + write_address_.data_length);
        std::abort();
      }
      write_address_.data_length += item->data_length;
    }
    prev_item_name = name;
  };

  // First retrieve all the addresses of interest, and ensure that they form a
  // contiguous address range and follow the expected order. The resulting write
  // address range will be stored in `write_address_`.
  ExtendWriteAddressRange("Goal_PWM");
  ExtendWriteAddressRange("Goal_Current");
  ExtendWriteAddressRange("Goal_Velocity");
  ExtendWriteAddressRange("Profile_Acceleration");
  ExtendWriteAddressRange("Profile_Velocity");
  ExtendWriteAddressRange("Goal_Position");

  // Next, creates the write handler.
  write_handler_index_ = dxl_wb_.getTheNumberOfSyncWriteHandler();

  const char *log = nullptr;
  if (!dxl_wb_.addSyncWriteHandler(
          write_address_.address, write_address_.data_length, &log)) {
    spdlog::critical("Failed to create the sync writer: {}", log);
    std::abort();
  }

  spdlog::info("Registered sync writer handler (address = {}, length = {})",
               write_address_.address,
               write_address_.data_length);
}

}  // namespace horizon::wx_armor
