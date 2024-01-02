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
  size_t num_failed = 0;
  for (const RegistryKV &kv : profile.eeprom) {
    int32_t current_value = 0;
    dxl_wb->itemRead(kv.motor_id, kv.key.c_str(), &current_value);

    // The EEPROM on the motors has a limited number of writes during its
    // lifespan. Only write when there is a discrepancy between the intended
    // value and the current value.
    if (current_value == kv.value) {
      continue;
    }

    if (!dxl_wb->itemWrite(kv.motor_id, kv.key.c_str(), kv.value)) {
      spdlog::error(
          "Failed to flash EEPROM for key value pair ({}, {}) on motor ID = {}",
          kv.key,
          kv.value,
          kv.motor_id);
      ++num_failed;
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
      int32_t homing_offset = ((shadow_drive_mode & 1) == 0)
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

}  // namespace

WxArmorDriver::WxArmorDriver(const std::string &usb_port,
                             fs::path motor_config_path,
                             bool flash_eeprom)
    : profile_(LoadConfigOrDie(motor_config_path).as<RobotProfile>()) {
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

  // The EEPROM on the motors has a lifespan about 100,000 write
  // cycles. However, reading from eeprom does not affect it lifespan
  //
  // TODO(breakds): We should only write to it if we find discrepancies, i.e.
  // write-on-change.
  if (flash_eeprom) {
    FlashEEPROM(&dxl_wb_, profile_);
  }

  CalibrateShadowOrDie(&dxl_wb_, profile_);

  InitReadHandler();
  InitWriteHandler();

  for (int i = 0; i < 10; ++i) {
    spdlog::info("Start Read.");
    FetchSensorData();
    spdlog::info("Positions: {}, {}, {}, {}, {}, {}, {}",
                 latest_reading_.pos[0],
                 latest_reading_.pos[1],
                 latest_reading_.pos[2],
                 latest_reading_.pos[3],
                 latest_reading_.pos[4],
                 latest_reading_.pos[5],
                 latest_reading_.pos[6]);
  }
}

// TODO(breakds): Support fetching position only, and see whether it will be
// faster.
void WxArmorDriver::FetchSensorData() {
  std::vector<int32_t> buffer(profile_.joint_ids.size());
  const uint8_t num_joints = static_cast<uint8_t>(buffer.size());

  std::unique_lock<std::mutex> handler_lock{read_handler_mutex_};
  const char *log;

  if (!dxl_wb_.syncRead(
          read_handler_index_, profile_.joint_ids.data(), num_joints, &log)) {
    spdlog::critical("Failed to syncRead: {}", log);
    std::abort();
  }

  // No body can hold latest_reading_lock forever, meaning there is no risk of
  // dead lock here.
  std::lock_guard<std::mutex> latest_reading_lock{latest_reading_mutex_};

  // We use the time here as the timestamp for the latest reading. This is,
  // however, an approximation. It won't be much off because the syncRead above
  // typically takes 2ms to complete.
  latest_reading_.timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(
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
    // TODO(breakds): Roll out our own version using double.
    latest_reading_.pos[i] = static_cast<double>(
        dxl_wb_.convertValue2Radian(profile_.joint_ids[i], buffer[i]));
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
    // TODO(breakds): Roll out our own version using double.
    latest_reading_.vel[i] = static_cast<double>(
        dxl_wb_.convertValue2Velocity(profile_.joint_ids[i], buffer[i]));
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
    // TODO(breakds): Roll out our own version using double.
    latest_reading_.crt[i] = static_cast<double>(
        dxl_wb_.convertValue2Current(profile_.joint_ids[i], buffer[i]));
  }
}

nlohmann::json WxArmorDriver::SensorDataToJson() const {
  std::lock_guard<std::mutex> lock(latest_reading_mutex_);
  return nlohmann::json(latest_reading_);
}

void WxArmorDriver::SetPosition(const std::vector<double> &position) {
  std::vector<int32_t> int_command(profile_.joint_ids.size(), 0);
  for (size_t i = 0; i < profile_.joint_ids.size(); ++i) {
    int_command[i] = dxl_wb_.convertRadian2Value(
        profile_.joint_ids[i], static_cast<float>(position.at(i)));
  }

  const char *log = nullptr;
  const uint8_t num_joints = static_cast<uint8_t>(profile_.joint_ids.size());
  if (!dxl_wb_.syncWrite(write_position_handler_index_,
                         profile_.joint_ids.data(),
                         num_joints,
                         int_command.data(),
                         1, /* TODO(breakds) what is it? */
                         &log)) {
    spdlog::error("Cannot write position command: {}", log);
  }
}

void WxArmorDriver::StartLoop() {
  read_loop_thread_ = std::jthread([this]() {
    while (true) {
      FetchSensorData();
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
  });
}

ControlItem WxArmorDriver::AddItemToRead(const std::string &name) {
  // Here we assume that the data allocation on all of the motors are identical.
  // Therefore, we can just read the address of the motor ID and call it a day.
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

  latest_reading_.pos.resize(profile_.joint_ids.size(), 0.0);
  latest_reading_.vel.resize(profile_.joint_ids.size(), 0.0);
  latest_reading_.crt.resize(profile_.joint_ids.size(), 0.0);
}

void WxArmorDriver::InitWriteHandler() {
  static constexpr char GOAL_POSITION[] = "Goal_Position";

  // TODO(breakds): Initialize write handler for other operation mode as well.
  if (profile_.motors.front().op_mode != OpMode::POSITION) {
    spdlog::critical("WxArmorDriver not implemented for OpMode != POSITON.");
    std::abort();
  }

  const ControlItem *address =
      dxl_wb_.getItemInfo(profile_.joint_ids.front(), GOAL_POSITION);
  if (address == nullptr) {
    spdlog::critical("Cannot find onboard item '{}' to write.", GOAL_POSITION);
    std::abort();
  }

  write_position_address_ = *address;

  write_position_handler_index_ = dxl_wb_.getTheNumberOfSyncWriteHandler();
  if (!dxl_wb_.addSyncWriteHandler(write_position_address_.address,
                                   write_position_address_.data_length)) {
    spdlog::critical("Failed to add sync write handler for {}", GOAL_POSITION);
    std::abort();
  } else {
    spdlog::info(
        "Registered sync write handler for {} (address = {}, length = {})",
        GOAL_POSITION,
        write_position_address_.address,
        write_position_address_.data_length);
  }
}

}  // namespace horizon::wx_armor
