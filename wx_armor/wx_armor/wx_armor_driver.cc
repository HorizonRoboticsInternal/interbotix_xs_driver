#include "wx_armor/wx_armor_driver.h"

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <thread>

#include "spdlog/spdlog.h"

namespace fs = std::filesystem;

namespace horizon::wx_armor
{
namespace
{

static constexpr uint32_t DEFAULT_BAUDRATE = 1'000'000;

// Load the configuration file, which should be in YAML format. Upon failure,
// crash the program.
auto LoadConfigOrDie(fs::path config_path) -> YAML::Node {
    try {
        YAML::Node node = YAML::LoadFile(config_path.c_str());
        if (node.IsNull()) {
            spdlog::critical("Failed to read config file {} as it is empty.", config_path.string());
            std::abort();
        }
        return node;
    }
    catch (YAML::BadFile& error) {
        spdlog::critical("Failed to load the config file {}, due to {}", config_path.string(),
                         error.what());
        std::abort();
    }
}

// The driver is a long running service. There are cases that when the driver
// starts running (either because of power on or restarted from previous
// failure) the USB cable connecting the arm (U2D2) and the runnig host (usually
// a Raspberry Pi) isn't plugged in yet. In this case the driver will be waiting
// for the port to get ready (i.e. for the cable to be plugged in).
void WaitUntilPortAvailable(DynamixelWorkbench* dxl_wb, const std::string usb_port) {
    spdlog::info("Still waiting for USB port {} to be available ...", usb_port);
    while (true) {
        // Note that for WidowX 250s, if we do not use the default Baudrate
        // 1000000, the communication between the driver and the robotic arm
        // WILL NOT work.
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
auto PingMotors(DynamixelWorkbench* dxl_wb, const RobotProfile& profile,
                bool log_errors_only = false, int num_trials = 3,
                std::chrono::milliseconds sleep_between_trials = std::chrono::milliseconds(200))
    -> bool {
    const char* log;

    std::set<uint8_t> success{};

    for (int i = 0; i < num_trials; ++i) {
        for (const MotorInfo& motor : profile.motors) {
            if (success.count(motor.id) > 0)
                continue;
            if (dxl_wb->ping(motor.id, &log)) {
                success.emplace(motor.id);
                if (log_errors_only) {
                    continue;
                }
                std::string model_name = dxl_wb->getModelName(motor.id);
                spdlog::info("Found DYNAMIXEL Motor ID: {}, Model: {}, Name: {}", motor.id,
                             model_name, motor.name);
                if (model_name == "XL-320") {
                    spdlog::warn("Model XL-320's current reading is not supported by "
                                 "this "
                                 "driver, because its effort is load (%) based.");
                }
            }
            else {
                spdlog::error("FAILED to ping Motor ID: {}, Name: {}", motor.id, motor.name);
                break;
            }
        }
        if (success.size() == profile.motors.size()) {
            return true;
        }
        else if (i + 1 < num_trials) {
            if (!log_errors_only) {
                spdlog::warn("Found only {} / {} motors. Will wait for {} ms and retry.",
                             success.size(), profile.motors.size(), sleep_between_trials.count());
            }
            std::this_thread::sleep_for(sleep_between_trials);
        }
    }
    return false;
}

void FlashEEPROM(DynamixelWorkbench* dxl_wb, const RobotProfile& profile) {
    const char* log;

    size_t num_failed = 0;
    for (const RegistryKV& kv : profile.eeprom) {
        int32_t current_value = 0;

        // Hacky fix for now... Probably a better way to handle this
        // Safety_Velocity_Limit is our own custom param. Don't write it to
        // EEPROM
        if (kv.key == "Safety_Velocity_Limit" || kv.key == "Current_Limit") {
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
            spdlog::error("Failed to flash EEPROM for key value pair ({}, {}) on motor "
                          "ID = "
                          "{}: {}",
                          kv.key, kv.value, kv.motor_id, log);
            ++num_failed;
        }
        else {
            spdlog::info("Value '{}' of Motor {} is set to {}.", kv.key, kv.motor_id, kv.value);
        }
    }

    if (num_failed > 0) {
        spdlog::critical("Failed to flash all registry key value pairs to EEPROM (failure "
                         "{} / {}).",
                         num_failed, profile.eeprom.size());
        std::abort();
    }
    else {
        spdlog::info("Successfully flashed all registry key value pairs to EEPROM.");
    }
}

// Calibrate the shadow motors so that their 0 position is identical to the 0
// position of their corresponding master motor. This is done by setting the
// homing offset of the shadow motors.
void CalibrateShadowOrDie(DynamixelWorkbench* dxl_wb, const RobotProfile& profile) {
    for (const MotorInfo& motor : profile.motors) {
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
                spdlog::critical("Failed to reset homing offset of motor {} to 0.", shadow_id);
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
                spdlog::critical("Failed to write homing offset of motor {} during shadow "
                                 "motor "
                                 "calibration",
                                 shadow_id);
                std::abort();
            }
            else {
                spdlog::info("Successfully calibrated motor {} and {} with homing "
                             "offset = {}",
                             motor.id, shadow_id, homing_offset);
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
void SetCurrentLimit(DynamixelWorkbench* dxl_wb, RobotProfile* profile) {
    const char* log = nullptr;

    for (MotorInfo& motor : profile->motors) {
        std::string model_name = dxl_wb->getModelName(motor.id);
        uint32_t current_limit = motor.current_limit;

        // If the current limit (effectively torque limit) is set to a non-zero
        // value, we should use current-based position controller as the
        // operation mode, so that we can set current limit to the motor.
        //
        // There is one exception. If the motor is an XL series motor, it does
        // support current-based position controller operation mode. In this
        // case, we are still going to use the default position controller
        // operation mode.
        if (current_limit == 0 || model_name.substr(0, 2) == "XL") {
            motor.op_mode = OpMode::POSITION;
            if (!dxl_wb->setPositionControlMode(motor.id, &log)) {
                spdlog::critical("Failed to set OpMode to POSITION on motor {} (id = {}): "
                                 "{}",
                                 motor.name, motor.id, log);
                std::abort();
            }
        }
        else {
            motor.op_mode = OpMode::CURRENT_BASED_POSITION;
            if (!dxl_wb->currentBasedPositionMode(motor.id, current_limit, &log)) {
                spdlog::critical("Failed to set OpMode to POSITION on motor {} (id = {}): "
                                 "{}",
                                 motor.name, motor.id, log);
                std::abort();
            }
        }
        spdlog::info("Successfully set OpMode to {} on motor {} (id = {})",
                     OpModeName(motor.op_mode), motor.name, motor.id);
    }
}

// Check all the motors and see whether there are motors in error state. If so,
// that motor is rebooted. This function is called at initialization.
void RebootMotorIfInErrorState(DynamixelWorkbench* dxl_wb, const RobotProfile& profile) {
    const char* log;
    int32_t value = 0;
    for (const MotorInfo& motor : profile.motors) {
        bool success = dxl_wb->itemRead(motor.id, "Hardware_Error_Status", &value, &log);

        if (success && value == 0) {
            continue;
        }
        else if (dxl_wb->reboot(motor.id, &log)) {
            spdlog::info("Motor {} '{}' was in error state, and is rebooted.", motor.id,
                         motor.name);
        }
        else {
            spdlog::critical("Motor {} '{}' was in error state, but fail to reboot it: {}",
                             motor.id, motor.name, log);
            std::abort();
        }
    }
}

}  // namespace

WxArmorDriver::WxArmorDriver(const std::string& usb_port, fs::path motor_config_path,
                             bool flash_eeprom)
    : profile_(LoadConfigOrDie(motor_config_path).as<RobotProfile>()) {
    WaitUntilPortAvailable(&dxl_wb_, usb_port);

    if (dxl_wb_.getProtocolVersion() != 2.0) {
        spdlog::critical("Requires protocol 2.0, but got {:.1f}", dxl_wb_.getProtocolVersion());
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
    while (!PingMotors(&dxl_wb_, profile_)) {
        spdlog::critical("Could not find all specified motors.");
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    // Torque off so that we can write EEPROM, calibrate shadow motors and so
    // on.
    TorqueOff();

    RebootMotorIfInErrorState(&dxl_wb_, profile_);
    if (flash_eeprom) {
        // Note that FlashEEPROM performs "write-on-diff", meaning that it will
        // not write if the desired value and current value are the same. This
        // can effectively extend the lifespan of the EEPROM because it has
        // limited number of writes.
        FlashEEPROM(&dxl_wb_, profile_);
    }
    CalibrateShadowOrDie(&dxl_wb_, profile_);
    SetCurrentLimit(&dxl_wb_, &profile_);

    // Now torque back on.
    TorqueOn();

    InitReadHandler();
    InitWriteHandler();
}

WxArmorDriver::~WxArmorDriver() {
}

// bool WxArmorDriver::MotorHealthCheck() {
//     std::unique_lock<std::mutex> handler_lock{io_mutex_};
//     return PingMotors(&dxl_wb_, profile_,
//                       /* log_errors_only */ true,
//                       /* num_trials */ 1);
// }
bool WxArmorDriver::MotorHealthCheck() {
    std::unique_lock<std::mutex> handler_lock{io_mutex_};
    for (const MotorInfo& motor : profile_.motors) {
        int32_t curr_motor_error = 0;

        if (!dxl_wb_.itemRead(motor.id, "Hardware_Error_Status", &curr_motor_error)) {
            spdlog::warn("Motor {} '{}' could not be read.", motor.id, motor.name);
            return false;
        }

        // Ifor motor has error
        if (curr_motor_error != 0) {
            spdlog::warn("Motor {} '{}' has error status: {}", motor.id, motor.name,
                         curr_motor_error);
            return false;
        }
    }
    return true;
}

std::optional<SensorData> WxArmorDriver::Read() {
    static uint64_t error_count = 0;
    std::vector<int32_t> buffer(profile_.joint_ids.size());
    const uint8_t num_joints = static_cast<uint8_t>(buffer.size());

    SensorData result = SensorData{
        .pos = std::vector<float>(num_joints),
        .vel = std::vector<float>(num_joints),
        .crt = std::vector<float>(num_joints),
        .err = std::vector<uint32_t>(num_joints),
    };

    std::unique_lock<std::mutex> handler_lock{io_mutex_};
    const char* log;

    if (!dxl_wb_.syncRead(read_handler_index_, profile_.joint_ids.data(), num_joints, &log)) {
        if (error_count % 1000 == 0) {
            spdlog::warn("Failed to syncRead: {}", log);
        }
        ++error_count;
        return std::nullopt;
    }
    else {
        error_count = 0;
    }

    // We use the time here as the timestamp for the latest reading. This is,
    // however, an approximation. It won't be much off because the syncRead
    // above typically takes 2ms to complete.
    result.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();

    // 1. Extract Position

    if (!dxl_wb_.getSyncReadData(read_handler_index_, profile_.joint_ids.data(), num_joints,
                                 read_position_address_.address, read_position_address_.data_length,
                                 buffer.data(), &log)) {
        spdlog::critical("Cannot getSyncReadData (position): {}", log);
        std::abort();
    }

    for (size_t i = 0; i < profile_.joint_ids.size(); ++i) {
        result.pos[i] = dxl_wb_.convertValue2Radian(profile_.joint_ids[i], buffer[i]);
    }

    // 2. Extract Velocity

    if (!dxl_wb_.getSyncReadData(read_handler_index_, profile_.joint_ids.data(), num_joints,
                                 read_velocity_address_.address, read_velocity_address_.data_length,
                                 buffer.data(), &log)) {
        spdlog::critical("Cannot getSyncReadData (velocity): {}", log);
        std::abort();
    }

    for (size_t i = 0; i < profile_.joint_ids.size(); ++i) {
        result.vel[i] = dxl_wb_.convertValue2Velocity(profile_.joint_ids[i], buffer[i]);
    }

    // 3. Extract Current

    if (!dxl_wb_.getSyncReadData(read_handler_index_, profile_.joint_ids.data(), num_joints,
                                 read_current_address_.address, read_current_address_.data_length,
                                 buffer.data(), &log)) {
        spdlog::critical("Cannot getSyncReadData (current): {}", log);
        std::abort();
    }

    for (size_t i = 0; i < profile_.joint_ids.size(); ++i) {
        result.crt[i] = dxl_wb_.convertValue2Current(profile_.joint_ids[i], buffer[i]);
    }

    return std::move(result);
}

std::vector<float> WxArmorDriver::GetSafetyVelocityLimits() {
    std::vector<float> safety_velocity_limits;

    int counter = 0;
    for (const auto& motor : profile_.motors) {
        // Skip the shadowed motors, which have IDs 3 and 5.
        if (counter != 3 && counter != 5) {
            safety_velocity_limits.push_back(motor.safety_vel_limit);
        }
        counter++;
    }
    return safety_velocity_limits;
}

std::vector<float> WxArmorDriver::GetSafetyCurrentLimits() {
    std::vector<float> safety_current_limits;

    for (const auto& motor : profile_.motors) {
        float limit = motor.current_limit;
        if (limit < 1) {
            limit = 1200;
        }
        safety_current_limits.push_back(limit);
    }
    return safety_current_limits;
}

void WxArmorDriver::SetPosition(const std::vector<float>& position, float moving_time,
                                float acc_time) {
    assert(moving_time / 2 >= acc_time &&
           "Acceleration time cannot be more than half the moving time.");

    int32_t moving_time_ms = static_cast<int32_t>(moving_time * 1000.0);
    int32_t acc_time_ms = static_cast<int32_t>(acc_time * 1000.0);
    const uint8_t num_joints = static_cast<uint8_t>(profile_.joint_ids.size());
    std::vector<int32_t> int_command(num_joints * 3, 0);
    size_t j = 0;
    for (size_t i = 0; i < profile_.joint_ids.size(); ++i) {
        // Profile acceleration
        int_command[j++] = acc_time_ms;
        // Profile velocity
        int_command[j++] = moving_time_ms;
        // Goal Position
        int_command[j++] = dxl_wb_.convertRadian2Value(profile_.joint_ids[i], position.at(i));
    }

    const char* log = nullptr;
    std::unique_lock<std::mutex> lock{io_mutex_};

    // NOTE: The number of data for each motor (= 1) in this call to syncWrite()
    // means that each motor will take one int32_t value from
    // int_command.data().
    bool success =
        dxl_wb_.syncWrite(write_position_and_profile_handler_index_, profile_.joint_ids.data(),
                          num_joints, int_command.data(), 3, /* number of data for each motor */
                          &log);
    lock.unlock();
    if (!success) {
        spdlog::error("Cannot write position command: {}", log);
    }
}

void WxArmorDriver::TorqueOn() {
    std::lock_guard<std::mutex> lock{io_mutex_};
    for (const MotorInfo& motor : profile_.motors) {
        dxl_wb_.torqueOn(motor.id);
    }
}

void WxArmorDriver::TorqueOff() {
    std::lock_guard<std::mutex> lock{io_mutex_};
    for (const MotorInfo& motor : profile_.motors) {
        dxl_wb_.torqueOff(motor.id);
    }
}

void WxArmorDriver::SetPID(const std::vector<PIDGain>& gain_cfgs) {
    std::lock_guard<std::mutex> lock{io_mutex_};

    auto SetPIDSingleMotor = [this](uint8_t motor_id, int32_t p, int32_t i, int32_t d) -> void {
        const char* log;
        if (!dxl_wb_.itemWrite(motor_id, "Position_P_Gain", p, &log)) {
            spdlog::error("Failed To set P Gain of motor {}: {}", motor_id, log);
        }
        if (!dxl_wb_.itemWrite(motor_id, "Position_I_Gain", i, &log)) {
            spdlog::error("Failed To set I Gain of motor {}: {}", motor_id, log);
        }
        if (!dxl_wb_.itemWrite(motor_id, "Position_D_Gain", d, &log)) {
            spdlog::error("Failed To set D Gain of motor {}: {}", motor_id, log);
        }
        spdlog::info("Successfully set PID = ({}, {}, {}) for motor {}", p, i, d, motor_id);
    };

    for (const PIDGain& gain_cfg : gain_cfgs) {
        if (gain_cfg.name == "all") {
            for (const MotorInfo& motor : profile_.motors) {
                SetPIDSingleMotor(motor.id, gain_cfg.p, gain_cfg.i, gain_cfg.d);
            }
        }
        else if (const MotorInfo* motor = profile_.motor(gain_cfg.name)) {
            SetPIDSingleMotor(motor->id, gain_cfg.p, gain_cfg.i, gain_cfg.d);
        }
        else {
            spdlog::error("Cannot set PID: motor '{}' not found", gain_cfg.name);
        }
    }
}

bool WxArmorDriver::SafetyViolationTriggered() {
    return safety_violation_.load();
}

void WxArmorDriver::TriggerSafetyViolationMode() {
    safety_violation_ = true;
}

void WxArmorDriver::ResetSafetyViolationMode() {
    safety_violation_ = false;
}

ControlItem WxArmorDriver::AddItemToRead(const std::string& name) {
    // Here we assume that the data allocation on all the motors are
    // identical. Therefore, we can just read the address of the motor ID and
    // call it a day.
    const ControlItem* address = dxl_wb_.getItemInfo(profile_.joint_ids.front(), name.c_str());

    if (address == nullptr) {
        spdlog::critical("Cannot find onboard item '{}' to read.", name);
        std::abort();
    }
    else {
        spdlog::info("Register '{}' reader at (address = {}, length = {})", name, address->address,
                     address->data_length);
    }

    read_start_ = std::min(read_start_, address->address);
    read_end_ = std::max(read_end_, static_cast<uint16_t>(address->address + address->data_length));

    return *address;
}

void WxArmorDriver::InitReadHandler() {
    read_position_address_ = AddItemToRead("Present_Position");
    read_velocity_address_ = AddItemToRead("Present_Velocity");
    read_current_address_ = AddItemToRead("Present_Current");
    read_error_address_ = AddItemToRead("Hardware_Error_Status");

    read_handler_index_ = dxl_wb_.getTheNumberOfSyncReadHandler();
    if (!dxl_wb_.addSyncReadHandler(read_start_, read_end_ - read_start_)) {
        spdlog::critical("Failed to add sync read handler.");
        std::abort();
    }
    else {
        spdlog::info("Registered sync read handler (address = {}, length = {})", read_start_,
                     read_end_ - read_start_);
    }
}

void WxArmorDriver::InitWriteHandler() {
    static constexpr char GOAL_POSITION[] = "Goal_Position";
    static constexpr char PROFILE_ACC[] = "Profile_Acceleration";
    static constexpr char PROFILE_VEL[] = "Profile_Velocity";

    OpMode op_mode = profile_.motors.front().op_mode;
    if (op_mode != OpMode::POSITION && op_mode != OpMode::CURRENT_BASED_POSITION) {
        spdlog::critical("WxArmorDriver only support Operation Mode 'POSITION' and "
                         "'CURRENT_BASED_POSITION'.");
        std::abort();
    }

    // Add the write handler that writes only the goal position
    const ControlItem* goal_position_address =
        dxl_wb_.getItemInfo(profile_.joint_ids.front(), GOAL_POSITION);
    if (goal_position_address == nullptr) {
        spdlog::critical("Cannot find onboard item '{}' to write.", GOAL_POSITION);
        std::abort();
    }

    write_position_address_ = *goal_position_address;

    write_position_handler_index_ = dxl_wb_.getTheNumberOfSyncWriteHandler();
    if (!dxl_wb_.addSyncWriteHandler(write_position_address_.address,
                                     write_position_address_.data_length)) {
        spdlog::critical("Failed to add sync write handler for {}", GOAL_POSITION);
        std::abort();
    }
    else {
        spdlog::info("Registered sync write handler for {} (address = {}, length = {})",
                     GOAL_POSITION, write_position_address_.address,
                     write_position_address_.data_length);
    }

    // Add the write handler that writes the goal position, as well as
    // the velocity and acceleration profile.
    const ControlItem* profile_vel_address =
        dxl_wb_.getItemInfo(profile_.joint_ids.front(), PROFILE_VEL);
    const ControlItem* profile_acc_address =
        dxl_wb_.getItemInfo(profile_.joint_ids.front(), PROFILE_ACC);
    if (profile_acc_address->address + profile_acc_address->data_length !=
        profile_vel_address->address) {
        spdlog::critical("The register address of profile acceleration is not "
                         "next to that of profile velocity");
        std::abort();
    }
    if (profile_vel_address->address + profile_vel_address->data_length !=
        goal_position_address->address) {
        spdlog::critical("The register address of profile velocity is not "
                         "next to that of goal position");
        std::abort();
    }

    write_position_and_profile_handler_index_ = dxl_wb_.getTheNumberOfSyncWriteHandler();

    uint16_t start = profile_acc_address->address;
    uint16_t length = profile_acc_address->data_length + profile_vel_address->data_length +
                      goal_position_address->data_length;

    if (!dxl_wb_.addSyncWriteHandler(start, length)) {
        spdlog::critical("Failed to add sync write handler for {} + {} + {}", PROFILE_VEL,
                         PROFILE_ACC, GOAL_POSITION);
        std::abort();
    }
    else {
        spdlog::info("Registered sync write handler for {} + {} + {} (address = {}, "
                     "length "
                     "= {})",
                     PROFILE_VEL, PROFILE_ACC, GOAL_POSITION, start, length);
    }
}

WxArmorDriver* Driver() {
    static std::unique_ptr<WxArmorDriver> driver = []() {
        std::string usb_port = GetEnv<std::string>("WX_ARMOR_USB_PORT", "/dev/ttyDXL");
        std::filesystem::path motor_config = GetEnv<std::filesystem::path>(
            "WX_ARMOR_MOTOR_CONFIG",
            std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "configs" /
                "wx250s_motor_config.yaml");
        int flash_eeprom = true;
        return std::make_unique<WxArmorDriver>(usb_port, motor_config,
                                               static_cast<bool>(flash_eeprom));
    }();
    return driver.get();
}

void SlowDownToStop(const SensorData& curr_reading, float dt, float deceleration_time) {
    std::vector<float> curr_pos = curr_reading.pos;
    std::vector<float> curr_vel = curr_reading.vel;
    std::vector<float> targets;

    for (size_t i = 0; i < curr_pos.size(); i++) {
        // Just some simple kinematic integration to minimize acceleration
        // changes
        float curr_target = curr_pos[i] + curr_vel[i] * dt;
        targets.push_back(curr_target);
    }

    // Overwrite the current trajectory with our decelerating one.
    Driver()->SetPosition(targets, deceleration_time, 0.49 * deceleration_time);
    // SetPID to zero Kp, large Kd and zero Ki to drop to the ground from
    // current pos.
    Driver()->SetPID({{"all", 0, 0, 80000}});
}

}  // namespace horizon::wx_armor
