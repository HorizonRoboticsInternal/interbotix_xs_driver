#include "wx_armor/robot_profile.h"

#include "spdlog/spdlog.h"

namespace YAML
{

bool convert<horizon::wx_armor::RobotProfile>::decode(const Node& node,
                                                      horizon::wx_armor::RobotProfile& profile) {
    using namespace horizon::wx_armor;

    // Fill in model information.
    for (const auto& child : node["motors"]) {
        YAML::Node info = child.second;
        uint8_t motor_id = info["ID"].as<uint8_t>();
        profile.motor_ids.emplace_back(motor_id);

        // Safety velocity limit is given in [deg/s]. Convert it to [rad/s]
        float safety_vel_limit = info["Safety_Velocity_Limit"].as<float>();
        safety_vel_limit *= M_PI / 180.;

        uint32_t current_limit = info["Current_Limit"].as<uint32_t>();

        OpMode op_mode = (current_limit == 0) ? OpMode::POSITION : OpMode::CURRENT_BASED_POSITION;

        profile.motors.emplace_back(
            MotorInfo{.id = motor_id,
                      .name = child.first.as<std::string>(),
                      // By default, set the operation mode to position control.
                      .op_mode = op_mode,
                      .current_limit = current_limit,
                      .safety_vel_limit = safety_vel_limit});

        // Now, populate the register table.
        for (const auto& kv : info) {
            std::string key = kv.first.as<std::string>();
            // Ignore "ID" and "Baud_Rate" as they are not valid register
            // names on the motor's internal EEPROM register table.
            if (key == "ID" || key == "Baud_Rate")
                continue;
            profile.eeprom.emplace_back(motor_id, key, kv.second.as<int32_t>());
        }
    }

    // Fill in joint IDs.
    for (const auto& child : node["joint_order"]) {
        std::string name = child.as<std::string>();
        const MotorInfo* motor = profile.motor(name);
        if (motor == nullptr) {
            spdlog::critical("Malformed motor config. Cannot find a motor named '{}', which "
                             "appears in 'joint_order'",
                             name);
        }
        profile.joint_ids.emplace_back(motor->id);
        profile.joint_names.emplace_back(motor->name);
    }

    // Handle shadows info. Such info is used to calibrate the homing offsets of
    // the motors.

    for (const auto& child : node["shadows"]) {
        // Shadow motors that do not need homing offset calibration are skipped.
        if (!child.second["calibrate"].as<bool>()) {
            continue;
        }

        std::string master_motor_name = child.first.as<std::string>();
        MotorInfo* motor = const_cast<MotorInfo*>(profile.motor(master_motor_name));

        if (motor == nullptr) {
            spdlog::critical("Malformed motor config. Cannot find a motor named '{}', which "
                             "appears in 'shadows'",
                             master_motor_name);
        }

        for (const auto& grandchild : child.second["shadow_list"]) {
            std::string shadow_motor_name = grandchild.as<std::string>();
            const MotorInfo* shadow_motor = profile.motor(shadow_motor_name);
            if (motor == nullptr) {
                spdlog::critical("Malformed motor config. Cannot find a motor named '{}', "
                                 "which "
                                 "appears in 'shadows'",
                                 shadow_motor_name);
            }
            motor->shadow_motor_ids.emplace_back(shadow_motor->id);
        }
    }

    return true;
}

}  // namespace YAML

namespace horizon::wx_armor
{

std::string_view OpModeName(OpMode mode) {
    switch (mode) {
        case OpMode::CURRENT:
            return "CURRENT";
        case OpMode::VELOCITY:
            return "VELOCITY";
        case OpMode::POSITION:
            return "POSITION";
        case OpMode::CURRENT_BASED_POSITION:
            return "CURRENT_BASED_POSITION";
        case OpMode::PWM:
            return "PWM";
        case OpMode::TORQUE:
            return "TORQUE";
    }
    return "UNKNOWN";
}

}  // namespace horizon::wx_armor
