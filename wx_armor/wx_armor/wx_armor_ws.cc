#include "wx_armor/wx_armor_ws.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string_view>

#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

using drogon::HttpRequestPtr;
using drogon::WebSocketConnectionPtr;
using drogon::WebSocketMessageType;

namespace horizon::wx_armor
{

void WxArmorWebController::handleNewMessage(const WebSocketConnectionPtr& conn,
                                            std::string&& message,
                                            const WebSocketMessageType& type) {
    spdlog::info("{}", message);
    auto s = std::chrono::high_resolution_clock::now();

    std::string_view payload{};

    // Valid message should be in the format of "<COMMAND> <PAYLOAD>". This
    // helper function try to match the message with predefined command, and if
    // there is a match, put the payload into `payload`.
    auto Match = [&message, &payload](const char* command) -> bool {
        size_t command_length = strlen(command);
        if (std::strncmp(message.data(), command, command_length) == 0) {
            payload = std::string_view(message.data() + command_length + 1,
                                       message.size() - command_length - 1);
            return true;
        }
        return false;
    };

    if (type != WebSocketMessageType::Text) {
        // This happens during keepalive or closing connection, ignored
    }
    else if (Match("SETPID") && !Driver()->Read().has_value()) {
        // Allow client to reset error status via SETPID only if driver can read
        // motors
        spdlog::error("SETPID ignored.  Cannot read");
    }
    else if (!Match("SETPID") && !Match("MOVETO") && Driver()->SafetyViolationTriggered()) {
        // If safety violation is triggered, ignore all commands except SETPID
        // which resets error status, and MOVETO which allows the client to set
        // the jointpos command to the current position to avoid jumps after
        // SETPID.
        spdlog::error("Call from message handler ignored. Safety violation was "
                      "encountered.");
    }
    else if (Match("SETPOS")) {
        // Update the states for bookkeeping purpose.
        ClientState& state = conn->getContextRef<ClientState>();
        state.engaging = true;
        state.latest_healthy_time = std::chrono::system_clock::now();

        // Relay the command to the driver.
        nlohmann::json json = nlohmann::json::parse(payload);
        std::vector<float> position(json.size());
        for (size_t i = 0; i < json.size(); ++i) {
            position[i] = json.at(i).get<float>();
        }
        CheckAndSetPosition(position, 0.0);
    }
    else if (Match("MOVETO")) {
        // Update the states for bookkeeping purpose.
        ClientState& state = conn->getContextRef<ClientState>();
        state.engaging = true;
        state.latest_healthy_time = std::chrono::system_clock::now();

        // Relay the command to the driver. Note that the last numbers
        // in the list is the moving time, in seconds.
        nlohmann::json json = nlohmann::json::parse(payload);
        std::vector<float> position(json.size() - 1);
        for (size_t i = 0; i < json.size() - 1; ++i) {
            position[i] = json.at(i).get<float>();
        }
        float moving_time = json.at(json.size() - 1).get<float>();
        CheckAndSetPosition(position, moving_time);
    }
    else if (Match("TORQUE ON")) {
        Driver()->TorqueOn();
    }
    else if (Match("TORQUE OFF")) {
        Driver()->TorqueOff();
    }
    else if (Match("SETPID")) {
        std::vector<PIDGain> gain_cfgs = nlohmann::json::parse(payload);
        Driver()->SetPID(gain_cfgs);
        guardian_thread_.ResetErrorCodes();
    }

    auto e = std::chrono::high_resolution_clock::now();
    auto duration = e - s;
    auto sec = std::chrono::duration_cast<std::chrono::milliseconds>(duration);

    spdlog::info(sec.count());
}

void WxArmorWebController::handleConnectionClosed(const WebSocketConnectionPtr& conn) {
    guardian_thread_.Unsubscribe(conn);
    spdlog::info("A connection is closed.");
}

void WxArmorWebController::handleNewConnection(const HttpRequestPtr& req,
                                               const WebSocketConnectionPtr& conn) {
    conn->setContext(std::make_shared<ClientState>());
    guardian_thread_.Subscribe(conn);
    spdlog::info("A new connection is established.");
    conn->send("ok");
}

void WxArmorWebController::CheckAndSetPosition(const std::vector<float>& cmd, float moving_time) {
    const SensorData readings = guardian_thread_.GetCachedSensorData();
    for (int i = 0; i + 1 < cmd.size(); i++) {
        // Ignore last position for the grippers
        float reading = readings.pos.at(i);
        // 0.2 is a generous action delta for pid control
        float thd = 0.2;
        if (moving_time > 0.1)
            thd = 2.5 * moving_time;
        if (fabs(reading - cmd[i]) > thd && !Driver()->SafetyViolationTriggered()) {
            spdlog::error("Joint {} command is out of range: {} -> {} > {}. Command "
                          "ignored.",
                          i, reading, cmd[i], thd);
            Driver()->TriggerSafetyViolationMode();
            guardian_thread_.SetErrorCode(i, GuardianThread::kErrorCommandDeltaTooLarge);
            SlowDownToStop(readings);
            // guardian_thread_.KillConnections();
            return;
        }
    }

    Driver()->SetPosition(cmd, moving_time);

    // Check if all motors are healthy
    if (!Driver()->SafetyViolationTriggered()) {
        auto dxl_wb = Driver()->DxlWb();
        bool all_motors_healthy = true;
        {
            std::unique_lock<std::mutex> handler_lock{Driver()->IOMutex()};

            for (const auto& motor : Driver()->Profile().motors) {
                int32_t curr_motor_error = 0;

                // Set the motor index properly for shadow motors
                int motor_index = motor.id;
                if (motor_index >= 3) {
                    motor_index -= 1;
                }
                if (motor_index >= 5) {
                    motor_index -= 1;
                }

                // Check if we can read the motor
                if (!dxl_wb.itemRead(motor.id, "Hardware_Error_Status", &curr_motor_error)) {
                    guardian_thread_.SetErrorCode(motor_index,
                                                  GuardianThread::kErrorMotorNotReachable);
                    spdlog::warn("Motor {} '{}' could not be read.", motor.id, motor.name);
                    all_motors_healthy = false;
                    break;
                }

                // Check to see if there's an error for the motor
                if (curr_motor_error != 0) {
                    guardian_thread_.SetErrorCode(motor_index, curr_motor_error);
                    spdlog::warn("Motor {} '{}' has error status: {}", motor.id, motor.name,
                                 curr_motor_error);
                    all_motors_healthy = false;
                    break;
                }
            }
        }
        if (!all_motors_healthy) {
            Driver()->TriggerSafetyViolationMode();
            SlowDownToStop(readings);
        }
    }

    //    if (!Driver()->SafetyViolationTriggered() && !Driver()->MotorHealthCheck()) {
    //        Driver()->TriggerSafetyViolationMode();
    //        guardian_thread_.SetErrorCode(0, GuardianThread::kErrorMotorNotReachable);
    //        SlowDownToStop(readings);
    //        return;
    //    }
}

}  // namespace horizon::wx_armor
