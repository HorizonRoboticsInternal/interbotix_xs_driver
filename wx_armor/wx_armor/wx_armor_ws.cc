#include "wx_armor/wx_armor_ws.h"

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

    bool set_pid = Match("SETPID");
    bool move_to = Match("MOVETO");
    bool reboot = Match("REBOOT");

    if (type != WebSocketMessageType::Text) {
        // This happens during keepalive or closing connection, ignored
    }
    else if (set_pid && !Driver()->Read().has_value()) {
        // Allow client to reset error status via SETPID only if driver can read
        // motors
        spdlog::error("SETPID ignored.  Cannot read");
    }
    else if (!set_pid && !move_to && !reboot && Driver()->SafetyViolationTriggered()) {
        // If safety violation is triggered, ignore all commands except
        // 1) SETPID which resets error status,
        // 2) MOVETO which allows the client to set the jointpos command to the
        //    current position to avoid jumps after SETPID, and
        // 3) REBOOT which tells the driver to reboot faulted motors.
        spdlog::error("Call from message handler ignored. Safety violation was "
                      "encountered.");
    }
    else if (move_to) {
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
    else if (set_pid) {
        // Make sure to set the position to the current one in case
        // we are resetting PID after a hard reset to avoid large jumps.
        std::optional<SensorData> sensor_data = Driver()->Read();
        if (sensor_data.has_value()) {
            CheckAndSetPosition(sensor_data.value().pos, 0.0);
        }
        std::vector<PIDGain> gain_cfgs = nlohmann::json::parse(payload);
        Driver()->SetPID(gain_cfgs);
        guardian_thread_.ResetErrorCodes();
    }
    else if (reboot) {
        Driver()->RebootMotors();
    }
    else if (Match("TORQUE ON")) {
        Driver()->TorqueOn();
    }
    else if (Match("TORQUE OFF")) {
        Driver()->TorqueOff();
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
            // Make sure to set is_joint_idx to True since velocities are reported
            // at the joint level.
            guardian_thread_.SetErrorCode(i, GuardianThread::kErrorCommandDeltaTooLarge, true);
            return;
        }
    }
    Driver()->SetPosition(cmd, moving_time);
}

}  // namespace horizon::wx_armor
