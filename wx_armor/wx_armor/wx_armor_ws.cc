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

namespace horizon::wx_armor {

WxArmorDriver *Driver() {
  static std::unique_ptr<WxArmorDriver> driver = []() {
    std::string usb_port =
        GetEnv<std::string>("WX_ARMOR_USB_PORT", "/dev/ttyDXL");
    std::filesystem::path motor_config = GetEnv<std::filesystem::path>(
        "WX_ARMOR_MOTOR_CONFIG",
        std::filesystem::path(__FILE__)
                .parent_path()
                .parent_path()
                .parent_path() /
            "configs" / "wx250s_motor_config.yaml");
    int flash_eeprom = true;
    return std::make_unique<WxArmorDriver>(
        usb_port, motor_config, static_cast<bool>(flash_eeprom));
  }();
  return driver.get();
}

void WxArmorWebController::handleNewMessage(const WebSocketConnectionPtr &conn,
                                            std::string &&message,
                                            const WebSocketMessageType &type) {
  std::string_view payload{};

  if (Driver()->SafetyViolationTriggered()) {
    spdlog::error(
        "Call from message handler ignored. Safety violation was encountered.");
    return;
  }

  // Valid message should be in the format of "<COMMAND> <PAYLOAD>". This helper
  // function try to match the message with predefined command, and if there is
  // a match, put the payload into `payload`.
  auto Match = [&message, &payload](const char *command) -> bool {
    size_t command_length = strlen(command);
    if (std::strncmp(message.data(), command, command_length) == 0) {
      payload = std::string_view(message.data() + command_length + 1,
                                 message.size() - command_length - 1);
      return true;
    }
    return false;
  };

  if (type == WebSocketMessageType::Text) {
    if (Match("SETPOS")) {
      // Update the states for bookkeeping purpose.
      ClientState &state = conn->getContextRef<ClientState>();
      state.engaging = true;
      state.latest_healthy_time = std::chrono::system_clock::now();

      // Relay the command to the driver.
      nlohmann::json json = nlohmann::json::parse(payload);
      std::vector<float> position(json.size());
      for (size_t i = 0; i < json.size(); ++i) {
        position[i] = json.at(i).get<float>();
      }
      CheckAndSetPosition(position, 0.0);
    } else if (Match("MOVETO")) {
      // Update the states for bookkeeping purpose.
      ClientState &state = conn->getContextRef<ClientState>();
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
    } else if (Match("TORQUE ON")) {
      Driver()->TorqueOn();
    } else if (Match("TORQUE OFF")) {
      Driver()->TorqueOff();
    } else if (Match("SETPID")) {
      std::vector<PIDGain> gain_cfgs = nlohmann::json::parse(payload);
      Driver()->SetPID(gain_cfgs);
    }
  }
}

void WxArmorWebController::handleConnectionClosed(
    const WebSocketConnectionPtr &conn) {
  guardian_thread_.Unsubscribe(conn);
  spdlog::info("A connection is closed.");
}

void WxArmorWebController::handleNewConnection(
    const HttpRequestPtr &req, const WebSocketConnectionPtr &conn) {
  conn->setContext(std::make_shared<ClientState>());
  guardian_thread_.Subscribe(conn);
  spdlog::info("A new connection is established.");
  conn->send("ok");
}

static constexpr int MAX_TOLERABLE_CONSECUTIVE_NUM_READ_ERRORS = 6;

void SlowDownToStop(const SensorData &curr_reading,
                    float dt = 0.5,
                    float deceleration_time = 1.5) {
  std::vector<float> curr_pos = curr_reading.pos;
  std::vector<float> curr_vel = curr_reading.vel;
  std::vector<float> targets;

  for (size_t i = 0; i < curr_pos.size(); i++) {
    // Just some simple kinematic integration to minimize acceleration changes
    float curr_target = curr_pos[i] + curr_vel[i] * dt;
    targets.push_back(curr_target);
  }

  // Overwrite the current trajectory with our decelerating one.
  Driver()->SetPosition(targets, deceleration_time, 0.49 * deceleration_time);
}

void WxArmorWebController::CheckAndSetPosition(const std::vector<float> &cmd,
                                               float moving_time) {
  const SensorData readings = guardian_thread_.GetCachedSensorData();
  for (int i = 0; i + 1 < cmd.size(); i++) {
    // Ignore last position for the grippers
    float reading = readings.pos.at(i);
    // 0.2 is a generous action delta for pid control
    float thd = 0.2;
    if (moving_time > 0.1)
      thd = 2.5 * moving_time;
    if (fabs(reading - cmd[i]) > thd) {
      spdlog::error(
          "Joint {} command is out of range: {} -> {} > {}. Command ignored.",
          i,
          reading,
          cmd[i],
          thd);
      Driver()->TriggerSafetyViolationMode();
      SlowDownToStop(readings);
      guardian_thread_.KillConnections();
      return;
    }
  }
  Driver()->SetPosition(cmd, moving_time);
}

WxArmorWebController::GuardianThread::GuardianThread() {
  thread_ = std::jthread([this]() {
    std::vector<float> safety_velocity_limits =
        Driver()->GetSafetyVelocityLimits();
    bool logged = false;
    while (!shutdown_.load()) {
      // Read the sensor data
      std::optional<SensorData> sensor_data = Driver()->Read();

      // Publish the sensor data
      if (sensor_data.has_value()) {
        {
          std::unique_lock<std::mutex> cache_lock{cache_mutex_};
          sensor_data_cache_ = SensorData{.pos = sensor_data.value().pos,
                                          .vel = sensor_data.value().vel,
                                          .crt = sensor_data.value().crt};
        }
        num_consecutive_read_errors_ = 0;
        std::string message = nlohmann::json(sensor_data.value()).dump();
        std::lock_guard<std::mutex> lock{conns_mutex_};
        for (const WebSocketConnectionPtr &conn : conns_) {
          conn->send(message);
        }

        // Don't check for a new safety violation if state hasn't been reset yet
        if (Driver()->SafetyViolationTriggered()) {
          if (!logged)
            spdlog::error(
                "Guardian thread safety violation checking ignored. Safety "
                "violation is already triggered.");
            logged = true;
          continue;
        }

        // GuardianThread also has the dual role of monitoring for safety
        // e.g., checking for velocity limit violations
        std::vector<float> curr_velocities = sensor_data.value().vel;
        for (int i = 0; i < curr_velocities.size(); i++) {
          float cv = fabs(curr_velocities[i]);
          float limit = safety_velocity_limits[i];
          if (cv > limit) {
            Driver()->TriggerSafetyViolationMode();
            break;
          }
        }

        // If a safety violation is triggered, first slow down to a stop
        // and then kill all client connections.
        // This way, user will have to reset on client-side, but the server
        // will keep running.
        if (Driver()->SafetyViolationTriggered()) {
          SlowDownToStop(sensor_data.value());
          KillConnections();
        }

      } else {
        // When fail to read, accumulate the counter, check for threshold and
        // crash if threshold is exceeded.
        ++num_consecutive_read_errors_;
        if (num_consecutive_read_errors_ >
            MAX_TOLERABLE_CONSECUTIVE_NUM_READ_ERRORS) {
          spdlog::critical(
              "Encounter {} consecutive read errors, which is considered too "
              "many. Forcefully shutting down.",
              num_consecutive_read_errors_);
          std::abort();
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
}

const SensorData WxArmorWebController::GuardianThread::GetCachedSensorData() {
  std::unique_lock<std::mutex> cache_lock{cache_mutex_};
  return sensor_data_cache_;
}

WxArmorWebController::GuardianThread::~GuardianThread() {
  shutdown_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void WxArmorWebController::GuardianThread::Subscribe(
    const WebSocketConnectionPtr &conn) {
  // Clear all safety violations only if there are no previous connections
  if (conns_.empty()) {
    Driver()->ResetSafetyViolationMode();
  }
  std::lock_guard<std::mutex> lock{conns_mutex_};
  conns_.emplace_back(conn);
}

void WxArmorWebController::GuardianThread::Unsubscribe(
    const WebSocketConnectionPtr &conn) {
  std::lock_guard<std::mutex> lock{conns_mutex_};
  conns_.erase(std::remove(conns_.begin(), conns_.end(), conn), conns_.end());
}

void WxArmorWebController::GuardianThread::KillConnections() {
  for (auto &conn : conns_) {
    // TODO(andrew): log which joint was responsible?
    conn->shutdown(drogon::CloseCode::kViolation,
                   "Shutting down due to safety violation.");
  }
  conns_.clear();
}

}  // namespace horizon::wx_armor
