#include "wx_armor/wx_armor_ws.h"

#include <cstring>
#include <memory>
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
    int flash_eeprom = GetEnv<int>("WX_ARMOR_FLASH_EEPROM", 0);
    return std::make_unique<WxArmorDriver>(
        usb_port, motor_config, static_cast<bool>(flash_eeprom));
  }();
  return driver.get();
}

void StartDriverLoop() {
  // This will start the internal reading loop thread.
  Driver()->StartLoop();
}

void WxArmorWebController::handleNewMessage(const WebSocketConnectionPtr &conn,
                                            std::string &&message,
                                            const WebSocketMessageType &type) {
  std::string_view payload{};

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
    if (Match("READ")) {
      nlohmann::json reading = Driver()->SensorDataToJson();
      conn->send(reading.dump());
    } else if (Match("SETPOS")) {
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
      Driver()->SetPosition(position);
    } else if (Match("TORQUE ON")) {
      Driver()->TorqueOn();
    } else if (Match("TORQUE OFF")) {
      Driver()->TorqueOff();
    }
  }
}

void WxArmorWebController::handleConnectionClosed(
    const WebSocketConnectionPtr &conn) {
  spdlog::info("A connection is closed.");
}

void WxArmorWebController::handleNewConnection(
    const HttpRequestPtr &req, const WebSocketConnectionPtr &conn) {
  conn->setContext(std::make_shared<ClientState>());
  spdlog::info("A new connection is established.");
  conn->send("ok");
}

}  // namespace horizon::wx_armor
