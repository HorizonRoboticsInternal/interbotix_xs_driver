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
  if (type == WebSocketMessageType::Text) {
    if (std::strncmp(message.data(), CMD_READ, 4) == 0) {
      nlohmann::json reading = Driver()->SensorDataToJson();
      conn->send(reading.dump());
    } else if (std::strncmp(message.data(), CMD_SETPOS, 6) == 0) {
      std::string payload = message.substr(7);
      nlohmann::json json = nlohmann::json::parse(payload);
      std::vector<double> position(json.size());
      for (size_t i = 0; i < json.size(); ++i) {
        position[i] = json.at(i).get<double>();
      }
      Driver()->SetPosition(position);
    }
  }
}

void WxArmorWebController::handleConnectionClosed(
    const WebSocketConnectionPtr &conn) {
  spdlog::info("Closed!");
}

void WxArmorWebController::handleNewConnection(
    const HttpRequestPtr &req, const WebSocketConnectionPtr &conn) {
  conn->setContext(std::make_shared<ClientState>());
  spdlog::info("Connected");
  conn->send("ok");
}

}  // namespace horizon::wx_armor
