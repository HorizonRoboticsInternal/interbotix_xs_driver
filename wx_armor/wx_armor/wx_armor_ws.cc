#include "wx_armor/wx_armor_ws.h"

#include <memory>

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
    return std::make_unique<WxArmorDriver>(usb_port, motor_config);
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
      nlohmann::json json = nlohmann::json::parse(message);
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