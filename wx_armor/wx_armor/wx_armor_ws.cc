#include "wx_armor/wx_armor_ws.h"

#include <memory>

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
        // TODO(breakds): An saner default probably.
        std::filesystem::path("/home/breakds/projects/interbotix_xs_driver/"
                              "configs/wx250s_motor_config.yaml"));
    return std::make_unique<WxArmorDriver>(usb_port, motor_config);
  }();
  return driver.get();
}

void InitDriverLoop() {
  // This will start the internal reading loop thread.
  Driver()->StartLoop();
}

void WxArmorWebController::handleNewMessage(const WebSocketConnectionPtr &conn,
                                            std::string &&message,
                                            const WebSocketMessageType &type) {
  if (type == WebSocketMessageType::Text) {
    if (std::strncmp(message.data(), CMD_READ, 4) == 0) {
      spdlog::info("READ");
    } else if (std::strncmp(message.data(), CMD_SETPOS, 6) == 0) {
      spdlog::info("SETPOS");
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
