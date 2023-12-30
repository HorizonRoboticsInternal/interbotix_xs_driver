#include "wx_armor/wx_armor_ws.h"

#include "spdlog/spdlog.h"

using drogon::HttpRequestPtr;
using drogon::WebSocketConnectionPtr;
using drogon::WebSocketMessageType;

namespace horizon::wx_armor {
void WxArmorWebController::handleNewMessage(const WebSocketConnectionPtr &conn,
                                            std::string &&message,
                                            const WebSocketMessageType &type) {
  if (type == WebSocketMessageType::Text) {
    spdlog::info("Received! Message = {}", message);
  }
}

void WxArmorWebController::handleConnectionClosed(
    const WebSocketConnectionPtr &conn) {
  spdlog::info("Closed!");
}

void WxArmorWebController::handleNewConnection(
    const HttpRequestPtr &req, const WebSocketConnectionPtr &conn) {
  spdlog::info("Connected!");
  conn->send("ok");
}

}  // namespace horizon::wx_armor
