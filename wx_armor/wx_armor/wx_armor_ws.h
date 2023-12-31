#pragma once

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <type_traits>

#include "drogon/HttpAppFramework.h"
#include "drogon/PubSubService.h"
#include "drogon/WebSocketController.h"
#include "wx_armor/wx_armor_driver.h"

namespace horizon::wx_armor {

WxArmorDriver *Driver();

void StartDriverLoop();

struct ClientState {
  bool engaging = false;
  std::chrono::time_point<std::chrono::system_clock> latest_healthy_time{};
};

class WxArmorWebController
    : public drogon::WebSocketController<WxArmorWebController> {
 public:
  virtual void handleNewMessage(const drogon::WebSocketConnectionPtr &,
                                std::string &&,
                                const drogon::WebSocketMessageType &) override;

  virtual void handleConnectionClosed(
      const drogon::WebSocketConnectionPtr &) override;

  virtual void handleNewConnection(
      const drogon::HttpRequestPtr &,
      const drogon::WebSocketConnectionPtr &) override;

  WS_PATH_LIST_BEGIN
  WS_PATH_ADD("/api/engage", drogon::Get);
  WS_PATH_LIST_END

 private:
  static constexpr char CMD_SETPOS[] = "SETPOS";
  static constexpr char CMD_READ[] = "READ";
};

template <typename T>
T GetEnv(const char *name, T default_value) {
  const char *text = std::getenv(name);

  if (text == nullptr) {
    return default_value;
  }

  if constexpr (std::is_same_v<T, std::string>) {
    return std::string(text);
  } else if constexpr (std::is_integral_v<T>) {
    return static_cast<T>(std::stoi(text));
  } else if constexpr (std::is_same_v<T, std::filesystem::path>) {
    return std::filesystem::path(text);
  }

  std::abort();
}

}  // namespace horizon::wx_armor
