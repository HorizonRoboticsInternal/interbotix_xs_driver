#pragma once

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>

#include "drogon/HttpAppFramework.h"
#include "drogon/PubSubService.h"
#include "drogon/WebSocketController.h"
#include "wx_armor/wx_armor_driver.h"

namespace horizon::wx_armor {

WxArmorDriver *Driver();

// This is for bookkeeping purpose. We maintain ClientState for each of the
// client that talks to the server.
struct ClientState {
  // Upon connection establishment, engaging is `false`. It is switched to
  // `true` as soon as the first `SETPOS` command is received.
  bool engaging = false;

  // Record the time of the last SETPOS command. This can be used to check
  // whether the client stops publishing command for too long.
  std::chrono::time_point<std::chrono::system_clock> latest_healthy_time{};
};

class WxArmorWebController
    : public drogon::WebSocketController<WxArmorWebController> {
 public:
  // Callback when a new message is received.
  virtual void handleNewMessage(const drogon::WebSocketConnectionPtr &,
                                std::string &&,
                                const drogon::WebSocketMessageType &) override;

  // Callback upon closing of a connection.
  virtual void handleConnectionClosed(
      const drogon::WebSocketConnectionPtr &) override;

  // Callback upon a new connection is established.
  virtual void handleNewConnection(
      const drogon::HttpRequestPtr &,
      const drogon::WebSocketConnectionPtr &) override;

  // Listens on the `"/api/engage"` path for websocket connection.
  WS_PATH_LIST_BEGIN
  WS_PATH_ADD("/api/engage", drogon::Get);
  WS_PATH_LIST_END

 private:
  // The publisher is responsible for continuously (by running a background
  // thread) read the sensor data and publish it to the clients that subscribe
  // the sensor data.
  class Publisher {
   public:
    Publisher();
    ~Publisher();

    // Add the client (identified by the connection) to the subscribption list.
    void Subscribe(const drogon::WebSocketConnectionPtr &conn);

    // Remove the client (identified by the connection) to the subscribption
    // list.
    void Unsubscribe(const drogon::WebSocketConnectionPtr &conn);

   private:
    std::mutex conns_mutex_;  // protects conns_
    std::vector<drogon::WebSocketConnectionPtr> conns_{};

    std::atomic_bool shutdown_{false};
    std::jthread thread_{};
  };

  Publisher publisher_;
};

// Helper function to read the environment variable.
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
