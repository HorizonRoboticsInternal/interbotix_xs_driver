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

namespace horizon::wx_armor
{

class GuardianThread;
class MotorHealthThread;

// This is for bookkeeping purpose. We maintain ClientState for each of the
// client that talks to the server.
struct ClientState
{
    // Upon connection establishment, engaging is `false`. It is switched to
    // `true` as soon as the first `SETPOS` command is received.
    bool engaging = false;

    // Record the time of the last SETPOS command. This can be used to check
    // whether the client stops publishing command for too long.
    std::chrono::time_point<std::chrono::system_clock> latest_healthy_time{};
};

class WxArmorWebController : public drogon::WebSocketController<WxArmorWebController>
{
  public:
    WxArmorWebController();

    // Callback when a new message is received.
    virtual void handleNewMessage(const drogon::WebSocketConnectionPtr&, std::string&&,
                                  const drogon::WebSocketMessageType&) override;

    // Callback upon closing of a connection.
    virtual void handleConnectionClosed(const drogon::WebSocketConnectionPtr&) override;

    // Callback upon a new connection is established.
    virtual void handleNewConnection(const drogon::HttpRequestPtr&,
                                     const drogon::WebSocketConnectionPtr&) override;

    void CheckAndSetPosition(const std::vector<float>& cmd, float moving_time);

    // Listens on the `"/api/engage"` path for websocket connection.
    WS_PATH_LIST_BEGIN
    WS_PATH_ADD("/api/engage", drogon::Get);
    WS_PATH_LIST_END

  private:
    /**
     * This class uses the following two threads to execute safety logic.
     *
     *   - `GuardianThread`: This thread is responsible for setting fresh sensor
     *         data in a shared cache as well as executing safety violation logic.
     *   - `MotorHealthThread`: This thread is responsible for continuously checking
     *         the health of the motors via pinging. If the motors cannot be pinged,
     *         it will communicate this to the GuardianThread accordingly.
     */
    std::shared_ptr<GuardianThread> guardian_thread_;
    std::shared_ptr<MotorHealthThread> motor_health_thread_;
};

}  // namespace horizon::wx_armor
