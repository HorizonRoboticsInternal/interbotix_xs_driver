#include <filesystem>

#include "spdlog/spdlog.h"
#include "wx_armor/wx_armor_ws.h"

using horizon::wx_armor::Driver;
using horizon::wx_armor::GetEnv;
using horizon::wx_armor::WxArmorWebController;

/* This is the entry point of the long running daemon.
 *
 * It basically starts the websocket server (see `wx_armor_ws.h`) so that it can
 * talk to the clients via websocket.
 *
 * Under the hood, the websocket server handles the messages and relay the
 * queries and requests to the underlying driver (see `wx_armor_driver.h`).
 *
 * The architecture is as simple as illustrated below.
 *
 *  ┌────────┐               ┌───────────┐       ┌─────────┐
 *  │ Client │ ⇦ WiFi/ETH ⇨  │ WS Server │  ⇦ ⇨  │ Driver  │
 *  └────────┘               └───────────┘       └─────────┘
 *                                                    ⇧
 *                                                   USB
 *                                                    ⇩
 *                                               ┌─────────┐
 *                                               │  U2D2   │
 *                                               └────┬────┘
 *                                                    │
 *                                                    │
 *                                                    │
 *                                             ┌──────┼──────┐
 *                                             │  Dynamixel  │
 *                                             │   Motors    │
 *                                             └─────────────┘
 */

int main(int argc, char** argv) {
    Driver();  // Ensure driver is up and running.
    drogon::app()
        .addListener("0.0.0.0", GetEnv<int>("WX_ARMOR_WS_PORT", 8027))
        .setClientMaxWebSocketMessageSize(1 * 1024 * 1024 /* 1MB */)
        .setThreadNum(1)
        .run();

    return 0;
}
