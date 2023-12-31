#include <filesystem>

#include "spdlog/spdlog.h"
#include "wx_armor/wx_armor_ws.h"

using horizon::wx_armor::GetEnv;
using horizon::wx_armor::StartDriverLoop;
using horizon::wx_armor::WxArmorDriver;

int main(int argc, char** argv) {
  drogon::app()
      .addListener("0.0.0.0", GetEnv<int>("WX_ARMOR_WS_PORT", 8027))
      .setClientMaxWebSocketMessageSize(1 * 1024 * 1024 /* 1MB */)
      .run();

  return 0;
}
