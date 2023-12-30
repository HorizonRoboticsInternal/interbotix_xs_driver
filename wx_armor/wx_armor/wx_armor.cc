#include <filesystem>

#include "spdlog/spdlog.h"
#include "wx_armor/wx_armor_driver.h"
#include "wx_armor/wx_armor_ws.h"

using horizon::wx_armor::WxArmorDriver;

int main(int argc, char** argv) {
  // std::filesystem::path configs_path = std::filesystem::path(
  //     "/home/breakds/projects/interbotix_xs_driver/configs/");
  // WxArmorDriver driver("/dev/ttyDXL",
  //                      configs_path / "wx250s_motor_config.yaml");

  drogon::app()
      .addListener("0.0.0.0", 8027)
      .setClientMaxWebSocketMessageSize(1 * 1024 * 1024 /* 1MB */)
      .run();

  return 0;
}
