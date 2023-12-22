#include <filesystem>

#include "spdlog/spdlog.h"
#include "wx_armor/wx_armor_driver.h"

using horizon::wx_armor::WxArmorDriver;

int main(int argc, char** argv) {
  std::filesystem::path configs_path = std::filesystem::path(
      "/home/breakds/projects/interbotix_xs_driver/configs/");
  WxArmorDriver driver("/dev/ttyDXL",
                       configs_path / "wx250s_motor_config.yaml");

  return 0;
}
