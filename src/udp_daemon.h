#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "boost/asio.hpp"
#include "protocols.h"
#include "interbotix_xs_driver/xs_driver.hpp"

using InterbotixDriverXS = interbotix_xs::InterbotixDriverXS;

namespace horizon {
namespace widowx {

class UDPDaemon {
 public:
  using udp = boost::asio::ip::udp;

  static constexpr char CMD_STATUS[] = "STATUS";
  static constexpr char CMD_BOUNDS[] = "BOUNDS";
  static constexpr char CMD_SETPOS[] = "SETPOS";
  static constexpr char CMD_LISTEN[] = "LISTEN";

  explicit UDPDaemon(int port, bool sync=false);

  void Start();

  // This does not actually connect to the Sagittarius robotic arm and
  // is useful for testing UDP networking.
  void StartMock();

 private:
  // ---------- Actual Command Handlers ----------

  // Command: STATUS
  ArmStatus GetStatus();

  // Command: SETPOS
  void SetPosition(const std::vector<float>& positions);

  // Command: BOUNDS
  std::vector<PositionBound> GetBounds();

  int port_;
  std::unique_ptr<boost::asio::io_service> io_service_{};
  std::unique_ptr<udp::socket> socket_{};

  // Low level arm interface.
  std::unique_ptr<InterbotixDriverXS> arm_low_;
  bool sync_mode_ = false;
};

}  // namespace widowx
}  // namespace horizon
