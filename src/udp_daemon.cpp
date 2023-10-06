#include "udp_daemon.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>

#include "spdlog/spdlog.h"
#include "interbotix_xs_driver/xs_logging.hpp" // Logging macros and utils
#include "interbotix_xs_driver/xs_common.hpp"  // Common variables and types
#include "interbotix_xs_driver/xs_driver.hpp"  // The InterbotixDriverXS class

using boost::asio::ip::udp;

namespace horizon::widowx
{
  namespace
  {

    // Convert a string like "1.23 4.5 3.3" into a vector<float> like {1.23, 4.5, 3.3}.
    std::vector<float> ParseArray(char *text, size_t len)
    {
      std::vector<float> result{};
      char *start = text;
      for (; start - text < len; ++start)
      {
        if (!(std::isdigit(*start) || *start == '.' || *start == '-'))
          continue;
        result.emplace_back(std::strtof(start, &start));
      }
      return result;
    }

  } // namespace

  class UDPPusher
  {
   public:
    UDPPusher(InterbotixDriverXS *arm_low,
              const std::string &address,
              int port,
              bool sync = false)
        : arm_low_(arm_low), address_(address), port_(port), sync_mode_(sync),
          socket_(io_service_)
    {
      udp::resolver resolver(io_service_);
      listener_endpoint_ =
          *resolver.resolve({udp::v4(), address_, std::to_string(port_)}).begin();
      socket_.open(udp::v4());
      if (!sync_mode_)
        thread_ = std::make_unique<std::jthread>(&horizon::widowx::UDPPusher::Run, this);
    }

    ~UDPPusher()
    {
      socket_.close();
      if (thread_) thread_->request_stop();
    }

    void Run(std::stop_token stop_token)
    {
      boost::system::error_code error;
      char data[2048];

      // TODO(breakds): When the listener died it should be detected
      // here and the push thread should terminate.
      if (arm_low_ == nullptr)
      {
        // Mock case
        for (int i = 0; i < 100; ++i)
        {
          std::sprintf(data,
                        "%.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                        0.01f * i,
                        0.02f * i,
                        0.03f * i,
                        0.04f * i,
                        0.05f * i,
                        0.06f * i,
                        0.07f * i);
          int ret = socket_.send_to(
              boost::asio::buffer(std::string(data)), listener_endpoint_, 0, error);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          if (stop_token.stop_requested())
          {
            break;
          }
        }
      }
      else
      {
        // Actual case
        GetStatusAndSend(0); // initial send
        while (true)
        {
          GetStatusAndSend(2);
          if (stop_token.stop_requested())
          {
            break;
          }
        }
      }
    }

    void GetStatusAndSend(int ms = 1)
    {
      if (ms < 0)
      {
        // disable reading
        return;
      }
      boost::system::error_code error;
      char data[2048];
      const uint N_JOINTS = 8;
      std::vector<float> positions(N_JOINTS);
      std::vector<float> velocities(N_JOINTS);
      std::vector<float> efforts(N_JOINTS);
      bool succ = arm_low_->get_joint_states(
          "all", &positions, &velocities, &efforts);
      assert(succ);
      auto currentTime = std::chrono::high_resolution_clock::now();
      auto curr_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        currentTime.time_since_epoch()).count();

      std::sprintf(data,
                   "%.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f "
                   "%.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f "
                   "%.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f "
                   "%ld",
                   positions[0],
                   positions[1],
                   positions[2],
                   positions[3],
                   positions[4],
                   positions[5],
                   positions[6],
                   positions[7],
                   velocities[0],
                   velocities[1],
                   velocities[2],
                   velocities[3],
                   velocities[4],
                   velocities[5],
                   velocities[6],
                   velocities[7],
                   efforts[0],
                   efforts[1],
                   efforts[2],
                   efforts[3],
                   efforts[4],
                   efforts[5],
                   efforts[6],
                   efforts[7],
                   curr_ms);
      int ret = socket_.send_to(
          boost::asio::buffer(std::string(data)), listener_endpoint_, 0, error);
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

   private:
    // When true, write command and then read state
    bool sync_mode_ = false;
    InterbotixDriverXS *arm_low_ = nullptr;
    std::string address_;
    int port_;
    boost::asio::io_service io_service_;
    udp::endpoint listener_endpoint_;
    udp::socket socket_;
    std::unique_ptr<std::jthread> thread_;
  };

  UDPDaemon::UDPDaemon(int port,
                       bool sync,
                       int kp,
                       int dt,
                       std::string filepath_motor_configs,
                       std::string filepath_mode_configs)
      : port_(port), sync_mode_(sync), kp_(kp), dt_(dt),
        filepath_motor_configs_(filepath_motor_configs),
        filepath_mode_configs_(filepath_mode_configs) {
    if (filepath_motor_configs == "" || filepath_mode_configs == "")
    {
      char *username;
      username = getlogin();

      if (username == nullptr) {
        spdlog::error("getlogin() error");
        std::exit(EXIT_FAILURE);
      }
      std::string path = "/home/";
      path += username;
      path += "/interbotix_xs_driver";
      filepath_motor_configs_ = path + "/wx250s_motor_config.yaml";
      filepath_mode_configs_ = path + "/mode_configs.yaml";
    }
  }

  void UDPDaemon::Start()
  {
    try {
      io_service_ = std::make_unique<boost::asio::io_service>();
      socket_ =
          std::make_unique<udp::socket>(*io_service_, udp::endpoint(udp::v4(), port_));
    } catch (std::exception& e) {
      spdlog::critical("Fatal error while creating the socket: {}", e.what());
      std::exit(EXIT_FAILURE);
    }

    bool write_eeprom_on_startup = true;
    std::string logging_level = "INFO";

    arm_low_ = std::make_unique<InterbotixDriverXS>(
        filepath_motor_configs_, filepath_mode_configs_, write_eeprom_on_startup, logging_level);
    // reboot all motors for the arm to work properly:
    arm_low_->reboot_motors(interbotix_xs::cmd_type::GROUP, "all", true, false);
    arm_low_->set_motor_pid_gains(interbotix_xs::cmd_type::GROUP, "all", {kp_, 0, 1, 0, 0, 0, 0});
    // arm_low_->set_motor_pid_gains(interbotix_xs::cmd_type::SINGLE, "waist", {kp_, 0, 1, 0, 0, 0, 0});
    arm_low_->set_motor_pid_gains(interbotix_xs::cmd_type::SINGLE, "shoulder", {400, 0, 1, 0, 0, 0, 0});
    // arm_low_->set_motor_pid_gains(interbotix_xs::cmd_type::SINGLE, "elbow", {kp_, 0, 1, 0, 0, 0, 0});
    // arm_low_->set_motor_pid_gains(interbotix_xs::cmd_type::SINGLE, "forearm_roll", {kp_, 0, 1, 0, 0, 0, 0});
    // arm_low_->set_motor_pid_gains(interbotix_xs::cmd_type::SINGLE, "wrist_angle", {kp_, 0, 1, 0, 0, 0, 0});
    arm_low_->set_motor_pid_gains(interbotix_xs::cmd_type::SINGLE, "wrist_rotate", {500, 0, 0, 0, 0, 0, 0});
    // arm_low_->set_motor_pid_gains(interbotix_xs::cmd_type::SINGLE, "gripper", {kp_, 0, 0, 0, 0, 0, 0});
    spdlog::info("UDP Daemon for WidowX 250s Arm started successfully.");

    std::unique_ptr<UDPPusher> pusher;
    // The messages should be very small, so the buffer size is sufficient.
    char data[2048];
    size_t content_size = 0;

    while (true) {
      /* Control flow timeline:

      | Time (ms)  |       Client Activities      |         Server Activities          |
      |------------|------------------------------|------------------------------------|
      |     0      | Receive Robot Obs. (1ms)     | (Wait for Command)                 |
      |     1      | NN Inference (2ms)           |                                    |
      |     3      | Send Command (1ms)           | Receive Command (1ms)              |
      |     4      | Wait Control Downtime (16ms) | Wait for Effect (19ms - 3ms * n)   |
      |   ...      |                              | Read Robot State #1 (3ms)          |
      |   ...      |                              | Send to Client (1ms)               |
      |   ...      |                              | ...                                |
      |   ...      |                              | Read Robot State #n (3ms)          |
      |   ...      |                              | Send to Client (1ms)               |
      |    20      | Receive Robot Obs. (1ms)     | (Wait for Command)                 |
      |    21      | NN Inference (2ms)           |                                    |
      |    23      | Send Command (1ms)           | Receive Command (1ms)              |
      |    ...     | ...[Next Loop Activities]... | ...[Next Server Activities]...     |

      NOTE: Sending robot state to the client via udp does not block the server.
      Set command to the motors is instantaneous, and ignored from the diagram.

      The start time below can be between 0ms to 3ms, because Wait for Command can
      start anywhere from 0ms to 3ms without affecting command or obs latency.
      */
      auto start = std::chrono::high_resolution_clock::now();
      udp::endpoint sender_endpoint;
      // Below is a blocking call that returns as soon as there is data in. The
      // information about the sender is stored in the sneder_endpoint and the
      // message content will be in data.
      try {
        // This receive_from blocks if no command is sent.
        // auto start = std::chrono::high_resolution_clock::now();
        content_size = socket_->receive_from(boost::asio::buffer(data), sender_endpoint);
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> diff = end - start;
        // std::cout << "UDPDaemon: receive cmd from client took: " << diff.count() << " seconds" << std::endl;
        // TODO(breakds): Do I need to force set null ending?
      } catch (std::exception& e) {
        spdlog::error("Error in receiving from socket: {}", e.what());
        continue;
      }

      if (std::strncmp(data, CMD_STATUS, 6) == 0) {
        nlohmann::json status = GetStatus();
        socket_->send_to(boost::asio::buffer(status.dump()), sender_endpoint);
      } else if (std::strncmp(data, CMD_BOUNDS, 6) == 0) {
        nlohmann::json bounds = GetBounds();
        socket_->send_to(boost::asio::buffer(bounds.dump()), sender_endpoint);
      } else if (std::strncmp(data, CMD_SETPOS, 6) == 0) {
        std::vector<float> positions = ParseArray(data + 7, content_size - 7);
        // TODO(breakds): Check positions has 7 numbers.
        // This SetPosition is instantaneous.
        SetPosition(positions);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - start;
        // Buffer time in ms to make sure the client can get the most recent reading
        // in time for control to happen, but also w/o delaying command receiving.
        const uint buffer_ms = 12;
        // Estimated inference + send command time in ms.
        // TODO(lezh): increase send command time if control is across wifi.
        const uint est_inf_comm_ms = 3;
        // Shouldn't take more than est_inf_comm_ms to receive command.
        // Slightly relaxed by adding 3ms to it.
        if (diff.count() > 0.003 + est_inf_comm_ms * 0.001)
          spdlog::info("UDPDaemon: recv cmd & setpos overall took long: {}", diff.count());
        if (sync_mode_ && pusher) {
          // max_motor_ms is the maximium amount of time wx motor read or write can take.
          // Due to daisy chaining, the max time can be much more than 1ms.
          const uint max_motor_ms = 6;
          // Sleep before reading to make sure command set is all done,
          // otherwise conflicts arise.
          // NOTE: for agent high level control, dt_ is 100ms, so we can sleep for longer.
          auto sleep_time = dt_ - est_inf_comm_ms - 3 * max_motor_ms;
          sleep_time = std::max(sleep_time, max_motor_ms);
          std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
          bool has_enough_time_to_read = true;
          while (has_enough_time_to_read) {
            pusher->GetStatusAndSend(0);  // blocks for 0ms
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> diff = end - start;
            // We allow max_motor_ms of reading time, which can overlap with inference and command sending.
            // NOTE: dt_ - est_inf_comm_ms because start time can be anywhere from 0ms to est_inf_comm_ms.
            has_enough_time_to_read =
              dt_ - est_inf_comm_ms - diff.count() * 1000 > max_motor_ms - est_inf_comm_ms;
          }
        }
        if (sync_mode_ && !pusher)
          spdlog::error("Sync mode client should send LISTEN before SETPOS!");
      } else if (std::strncmp(data, CMD_LISTEN, 6) == 0) {
        std::string listener_address = sender_endpoint.address().to_string();
        int listener_port            = std::stoi(std::string(data + 7, content_size - 7));
        spdlog::info("Request LISTEN from {}:{}", listener_address, listener_port);
        pusher =
            std::make_unique<UDPPusher>(arm_low_.get(), listener_address, listener_port, sync_mode_);
      } else {
        spdlog::error("Invalid command: {}", data);
      }
    }
  }

  void UDPDaemon::StartMock() {
    try {
      io_service_ = std::make_unique<boost::asio::io_service>();
      socket_ =
          std::make_unique<udp::socket>(*io_service_, udp::endpoint(udp::v4(), port_));
    } catch (std::exception& e) {
      spdlog::critical("Fatal error while creating the socket: {}", e.what());
      std::exit(EXIT_FAILURE);
    }

    spdlog::info("Start mocking UDP Daemon");
    std::unique_ptr<UDPPusher> pusher;

    // The messages should be very small, so the buffer size is sufficient.
    char data[1024];
    size_t content_size = 0;

    while (true) {
      udp::endpoint sender_endpoint;
      // Below is a blocking call that returns as soon as there is data in. The
      // information about the sender is stored in the sneder_endpoint and the
      // message content will be in data.
      try {
        content_size = socket_->receive_from(boost::asio::buffer(data), sender_endpoint);
        // TODO(breakds): Do I need to force set null ending?
      } catch (std::exception& e) {
        spdlog::error("Error in receiving from socket: {}", e.what());
        continue;
      }

      if (std::strncmp(data, CMD_STATUS, 6) == 0) {
        nlohmann::json status = ArmStatus{
            .joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            .joint_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            .joint_efforts = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
        };
        socket_->send_to(boost::asio::buffer(status.dump()), sender_endpoint);
      } else if (std::strncmp(data, CMD_BOUNDS, 6) == 0) {
        nlohmann::json bounds = {
            PositionBound(-1.0, 1.0),
            PositionBound(-1.0, 1.0),
            PositionBound(-1.0, 1.0),
            PositionBound(-1.0, 1.0),
            PositionBound(-1.0, 1.0),
            PositionBound(-1.0, 1.0),
            PositionBound(-0.068, 0.0),
        };
        socket_->send_to(boost::asio::buffer(bounds.dump()), sender_endpoint);
      } else if (std::strncmp(data, CMD_SETPOS, 6) == 0) {
        std::vector<float> positions = ParseArray(data + 8, content_size - 8);
        spdlog::info("Set joint positions to [{}, {}, {}, {}, {}, {}] + {}",
                    positions[0],
                    positions[1],
                    positions[2],
                    positions[3],
                    positions[4],
                    positions[5],
                    positions[6]);
      } else if (std::strncmp(data, CMD_LISTEN, 6) == 0) {
        std::string listener_address = sender_endpoint.address().to_string();
        int listener_port            = std::stoi(std::string(data + 7, content_size - 7));
        spdlog::info("Request LISTEN from {}:{}", listener_address, listener_port);
        pusher = std::make_unique<UDPPusher>(nullptr, listener_address, listener_port);
      } else {
        spdlog::error("Invalid command: {}", data);
      }
    }
  }  // end of StartMock()

  void UDPDaemon::SetPosition(const std::vector<float>& positions) {
    if (positions.size() != 7) {
      spdlog::error("SetPosition() cannot handle inputs with a size of {}",
                    positions.size());
      return;
    }
    arm_low_->write_position_commands("all", positions);
  }

  ArmStatus UDPDaemon::GetStatus() {
    std::vector<float> positions(7);
    std::vector<float> velocities(7);
    std::vector<float> efforts(7);
    bool succ = arm_low_->get_joint_states(
        "all", &positions, &velocities, &efforts);
    assert(succ);

    return ArmStatus{
        .joint_positions =
            {positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]},
        .joint_velocities =
            {velocities[0], velocities[1], velocities[2], velocities[3], velocities[4], velocities[5], velocities[6]},
        .joint_efforts =
            {efforts[0], efforts[1], efforts[2], efforts[3], efforts[4], efforts[5], efforts[6]},
    };
  }

  std::vector<PositionBound> UDPDaemon::GetBounds() {
    return {
        PositionBound(-0.068, 0.0),
        PositionBound(-0.068, 0.0),
        PositionBound(-0.068, 0.0),
        PositionBound(-0.068, 0.0),
        PositionBound(-0.068, 0.0),
        PositionBound(-0.068, 0.0),
        PositionBound(-0.068, 0.0),
    };
  }

}  // namespace horizon::widowx
