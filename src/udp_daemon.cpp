#include "udp_daemon.h"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "interbotix_xs_driver/xs_common.hpp"   // Common variables and types
#include "interbotix_xs_driver/xs_driver.hpp"   // The InterbotixDriverXS class
#include "interbotix_xs_driver/xs_logging.hpp"  // Logging macros and utils
#include "spdlog/spdlog.h"

using boost::asio::ip::udp;

namespace horizon::widowx
{
namespace
{

// Convert a string like "1.23 4.5 3.3" into a vector<float> like
// {1.23, 4.5, 3.3}.
std::vector<float> ParseArray(char* text, size_t len) {
    std::vector<float> result{};
    char* start = text;
    for (; start - text < len; ++start) {
        if (!(std::isdigit(*start) || *start == '.' || *start == '-'))
            continue;
        result.emplace_back(std::strtof(start, &start));
    }
    return result;
}

}  // namespace

class UDPPusher
{
  public:
    UDPPusher(InterbotixDriverXS* arm_low, const std::string& address, int port, bool sync = false)
        : arm_low_(arm_low), address_(address), port_(port), sync_mode_(sync), socket_(io_service_),
          thread_([this]() { Run(); }) {
    }

    ~UDPPusher() {
        kill_.store(true);
        thread_.join();
    }

    void Run() {
        udp::resolver resolver(io_service_);
        listener_endpoint_ =
            *resolver.resolve({udp::v4(), address_, std::to_string(port_)}).begin();
        socket_.open(udp::v4());

        boost::system::error_code error;
        char data[2048];

        // TODO(breakds): When the listener died it should be detected
        // here and the push thread should terminate.
        if (arm_low_ == nullptr) {
            // Mock case
            for (int i = 0; i < 100; ++i) {
                std::sprintf(data, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f", 0.01f * i, 0.02f * i,
                             0.03f * i, 0.04f * i, 0.05f * i, 0.06f * i, 0.07f * i);
                int ret = socket_.send_to(boost::asio::buffer(std::string(data)),
                                          listener_endpoint_, 0, error);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (kill_.load()) {
                    break;
                }
            }
        }
        else {
            // Actual case
            GetStatusAndSend(0);  // initial send
            while (true) {
                GetStatusAndSend(sync_mode_ ? -1 : 1);
                if (kill_.load()) {
                    break;
                }
            }
        }
        socket_.close();
    }

    void GetStatusAndSend(int ms = 1) {
        if (ms < 0) {
            // disable reading
            sleep(10);
            return;
        }
        boost::system::error_code error;
        char data[2048];
        std::vector<float> positions(7);
        std::vector<float> velocities(7);
        std::vector<float> efforts(7);
        bool succ = arm_low_->get_joint_states("all", &positions, &velocities, &efforts);
        assert(succ);

        std::sprintf(data,
                     "%.8f %.8f %.8f %.8f %.8f %.8f %.8f "
                     "%.8f %.8f %.8f %.8f %.8f %.8f %.8f "
                     "%.8f %.8f %.8f %.8f %.8f %.8f %.8f",
                     positions[0], positions[1], positions[2], positions[3], positions[4],
                     positions[5], positions[6], velocities[0], velocities[1], velocities[2],
                     velocities[3], velocities[4], velocities[5], velocities[6], efforts[0],
                     efforts[1], efforts[2], efforts[3], efforts[4], efforts[5], efforts[6]);
        int ret =
            socket_.send_to(boost::asio::buffer(std::string(data)), listener_endpoint_, 0, error);
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

  private:
    // When true, write command and then read state
    bool sync_mode_ = false;
    InterbotixDriverXS* arm_low_ = nullptr;
    std::string address_;
    int port_;
    boost::asio::io_service io_service_;
    udp::endpoint listener_endpoint_;
    udp::socket socket_;
    std::thread thread_;
    std::atomic<bool> kill_{false};
};

UDPDaemon::UDPDaemon(int port, bool sync, std::string filepath_motor_configs,
                     std::string filepath_mode_configs)
    : port_(port), sync_mode_(sync), filepath_motor_configs_(filepath_motor_configs),
      filepath_mode_configs_(filepath_mode_configs) {
    if (filepath_motor_configs == "" || filepath_mode_configs == "") {
        char* username;
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

void UDPDaemon::Start() {
    try {
        io_service_ = std::make_unique<boost::asio::io_service>();
        socket_ = std::make_unique<udp::socket>(*io_service_, udp::endpoint(udp::v4(), port_));
    }
    catch (std::exception& e) {
        spdlog::critical("Fatal error while creating the socket: {}", e.what());
        std::exit(EXIT_FAILURE);
    }

    bool write_eeprom_on_startup = true;
    std::string logging_level = "DEBUG";

    arm_low_ = std::make_unique<InterbotixDriverXS>(filepath_motor_configs_, filepath_mode_configs_,
                                                    write_eeprom_on_startup, logging_level);

    spdlog::info("UDP Daemon for WidowX 250s Arm started successfully.");
    std::unique_ptr<UDPPusher> pusher;

    // The messages should be very small, so the buffer size is sufficient.
    char data[2048];
    size_t content_size = 0;

    while (true) {
        udp::endpoint sender_endpoint;
        // Below is a blocking call that returns as soon as there is data in.
        // The information about the sender is stored in the sneder_endpoint and
        // the message content will be in data.
        try {
            // auto start = std::chrono::high_resolution_clock::now();
            content_size = socket_->receive_from(boost::asio::buffer(data), sender_endpoint);
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> diff = end - start;
            // std::cout << "UDPDaemon: receive cmd from client took: " <<
            // diff.count() << " seconds" << std::endl;
            // TODO(breakds): Do I need to force set null ending?
        }
        catch (std::exception& e) {
            spdlog::error("Error in receiving from socket: {}", e.what());
            continue;
        }

        if (std::strncmp(data, CMD_STATUS, 6) == 0) {
            nlohmann::json status = GetStatus();
            socket_->send_to(boost::asio::buffer(status.dump()), sender_endpoint);
        }
        else if (std::strncmp(data, CMD_BOUNDS, 6) == 0) {
            nlohmann::json bounds = GetBounds();
            socket_->send_to(boost::asio::buffer(bounds.dump()), sender_endpoint);
        }
        else if (std::strncmp(data, CMD_SETPOS, 6) == 0) {
            std::vector<float> positions = ParseArray(data + 7, content_size - 7);
            // TODO(breakds): Check positions has 7 numbers.
            SetPosition(positions);
            // read after 15ms wait, to give policy 2ms, network 1ms and 2ms
            // buffer time.
            if (sync_mode_) {
                for (int i = 0; i < 1; ++i) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(15));
                    pusher->GetStatusAndSend(0);
                }
            }
        }
        else if (std::strncmp(data, CMD_LISTEN, 6) == 0) {
            std::string listener_address = sender_endpoint.address().to_string();
            int listener_port = std::stoi(std::string(data + 7, content_size - 7));
            spdlog::info("Request LISTEN from {}:{}", listener_address, listener_port);
            pusher = std::make_unique<UDPPusher>(arm_low_.get(), listener_address, listener_port,
                                                 sync_mode_);
        }
        else {
            spdlog::error("Invalid command: {}", data);
        }
    }
}

void UDPDaemon::StartMock() {
    try {
        io_service_ = std::make_unique<boost::asio::io_service>();
        socket_ = std::make_unique<udp::socket>(*io_service_, udp::endpoint(udp::v4(), port_));
    }
    catch (std::exception& e) {
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
        // Below is a blocking call that returns as soon as there is data in.
        // The information about the sender is stored in the sneder_endpoint and
        // the message content will be in data.
        try {
            content_size = socket_->receive_from(boost::asio::buffer(data), sender_endpoint);
            // TODO(breakds): Do I need to force set null ending?
        }
        catch (std::exception& e) {
            spdlog::error("Error in receiving from socket: {}", e.what());
            continue;
        }

        if (std::strncmp(data, CMD_STATUS, 6) == 0) {
            nlohmann::json status = ArmStatus{.joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                              .joint_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                              .joint_efforts = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            socket_->send_to(boost::asio::buffer(status.dump()), sender_endpoint);
        }
        else if (std::strncmp(data, CMD_BOUNDS, 6) == 0) {
            nlohmann::json bounds = {
                PositionBound(-1.0, 1.0),   PositionBound(-1.0, 1.0), PositionBound(-1.0, 1.0),
                PositionBound(-1.0, 1.0),   PositionBound(-1.0, 1.0), PositionBound(-1.0, 1.0),
                PositionBound(-0.068, 0.0),
            };
            socket_->send_to(boost::asio::buffer(bounds.dump()), sender_endpoint);
        }
        else if (std::strncmp(data, CMD_SETPOS, 6) == 0) {
            std::vector<float> positions = ParseArray(data + 8, content_size - 8);
            spdlog::info("Set joint positions to [{}, {}, {}, {}, {}, {}] + {}", positions[0],
                         positions[1], positions[2], positions[3], positions[4], positions[5],
                         positions[6]);
        }
        else if (std::strncmp(data, CMD_LISTEN, 6) == 0) {
            std::string listener_address = sender_endpoint.address().to_string();
            int listener_port = std::stoi(std::string(data + 7, content_size - 7));
            spdlog::info("Request LISTEN from {}:{}", listener_address, listener_port);
            pusher = std::make_unique<UDPPusher>(nullptr, listener_address, listener_port);
        }
        else {
            spdlog::error("Invalid command: {}", data);
        }
    }
}  // end of StartMock()

void UDPDaemon::SetPosition(const std::vector<float>& positions) {
    if (positions.size() != 7) {
        spdlog::error("SetPosition() cannot handle inputs with a size of {}", positions.size());
        return;
    }
    arm_low_->write_position_commands("all", positions);
}

ArmStatus UDPDaemon::GetStatus() {
    std::vector<float> positions(7);
    std::vector<float> velocities(7);
    std::vector<float> efforts(7);
    bool succ = arm_low_->get_joint_states("all", &positions, &velocities, &efforts);
    assert(succ);

    return ArmStatus{
        .joint_positions = {positions[0], positions[1], positions[2], positions[3], positions[4],
                            positions[5], positions[6]},
        .joint_velocities = {velocities[0], velocities[1], velocities[2], velocities[3],
                             velocities[4], velocities[5], velocities[6]},
        .joint_efforts = {efforts[0], efforts[1], efforts[2], efforts[3], efforts[4], efforts[5],
                          efforts[6]},
    };
}

std::vector<PositionBound> UDPDaemon::GetBounds() {
    return {
        PositionBound(-0.068, 0.0), PositionBound(-0.068, 0.0), PositionBound(-0.068, 0.0),
        PositionBound(-0.068, 0.0), PositionBound(-0.068, 0.0), PositionBound(-0.068, 0.0),
        PositionBound(-0.068, 0.0),
    };
}

}  // namespace horizon::widowx
