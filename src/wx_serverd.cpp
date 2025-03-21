#include <cstdlib>
#include <cstring>
#include <string>
#include <type_traits>

#include "spdlog/spdlog.h"
#include "udp_daemon.h"

using horizon::widowx::UDPDaemon;

template <typename T>
T GetEnv(const char* name, T default_value) {
    const char* value = std::getenv(name);
    if (value == nullptr) {
        return default_value;
    }

    if constexpr (std::is_same_v<T, int>) {
        return std::stoi(value);
    }
    else if constexpr (std::is_same_v<T, bool>) {
        return (std::strcmp(value, "yes") == 0 || std::strcmp(value, "1") == 0 ||
                std::strcmp(value, "true") == 0 || std::strcmp(value, "T") == 0);
    }
    else if constexpr (std::is_same_v<T, std::string>) {
        return std::string(value);
    }
    else {
        spdlog::critical("Unspported type for GetEnv, name = '{}'", name);
        std::exit(EXIT_FAILURE);
    }
}

int main(int argc, char** argv) {
    int sync = 0;
    if (argc > 1)
        sync = atoi(argv[1]);
    UDPDaemon daemon(GetEnv<int>("WX_PORT", 9211), (bool)sync);

    if (GetEnv<bool>("WXD_MOCK", false)) {
        // Run in mock mode
        daemon.StartMock();
    }
    else {
        daemon.Start();
    }
    return 0;
}
