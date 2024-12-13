#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

HOME_DIR=$(pwd)

# Parse flags
FRESH_INSTALL=false

# Check for the --fresh flag
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --fresh) FRESH_INSTALL=true ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Function to build a project
build_project() {
    local project_dir=$1
    cd "$project_dir"

    if [ "$FRESH_INSTALL" = true ]; then
        rm -rf build
    fi

    mkdir -p build
    cd build
    cmake ..
    make -j"$(nproc)"
    make install
    cd "$HOME_DIR"
}


if [ "$FRESH_INSTALL" = true ]; then
    apt install -y cmake
    apt install -y build-essential
    apt install -y libyaml-cpp-dev
    apt install libjsoncpp-dev
    apt install uuid-dev


    # Update cmake to version >= 3.23 for Ubuntu 22.04
    # For other repos, refer to
    # https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line#:~:text=Kitware%20now%20has%20an%20APT%20repository%20that%20currently%20supports%2020.04%2C%2022.04%20and%2024.04.
    # Get the installed version of CMake
    CMAKE_VERSION=$(cmake --version | head -n 1 | awk '{print $3}')
    # Compare the version to 3.23
    if [ "$(printf '%s\n' "$CMAKE_VERSION" "3.23" | sort -V | head -n 1)" != "3.23" ]; then
        echo "CMake version is less than 3.23 (current: $CMAKE_VERSION). Updating..."
        # Update CMake
        apt purge --auto-remove cmake
        wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
        echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null
        apt update
        apt install cmake
    else
        echo "CMake version is 3.23 or newer (current: $CMAKE_VERSION). No update needed."
    fi

    # Install json
    cd /tmp
    rm -rf json
    git clone https://github.com/nlohmann/json
    cd json
    mkdir build
    cd build
    cmake ..
    make -j"$(nproc)"
    make install
    cd /tmp
    rm -rf json
    cd "$HOME_DIR"

    # Install drogon
    cd /tmp
    rm -rf drogon
    git clone https://github.com/drogonframework/drogon
    cd drogon
    git submodule update --init
    mkdir build
    cd build
    cmake ..
    make -j"$(nproc)"
    make install
    cd /tmp
    rm -rf drogon
    cd "$HOME_DIR"

    # Install spdlog
    cd /tmp
    rm -rf spdlog
    git clone https://github.com/gabime/spdlog.git
    cd spdlog
    mkdir build
    cd build
    cmake -DSPDLOG_BUILD_SHARED=on ..
    make -j"$(nproc)"
    make install
    cd /tmp
    rm -rf spdlog
    cd "$HOME_DIR"

    # Build dynamixel_sdk
    build_project "third_party_libraries/DynamixelSDK"

    # Build dynamixel_workbench
    build_project "third_party_libraries/dynamixel-workbench"
fi

# Build wx_armor
build_project "wx_armor"
