cmake_minimum_required(VERSION 3.14)
project(interbotix_xs_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)           # Set C++ standard to C++20
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # Require C++20 support
# Use C++20 for jthread

include(CMakeDependentOption)
include(GNUInstallDirs)
include(InstallRequiredSystemLibraries)
include(CMakePackageConfigHelpers)

# NOTE(breakds): pkgconfig will handle the third party dependencies as long as
# they are installed, so we don't have to put them here in add_subdirectory.

# add_subdirectory(third_party_libraries/DynamixelSDK)
# add_subdirectory(third_party_libraries/dynamixel-workbench)

find_package(yaml-cpp REQUIRED)

set(DEPENDENCIES
  dynamixel_workbench_toolbox
  yaml-cpp
)

# Generate version header file from package.xml
include(cmake/Version.cmake)
message(${PATH_TO_VERSION_OUTPUT_DIR})
include_directories(${PATH_TO_VERSION_OUTPUT_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(
  ${PROJECT_NAME} SHARED
    src/xs_driver.cpp
    src/xs_logging.cpp
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${DEPENDENCIES}
)


# Arm server:
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(protocols INTERFACE)
target_sources(protocols INTERFACE src/protocols.h)
target_link_libraries(protocols INTERFACE nlohmann_json::nlohmann_json)

# +----------------------------------------+
# | The Arm Server (WX Armor)              |
# +----------------------------------------+

add_library(robot_profile STATIC)
target_sources(robot_profile
    PUBLIC
    FILE_SET HEADERS
    BASE_DIRS wx_armor
    FILES wx_armor/robot_profile.h
    PRIVATE wx_armor/robot_profile.cc)
target_link_libraries(robot_profile PUBLIC
    spdlog::spdlog
    yaml-cpp)

add_library(io STATIC)
target_sources(io
    PUBLIC
    FILE_SET HEADERS
    BASE_DIRS wx_armor
    FILES wx_armor/io.h
    PRIVATE wx_armor/io.cc)
target_link_libraries(io PUBLIC
    dynamixel_workbench_toolbox
    spdlog::spdlog)

add_library(wx_armor_driver STATIC)
target_sources(wx_armor_driver
    PUBLIC
    FILE_SET HEADERS
    BASE_DIRS wx_armor
    FILES wx_armor/wx_armor_driver.h
    PRIVATE wx_armor/wx_armor_driver.cc)
target_link_libraries(wx_armor_driver PUBLIC
    io
    robot_profile
    dynamixel_workbench_toolbox
    spdlog::spdlog
    yaml-cpp)

add_executable(wx_armor)
target_sources(wx_armor PRIVATE wx_armor/wx_armor.cc)
target_link_libraries(wx_armor PRIVATE wx_armor_driver)

# +----------------------------------------+
# | The UDP-based Server                   |
# +----------------------------------------+

add_library(udp_daemon STATIC)
target_include_directories(udp_daemon PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_sources(udp_daemon
    PUBLIC src/udp_daemon.h
    PRIVATE src/udp_daemon.cpp)
target_link_libraries(udp_daemon PUBLIC
    ${PROJECT_NAME}  # interbotix_xs_driver
    protocols
    spdlog::spdlog)

add_executable(udp_client)
target_sources(udp_client PRIVATE src/udp_client.cpp)

add_executable(wx_serverd)
target_include_directories(wx_serverd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_sources(wx_serverd PRIVATE src/wx_serverd.cpp)
target_link_libraries(wx_serverd PRIVATE udp_daemon spdlog::spdlog)


install(
  TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  FILES
    ${PATH_TO_VERSION_OUTPUT}
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  FILES
    package.xml
  DESTINATION
    share/${PROJECT_NAME}
)
