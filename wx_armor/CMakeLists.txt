cmake_minimum_required(VERSION 3.23)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Do not build in-source. Please remove CMakeCache.txt and the CMakeFiles/ directory. Then build out-of-source.")
endif()

project(wx_armor
    LANGUAGES CXX
    VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CMakeDependentOption)
include(GNUInstallDirs)
include(InstallRequiredSystemLibraries)
include(CMakePackageConfigHelpers)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# ┌─────────────────┐
# │ Dependencies    │
# └─────────────────┘

# Add third-party libraries
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party_libraries/DynamixelSDK DynamixelSDK)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party_libraries/dynamixel-workbench dynamixel-workbench)

find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Drogon REQUIRED)

# ┌─────────────────┐
# │ Targets         │
# └─────────────────┘

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

add_library(wx_armor_driver STATIC)
target_sources(wx_armor_driver
    PUBLIC
    FILE_SET HEADERS
    BASE_DIRS wx_armor
    FILES wx_armor/wx_armor_driver.h
    PRIVATE wx_armor/wx_armor_driver.cc)
target_link_libraries(wx_armor_driver PUBLIC
    robot_profile
    dynamixel_workbench_toolbox
    dynamixel_sdk
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    yaml-cpp)

add_library(guardian_thread STATIC)
target_sources(guardian_thread
        PUBLIC
        FILE_SET HEADERS
        BASE_DIRS wx_armor
        FILES wx_armor/guardian_thread.h
        PRIVATE wx_armor/guardian_thread.cc)
target_link_libraries(guardian_thread PUBLIC
        Drogon::Drogon
        wx_armor_driver
        spdlog::spdlog
        nlohmann_json::nlohmann_json
)

add_library(wx_armor_ws STATIC)
target_sources(wx_armor_ws
    PUBLIC
    FILE_SET HEADERS
    BASE_DIRS wx_armor
    FILES wx_armor/wx_armor_ws.h
    PRIVATE wx_armor/wx_armor_ws.cc)
target_link_libraries(wx_armor_ws PUBLIC
    Drogon::Drogon
    spdlog::spdlog
    guardian_thread
    wx_armor_driver)

add_executable(wx_armor)
target_sources(wx_armor PRIVATE wx_armor/wx_armor.cc)
target_link_libraries(wx_armor PRIVATE wx_armor_ws)

# ┌─────────────────┐
# │ Install         │
# └─────────────────┘

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERISON}
    COMPATIBILITY SameMajorVersion)


install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

install(
    TARGETS robot_profile wx_armor_driver wx_armor_ws wx_armor guardian_thread dynamixel_workbench_toolbox dynamixel_sdk
    EXPORT ${PROJECT_NAME}_Targets
    FILE_SET HEADERS)

install(
    EXPORT ${PROJECT_NAME}_Targets
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
