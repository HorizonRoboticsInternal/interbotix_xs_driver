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

find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
# find_package(dynamixel_workbench_toolbox)

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
    spdlog::spdlog
    yaml-cpp)

add_executable(wx_armor)
target_sources(wx_armor PRIVATE wx_armor/wx_armor.cc)
target_link_libraries(wx_armor PRIVATE wx_armor_driver)

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
    TARGETS robot_profile wx_armor_driver wx_armor
    EXPORT ${PROJECT_NAME}_Targets
    FILE_SET HEADERS)

install(
    EXPORT ${PROJECT_NAME}_Targets
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
