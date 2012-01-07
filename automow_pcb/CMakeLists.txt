cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(rosserial_arduino)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)

include_directories(${PROJECT_SOURCE_DIR}/src/OneWire
                    ${PROJECT_SOURCE_DIR}/src/Metro
                    ${PROJECT_SOURCE_DIR}/src/DallasTemperature)

set(FIRMWARE_NAME power_control)
set(${FIRMWARE_NAME}_BOARD atmega328)
set(${FIRMWARE_NAME}_SRCS src/power_control.cpp
    src/Metro/Metro.cpp
    src/OneWire/OneWire.cpp
    src/DallasTemperature/DallasTemperature.cpp)
set(${FIRMWARE_NAME}_HDR
    src/Metro/Metro.h
    src/OneWire/OneWire.h
    src/DallasTemperature/DallasTemperature.h)
set(${FIRMWARE_NAME}_PORT /dev/power_control)
generate_ros_firmware(${FIRMWARE_NAME})
