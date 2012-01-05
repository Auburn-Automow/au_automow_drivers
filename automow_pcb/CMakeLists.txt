cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(rosserial_arduino)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)

set(FIRMWARE_NAME power_control)
set(${FIRMWARE_NAME}_BOARD pro5v328)
set(${FIRMWARE_NAME}_SRCS src/power_control.cpp )
set(${FIRMWARE_NAME}_PORT /dev/power_control)
generate_ros_firmware(${FIRMWARE_NAME})
