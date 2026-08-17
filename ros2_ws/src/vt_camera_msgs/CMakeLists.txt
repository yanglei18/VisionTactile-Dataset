cmake_minimum_required(VERSION 3.8)
project(vt_camera_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(vt_camera_msgs
  "msg/CameraFrameTiming.msg"
  "msg/CameraGroupStatus.msg"
  "msg/CameraDescriptor.msg"
  "msg/StreamProfile.msg"
  "msg/SessionInfo.msg"
  "msg/CaptureCommand.msg"
  "msg/StreamStatus.msg"
  "msg/CaptureStatus.msg"
  "msg/CaptureEvent.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_interface_schema test/test_interface_schema.py)
endif()

ament_package()
