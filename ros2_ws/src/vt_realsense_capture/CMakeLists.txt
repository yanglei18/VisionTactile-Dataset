cmake_minimum_required(VERSION 3.8)
project(vt_realsense_capture)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realsense2 REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vt_camera_msgs REQUIRED)

add_library(metadata_parser src/metadata_parser.cpp)
target_compile_features(metadata_parser PUBLIC cxx_std_17)
target_include_directories(
  metadata_parser
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(metadata_parser PUBLIC nlohmann_json::nlohmann_json)

add_library(camera_frame_aggregator src/camera_frame_aggregator.cpp)
target_compile_features(camera_frame_aggregator PUBLIC cxx_std_17)
target_include_directories(
  camera_frame_aggregator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(camera_frame_aggregator vt_camera_msgs)
target_link_libraries(camera_frame_aggregator metadata_parser)

add_executable(timing_normalizer src/timing_normalizer_node.cpp)
target_compile_features(timing_normalizer PRIVATE cxx_std_17)
ament_target_dependencies(
  timing_normalizer rclcpp realsense2_camera_msgs std_srvs vt_camera_msgs)
target_link_libraries(timing_normalizer camera_frame_aggregator)

add_executable(depth_scale_probe src/depth_scale_probe.cpp)
target_compile_features(depth_scale_probe PRIVATE cxx_std_17)
target_link_libraries(depth_scale_probe realsense2::realsense2)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  FILES launch/triple_realsense.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS camera_frame_aggregator metadata_parser
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS timing_normalizer DESTINATION lib/${PROJECT_NAME})
install(TARGETS depth_scale_probe DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS vt_realsense_capture/controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME capture_controller)
install(
  PROGRAMS scripts/storage_bench.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME storage_bench)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_metadata_parser test/test_metadata_parser.cpp)
  target_compile_features(test_metadata_parser PRIVATE cxx_std_17)
  target_include_directories(test_metadata_parser PRIVATE include)
  target_compile_definitions(
    test_metadata_parser
    PRIVATE METADATA_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  target_link_libraries(test_metadata_parser metadata_parser)

  ament_add_gtest(
    test_camera_frame_aggregator test/test_camera_frame_aggregator.cpp)
  target_compile_features(test_camera_frame_aggregator PRIVATE cxx_std_17)
  target_include_directories(test_camera_frame_aggregator PRIVATE include)
  ament_target_dependencies(test_camera_frame_aggregator vt_camera_msgs)
  target_link_libraries(test_camera_frame_aggregator camera_frame_aggregator)

  ament_add_pytest_test(test_config test/test_config.py)
  ament_add_pytest_test(test_session test/test_session.py)
  ament_add_pytest_test(test_controller test/test_controller.py)
  ament_add_pytest_test(test_controller_state test/test_controller_state.py)
  ament_add_pytest_test(test_recorder test/test_recorder.py)
  ament_add_pytest_test(test_storage test/test_storage.py)
  ament_add_pytest_test(test_launch_arguments test/test_launch_arguments.py)
  ament_add_pytest_test(test_package_contract test/test_package_contract.py)
  ament_add_pytest_test(test_synthetic_capture test/test_synthetic_capture.py TIMEOUT 60)
  install(
    PROGRAMS test/synthetic_capture_support.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME synthetic_capture_test_support)
  install(
    FILES launch/synthetic_capture.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)
endif()

ament_export_dependencies(
  nlohmann_json realsense2_camera_msgs std_srvs vt_camera_msgs)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
