cmake_minimum_required(VERSION 3.10)
project(imu_validator)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_library(${PROJECT_NAME}
  src/imu_validator.cpp
  src/imu_validation_test_framework/programmatic_imu_data_source.cpp
  src/imu_validation_test_framework/rosbag_imu_data_source.cpp
  src/imu_validation_test_framework/scenario_validator.cpp
  src/imu_validation_test_framework/validation_scenario_runner.cpp
  src/imu_validation_test_framework/yaml_platform_profile_provider.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_programmatic_source_and_runner
    test/test_programmatic_source_and_runner.cpp
  )
  target_link_libraries(test_programmatic_source_and_runner ${PROJECT_NAME})

  ament_add_gtest(test_yaml_profile_provider
    test/test_yaml_profile_provider.cpp
  )
  target_link_libraries(test_yaml_profile_provider ${PROJECT_NAME})
  target_compile_definitions(test_yaml_profile_provider PRIVATE
    TEST_PLATFORM_PROFILE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config/platform_profiles")

  ament_add_gtest(test_extension_registry
    test/test_extension_registry.cpp
  )
  target_link_libraries(test_extension_registry ${PROJECT_NAME})

  ament_add_gtest(test_imu_validation_profiles_and_anomalies
    tests/test_imu_validation_profiles_and_anomalies.cpp
  )
  target_link_libraries(test_imu_validation_profiles_and_anomalies ${PROJECT_NAME})

  ament_add_gtest(test_time_sequence_and_rosbag_replay
    tests/test_time_sequence_and_rosbag_replay.cpp
  )
  target_link_libraries(test_time_sequence_and_rosbag_replay ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs rosbag2_cpp rosbag2_storage)
ament_package()
