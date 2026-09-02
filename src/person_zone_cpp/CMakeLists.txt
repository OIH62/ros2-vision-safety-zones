cmake_minimum_required(VERSION 3.8)
project(person_zone_cpp)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(person_pose_msgs REQUIRED)

add_library(person_zone_core src/zone_core.cpp)
target_compile_features(person_zone_core PUBLIC cxx_std_17)
target_include_directories(
  person_zone_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(person_zone_node src/person_zone_node.cpp)
target_compile_features(person_zone_node PUBLIC cxx_std_17)
target_link_libraries(person_zone_node person_zone_core)
ament_target_dependencies(person_zone_node rclcpp std_msgs sensor_msgs geometry_msgs std_srvs person_pose_msgs)
install(
  TARGETS person_zone_core person_zone_node
  EXPORT person_zone_core_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_zone_core test/test_zone_core.cpp)
  target_link_libraries(test_zone_core person_zone_core)
endif()

ament_export_targets(person_zone_core_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(person_zone_core)
ament_package()
