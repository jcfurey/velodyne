cmake_minimum_required(VERSION 3.16)
project(velodyne_laserscan)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/velodyne_laserscan.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp_components::component
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "velodyne_laserscan::VelodyneLaserScan")

add_executable(${PROJECT_NAME}_node
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}_node PRIVATE rclcpp::rclcpp ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Upstream sources keep their original ROS 1-era BSD notices, which
  # ament_copyright cannot parse; do not rewrite third-party headers.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(tests)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rcl_interfaces sensor_msgs)

ament_package()
