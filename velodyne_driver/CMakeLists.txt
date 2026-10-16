cmake_minimum_required(VERSION 3.16)
project(velodyne_driver)

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
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Threads REQUIRED)
find_package(velodyne_msgs REQUIRED)

find_path(PCAP_INCLUDE_DIR NAMES pcap.h)
find_library(PCAP_LIBRARY NAMES pcap)
if(NOT PCAP_INCLUDE_DIR OR NOT PCAP_LIBRARY)
  message(FATAL_ERROR "libpcap headers and library are required")
endif()

# compile the driver and input library
add_library(velodyne_input SHARED src/lib/input.cpp)
target_compile_features(velodyne_input PUBLIC cxx_std_17)
target_include_directories(velodyne_input PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${PCAP_INCLUDE_DIR})
target_link_libraries(velodyne_input PUBLIC
  ${PCAP_LIBRARY}
  rclcpp::rclcpp
  ${velodyne_msgs_TARGETS}
)

# build the driver node
add_library(velodyne_driver SHARED src/driver/driver.cpp)
target_compile_features(velodyne_driver PUBLIC cxx_std_17)
target_include_directories(velodyne_driver PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(velodyne_driver PUBLIC
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  velodyne_input
  Threads::Threads
  ${velodyne_msgs_TARGETS}
)
target_link_libraries(velodyne_driver PRIVATE
  rclcpp_components::component
)

add_executable(velodyne_driver_node src/driver/velodyne_node.cpp)
target_link_libraries(velodyne_driver_node PRIVATE rclcpp::rclcpp velodyne_driver)

# install runtime and library files
install(TARGETS velodyne_input velodyne_driver
  EXPORT velodyne_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS velodyne_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_nodes(velodyne_driver
  "velodyne_driver::VelodyneDriver")

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS src/vdump
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  # Upstream sources keep their original ROS 1-era BSD notices, which
  # ament_copyright cannot parse; do not rewrite third-party headers.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # unit test
  ament_add_gtest(time_test tests/timeconversiontest.cpp)
  target_link_libraries(time_test velodyne_input)

  ament_add_gtest(test_input tests/test_input.cpp TIMEOUT 30)
  target_link_libraries(test_input velodyne_driver)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(velodyne_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Threads
  diagnostic_updater
  rclcpp
  rcl_interfaces
  velodyne_msgs
)

ament_package()
