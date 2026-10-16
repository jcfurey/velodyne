cmake_minimum_required(VERSION 3.16)
project(velodyne_pointcloud)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(velodyne_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# Older versions of yaml-cpp have a target called "yaml-cpp",
# so use that where appropriate.
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET "yaml-cpp::yaml-cpp")
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

add_library(velodyne_rawdata SHARED
  src/lib/rawdata.cpp
  src/lib/calibration.cpp)
target_compile_features(velodyne_rawdata PUBLIC cxx_std_17)
target_include_directories(velodyne_rawdata PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(velodyne_rawdata PUBLIC
  angles::angles
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${velodyne_msgs_TARGETS}
  ${YAML_CPP_TARGET}
)

add_library(velodyne_cloud_types SHARED
  src/lib/pointcloudXYZIRT.cpp
  src/lib/organized_cloudXYZIRT.cpp
)
target_compile_features(velodyne_cloud_types PUBLIC cxx_std_17)
target_include_directories(velodyne_cloud_types PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(velodyne_cloud_types PUBLIC
  Eigen3::Eigen
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${velodyne_msgs_TARGETS}
)

add_library(transform SHARED
  src/conversions/transform.cpp)
target_compile_features(transform PUBLIC cxx_std_17)
target_include_directories(transform PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(transform PUBLIC
  diagnostic_updater::diagnostic_updater
  Eigen3::Eigen
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  velodyne_cloud_types
  velodyne_rawdata
  ${YAML_CPP_TARGET}
)
target_link_libraries(transform PRIVATE
  rclcpp_components::component
)

install(TARGETS velodyne_cloud_types velodyne_rawdata transform
  EXPORT ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

rclcpp_components_register_nodes(transform
  "velodyne_pointcloud::Transform")

add_executable(velodyne_transform_node
  src/conversions/transform_node.cpp)
target_include_directories(velodyne_transform_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(velodyne_transform_node PRIVATE
  rclcpp::rclcpp
  transform
)
install(TARGETS velodyne_transform_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch params
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Remove empty.xml from lint
  # Upstream sources keep their original ROS 1-era BSD notices, which
  # ament_copyright cannot parse; do not rewrite third-party headers.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_xmllint
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE XML_FILES
    "*.xml"
  )

  get_filename_component(
    EMPTY_XML_ABS
    "tests/empty.xml"
    ABSOLUTE
  )

  list(REMOVE_ITEM XML_FILES ${EMPTY_XML_ABS})

  find_package(ament_cmake_xmllint REQUIRED)

  ament_xmllint(${XML_FILES})

  # Files created by this fork carry recognized notices; keep them checked.
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright(TESTNAME copyright_fork
    tests/test_vlp16_exyn.cpp tests/test_decoding.cpp tests/test_utils.hpp)

  add_subdirectory(tests)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  angles
  diagnostic_updater
  Eigen3
  geometry_msgs
  rclcpp
  rcl_interfaces
  sensor_msgs
  tf2
  tf2_ros
  velodyne_msgs
  yaml_cpp_vendor
  yaml-cpp
)
ament_export_libraries(velodyne_rawdata velodyne_cloud_types)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
