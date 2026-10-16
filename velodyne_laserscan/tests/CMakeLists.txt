find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_projection test_projection.cpp)
target_link_libraries(test_projection ${PROJECT_NAME})
