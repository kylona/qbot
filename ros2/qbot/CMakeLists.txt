cmake_minimum_required(VERSION 3.8)
project(qbot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# Add find_package for image_transport
find_package(image_transport REQUIRED)

add_executable(image_decompressor src/image_decompressor.cpp)

# Use ament_target_dependencies to handle includes and linking
ament_target_dependencies(image_decompressor
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

# Note: No need for target_link_libraries if using ament_target_dependencies
# for these ROS packages.

install(TARGETS image_decompressor
        DESTINATION lib/${PROJECT_NAME}) # Use PROJECT_NAME for consistency

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
