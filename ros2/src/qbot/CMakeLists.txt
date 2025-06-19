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
find_package(image_transport REQUIRED)

# Find OpenCV - IMPORTANT for ORB and drawing functions
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d)

# ----------------------------------------------------------------------
# Executable for Image Decompressor Node
# ----------------------------------------------------------------------
add_executable(image_decompressor src/image_decompressor.cpp)

ament_target_dependencies(image_decompressor
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport # If image_decompressor also uses image_transport
)

# Link image_decompressor with OpenCV if it uses any OpenCV functions directly
# Based on the latest image_decompressor.cpp provided, it might not need direct OpenCV linking
# if it only uses cv_bridge for conversion and not ORB.
# However, if it previously used OpenCV for ORB, ensure it's linked here.
# For the *stripped-down* image_decompressor.cpp (only decompression), you technically don't need these lines
# but keeping them here is safe if you later add more OpenCV.
target_link_libraries(image_decompressor
  ${OpenCV_LIBS}
)


# ----------------------------------------------------------------------
# Executable for Camera IMU Sync (formerly ORB Detector) Node
# ----------------------------------------------------------------------
# Rename the executable target and source file
add_executable(camera_imu_sync src/camera_imu_sync.cpp)

# Define dependencies for the new node
ament_target_dependencies(camera_imu_sync
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport # If camera_imu_sync also uses image_transport
)

# Link the camera_imu_sync executable with OpenCV libraries
# This is crucial for ORB::create(), drawKeypoints(), imwrite(), etc.
target_link_libraries(camera_imu_sync
  ${OpenCV_LIBS}
)

# ----------------------------------------------------------------------
# Installation
# ----------------------------------------------------------------------
install(TARGETS
  image_decompressor
  camera_imu_sync # Add the new executable for installation
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()