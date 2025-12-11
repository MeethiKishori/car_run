cmake_minimum_required(VERSION 3.5)
project(f1tenth_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
# You should also add these control packages for proper Gazebo integration:
find_package(ros2_control REQUIRED) 
find_package(ros2_controllers REQUIRED) 

# --- CRITICAL ADDITIONS START HERE ---

# 1. Install URDF/XACRO files
# The destination is 'share/f1tenth_description' so that $(find f1tenth_description)/urdf works.
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install Mesh files
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install Gazebo model files (cones, tracks, etc.)
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# 4. Install Configuration files (like controllers.yaml, if applicable)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# --- CRITICAL ADDITIONS END HERE ---


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ... (linter setup)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()