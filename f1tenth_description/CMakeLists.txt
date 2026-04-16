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

# 4. Create symlinks from mesh files into each model folder for Gazebo resolution
# This allows model.sdf files to find meshes via relative paths after installation
install(CODE "
  set(INSTALL_PREFIX \${CMAKE_INSTALL_PREFIX})
  set(MODELS_DIR \"\${INSTALL_PREFIX}/share/${PROJECT_NAME}/models\")
  set(MESHES_DIR \"\${INSTALL_PREFIX}/share/${PROJECT_NAME}/meshes\")

  # Function to create symlinks for each model
  function(create_mesh_symlinks MODEL_NAME MESH_FILE)
    set(MODEL_MESH_DIR \"\${MODELS_DIR}/\${MODEL_NAME}/meshes\")
    execute_process(COMMAND \${CMAKE_COMMAND} -E make_directory \"\${MODEL_MESH_DIR}\")
    execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink 
      \"\${MESHES_DIR}/\${MESH_FILE}\" 
      \"\${MODEL_MESH_DIR}/\${MESH_FILE}\")
  endfunction()

  # Create symlinks for each model's mesh
  create_mesh_symlinks(cone cone.dae)
  create_mesh_symlinks(parking_1 parking_1.dae)
  create_mesh_symlinks(walker_racecourse walker_racecourse.dae)
")

# 5. Install Configuration files (like controllers.yaml, if applicable)
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