cmake_minimum_required(VERSION 3.8)
project(testAMR7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ============================================================================
# FIND DEPENDENCIES
# ============================================================================
# Core ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Controller framework packages
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Message and transform packages
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)          # NEW: For Float32MultiArray in Arduino interface
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Realtime tools for publishing
find_package(realtime_tools REQUIRED)

# ============================================================================
# BUILD OMNI BASE CONTROLLER LIBRARY
# ============================================================================
# This is your custom swerve drive controller that implements the control logic
# It receives cmd_vel messages and computes wheel velocities using inverse kinematics

add_library(
  omni_base_controller
  SHARED
  src/omni_base_controller.cpp
)

# Specify include directories for the controller
target_include_directories(
  omni_base_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link controller dependencies
ament_target_dependencies(
  omni_base_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  realtime_tools
)

# Export the controller plugin so ros2_control can find it
pluginlib_export_plugin_description_file(
  controller_interface 
  omni_base_controller_plugin.xml
)

# ============================================================================
# BUILD ARDUINO HARDWARE INTERFACE LIBRARY
# ============================================================================
# This is the hardware interface that communicates with Arduino via serial
# It sends velocity commands to Arduino and reads sensor feedback (currents)
# This replaces Gazebo simulation when using real hardware

add_library(
  swerve_arduino_hardware_interface
  SHARED
  src/swerve_arduino_hardware_interface.cpp
)

# Specify include directories for the hardware interface
target_include_directories(
  swerve_arduino_hardware_interface
  PRIVATE
  include
)

# Link hardware interface dependencies
ament_target_dependencies(
  swerve_arduino_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs                 # For publishing motor current data
)

# Export the hardware interface plugin so ros2_control can find it
pluginlib_export_plugin_description_file(
  hardware_interface 
  swerve_arduino_hardware_interface_plugin.xml
)

# ============================================================================
# INSTALL LIBRARIES
# ============================================================================

# Install the controller library
install(
  TARGETS omni_base_controller
  EXPORT export_omni_base_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the Arduino hardware interface library
install(
  TARGETS swerve_arduino_hardware_interface
  EXPORT export_swerve_arduino_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# INSTALL HEADER FILES
# ============================================================================
# Install all header files so other packages can use them if needed
install(
  DIRECTORY include/
  DESTINATION include
)

# ============================================================================
# INSTALL PACKAGE DATA FILES
# ============================================================================
# Install all configuration, description, and visualization files
install(
  DIRECTORY
    urdf         # Robot description files (URDF/xacro)
    meshes       # 3D mesh files (.STL) for visualization
    rviz         # RViz configuration files
    config       # Controller parameters and gamepad config
    launch       # Launch files to start the robot
    worlds       # Gazebo world files
  DESTINATION share/${PROJECT_NAME}
)


# Install Python scripts
install(PROGRAMS
  src/swerve_gamepad_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# EXPORT TARGETS AND DEPENDENCIES
# ============================================================================

# Export the controller library target
ament_export_targets(export_omni_base_controller HAS_LIBRARY_TARGET)

# Export the hardware interface library target
ament_export_targets(export_swerve_arduino_hardware_interface HAS_LIBRARY_TARGET)

# Export all dependencies so downstream packages can use them
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  realtime_tools
)

# ============================================================================
# FINALIZE PACKAGE
# ============================================================================
# This must be the last line - it generates all the CMake config files
ament_package()

# ============================================================================
# SUMMARY OF WHAT THIS FILE DOES
# ============================================================================
# 
# 1. CONTROLLER (omni_base_controller):
#    - Implements swerve drive kinematics
#    - Receives /cmd_vel commands from teleop or navigation
#    - Computes wheel and steering velocities
#    - Sends commands through ros2_control interfaces
#
# 2. HARDWARE INTERFACE (swerve_arduino_hardware_interface):
#    - Communicates with Arduino Mega via serial (/dev/ttyACM0)
#    - Sends 8 velocity commands (4 steering + 4 wheels) to Arduino
#    - Receives 8 current values from Arduino
#    - Publishes currents to /motor_currents topic
#
# 3. PLUGIN SYSTEM:
#    - Both controller and hardware interface are plugins
#    - ros2_control dynamically loads them at runtime
#    - Plugin XML files tell ros2_control where to find them
#
# 4. INSTALLATION:
#    - Libraries go to lib/ directory
#    - Headers go to include/ directory
#    - URDF, meshes, configs go to share/testAMR7/
#
# 5. EXPORTS:
#    - Other packages can depend on this package
#    - They get access to the libraries and dependencies
#
# ============================================================================