cmake_minimum_required(VERSION 3.5)
project(reaction_wheel_control)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  # Added this line

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Define the plugin library first
add_library(reaction_wheel_plugin SHARED src/reaction_wheel_plugin.cc)

# Now set include directories (AFTER the target is declared)
target_include_directories(reaction_wheel_plugin PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)

# Link dependencies
target_link_libraries(reaction_wheel_plugin ${GAZEBO_LIBRARIES})

# Add ament dependencies
ament_target_dependencies(reaction_wheel_plugin 
  rclcpp 
  gazebo_ros_pkgs
  std_msgs  # Added this
)

# Install
install(
  TARGETS reaction_wheel_plugin
  DESTINATION lib
)
install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)  # Added this

ament_package()
