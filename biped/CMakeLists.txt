cmake_minimum_required(VERSION 3.8)
project(biped)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(gz_ros2_control REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz, URDF, config, worlds, and meshes files
install(
  DIRECTORY rviz urdf config worlds meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src
)
# Export the package
ament_package()
