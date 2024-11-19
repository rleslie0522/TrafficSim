cmake_minimum_required(VERSION 3.8)
project(trafficsim)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "./srv/SpawnRobot.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
