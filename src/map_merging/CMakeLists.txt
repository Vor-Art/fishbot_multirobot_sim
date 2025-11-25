cmake_minimum_required(VERSION 3.8)
project(fishbot_map_merging)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetMergedMap.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_merging_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
