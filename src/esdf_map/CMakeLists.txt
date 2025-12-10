cmake_minimum_required(VERSION 3.8)
project(esdf_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueryEsdf.srv"
  DEPENDENCIES geometry_msgs
)

include_directories(include)

add_library(esdf_map_core
  src/esdf_map_core.cpp
)
target_compile_features(esdf_map_core PUBLIC cxx_std_17)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(esdf_map_node_lib
  src/esdf_map_node.cpp
)
target_compile_features(esdf_map_node_lib PUBLIC cxx_std_17)
target_link_libraries(esdf_map_node_lib esdf_map_core ${cpp_typesupport_target})
ament_target_dependencies(esdf_map_node_lib
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

add_executable(esdf_map_node
  src/main.cpp
)
target_link_libraries(esdf_map_node esdf_map_node_lib)
ament_target_dependencies(esdf_map_node rclcpp)

install(TARGETS
  esdf_map_core
  esdf_map_node_lib
  esdf_map_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  rosidl_default_runtime
)

ament_package()
