cmake_minimum_required(VERSION 3.8)
project(esdf_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(
  -O3
  -march=native
  -ffast-math
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# --- Interfaces (service) ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueryEsdf.srv"
  DEPENDENCIES geometry_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# --- Core library (ESDF implementation, no ROS) ---
add_library(esdf_map_core
  src/esdf_map_core.cpp
)
target_compile_features(esdf_map_core PUBLIC cxx_std_17)
target_link_libraries(esdf_map_core
  ${PCL_LIBRARIES}
)

# Typesupport for our srv
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- ROS node library ---
add_library(esdf_map_node_lib
  src/esdf_map_node.cpp
)
target_compile_features(esdf_map_node_lib PUBLIC cxx_std_17)
target_link_libraries(esdf_map_node_lib
  esdf_map_core
  ${cpp_typesupport_target}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(esdf_map_node_lib
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
)

# --- Node executable ---
add_executable(esdf_map_node
  src/main.cpp
)
target_link_libraries(esdf_map_node esdf_map_node_lib)
ament_target_dependencies(esdf_map_node rclcpp)

# --- Install targets ---
install(TARGETS
  esdf_map_core
  esdf_map_node_lib
  esdf_map_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# --- Install headers ---
install(DIRECTORY include/
  DESTINATION include
)

# --- Install launch + config + srv ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
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
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  rosidl_default_runtime
)

ament_package()
