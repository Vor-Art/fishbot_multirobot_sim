cmake_minimum_required(VERSION 3.8)
project(gazebo_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf world
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
