cmake_minimum_required(VERSION 3.8)
project(fishbot_foxglove)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch layouts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/goal_router.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
