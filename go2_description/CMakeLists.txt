cmake_minimum_required(VERSION 3.8)

project(go2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config dae launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

file(GLOB SCRIPT_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py"
)

install(
  PROGRAMS ${SCRIPT_FILES}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
