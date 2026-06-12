cmake_minimum_required(VERSION 3.8)

project(nftm_tester)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
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