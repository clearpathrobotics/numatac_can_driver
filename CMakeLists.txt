cmake_minimum_required(VERSION 2.8.3)
project(numatac_can_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  )


add_message_files(
  FILES
  FingerPressure.msg
  HandPressure.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES numatac_can_driver
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(numatac_can_driver
  src/numatac_can_driver.cpp
)

target_link_libraries(numatac_can_driver
  ${catkin_LIBRARIES}
)

add_dependencies(numatac_can_driver numatac_can_driver_gencpp)
add_executable(numatac_can_node src/numatac_can_node.cpp)


target_link_libraries(numatac_can_node
  numatac_can_driver
  ${catkin_LIBRARIES}
)

install(TARGETS numatac_can_node numatac_can_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
