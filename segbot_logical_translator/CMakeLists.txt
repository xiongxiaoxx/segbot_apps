# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(segbot_logical_translator)

find_package(catkin REQUIRED 
  COMPONENTS 
    actionlib 
    bwi_mapper 
    bwi_planning_common 
    message_filters
    move_base_msgs
    nav_msgs
    roscpp
    tf
)

catkin_package(
  CATKIN_DEPENDS 
    bwi_mapper 
    bwi_planning_common
    move_base_msgs
    nav_msgs
    roscpp 
    tf
  INCLUDE_DIRS 
    include
  LIBRARIES
    segbot_logical_translator
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(segbot_logical_translator 
  src/libsegbot_logical_translator/segbot_logical_translator.cpp)
target_link_libraries(segbot_logical_translator 
  ${catkin_LIBRARIES}
)

add_executable(segbot_logical_navigator src/nodes/segbot_logical_navigator.cpp)
target_link_libraries(segbot_logical_navigator 
  ${catkin_LIBRARIES}
  segbot_logical_translator)
 
install(TARGETS segbot_logical_translator segbot_logical_navigator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/segbot_logical_translator/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

