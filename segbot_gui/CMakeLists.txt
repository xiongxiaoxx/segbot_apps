cmake_minimum_required(VERSION 2.8.3)
project(segbot_gui)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  QuestionDialog.srv
)
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

install(PROGRAMS scripts/question_dialog_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
