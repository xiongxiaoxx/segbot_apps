cmake_minimum_required(VERSION 2.8.3)
project(segbot_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()
catkin_package()

install(PROGRAMS scripts/question_dialog_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
