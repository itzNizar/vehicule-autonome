cmake_minimum_required(VERSION 2.8.3)
project(limo_autonomous)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/controller_node.py
  scripts/obstacle_avoidance_node.py
  scripts/rviz_helper_node.py
  scripts/send_goal.py
  scripts/simulation_standalone.py
  scripts/supervisor_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
