cmake_minimum_required(VERSION 3.0.2)
project(moveit_python)

find_package(catkin REQUIRED COMPONENTS
geometry_msgs
roscpp
rospy
std_msgs
nav_msgs
tf
)

catkin_package()
catkin_python_setup()

catkin_install_python(
  PROGRAMS
    scripts/add_box.py
    scripts/attach_box.py
    scripts/dump_planning_scene.py
    scripts/list_objects.py
    scripts/load_planning_scene.py
    scripts/remove_objects.py
    scripts/teste.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
