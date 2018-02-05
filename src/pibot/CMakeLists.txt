cmake_minimum_required(VERSION 2.8.3)
project(pibot)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    roslint
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    scripts/teleop.py
    scripts/robot.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

roslint_python()
