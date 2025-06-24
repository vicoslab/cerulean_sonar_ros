cmake_minimum_required(VERSION 3.0.2)
project(cerulean_sonar_ros)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    rospy
    std_msgs
    sensor_msgs
)


catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure rospy geometry_msgs std_msgs sensor_msgs message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

file(
    GLOB PYTHON_SCRIPTS_DIR "scripts/*.py"
)
catkin_install_python(
    PROGRAMS ${PYTHON_SCRIPTS_DIR}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY public launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)