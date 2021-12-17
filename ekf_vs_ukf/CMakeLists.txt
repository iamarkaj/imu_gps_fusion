cmake_minimum_required(VERSION 3.0.2)
project(ekf_vs_ukf)

find_package(catkin REQUIRED COMPONENTS 
    rospy 
    robot_localization
)

catkin_package()

catkin_install_python(PROGRAMS 
    scripts/trajectory_marker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
