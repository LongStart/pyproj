cmake_minimum_required(VERSION 2.8)
project(calibration_service_test)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_service_files(DIRECTORY srv FILES Calibration.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

# catkin_install_python(PROGRAMS
#   src/calibration_server.py
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/calibration_server
# )

