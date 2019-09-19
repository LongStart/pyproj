cmake_minimum_required(VERSION 3.0)
project(pose_state)
set(PYBIND11_PYTHON_VERSION 2.7)

find_package(pybind11 REQUIRED)  # or add_subdirectory(pybind11)
find_package(Eigen3 REQUIRED)

include_directories( ${EIGEN3_INCLUDE_DIRS} )

add_library(pose_state MODULE pose_state.cpp)
target_link_libraries(pose_state PRIVATE pybind11::module)
set_target_properties(pose_state PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")