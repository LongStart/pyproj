cmake_minimum_required(VERSION 3.0)
project(rolling_shutter)
set(PYBIND11_PYTHON_VERSION 2.7)

find_package(pybind11 REQUIRED)  # or add_subdirectory(pybind11)
find_package(Eigen3 REQUIRED)

include_directories( ${EIGEN3_INCLUDE_DIRS} )

add_library(rolling_shutter MODULE rolling_shutter.cpp)
target_link_libraries(rolling_shutter PRIVATE pybind11::module)
set_target_properties(rolling_shutter PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")