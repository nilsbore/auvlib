cmake_minimum_required(VERSION 2.8)
project(gpgs_slam_project)

#add_definitions(-std=c++11 -O3)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -O3 -fPIC")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# libigl
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" OFF)
option(LIBIGL_WITH_ANTTWEAKBAR       "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL              "Use CGAL"           OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK              "Use Cork"           OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"         ON)
option(LIBIGL_WITH_LIM               "Use LIM"            OFF)
option(LIBIGL_WITH_MATLAB            "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK             "Use MOSEK"          OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          OFF)
option(LIBIGL_WITH_PNG               "Use PNG"            OFF)
option(LIBIGL_WITH_PYTHON            "Use Python"         OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         OFF)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       OFF)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML               "Use XML"            OFF)

find_package(LIBIGL REQUIRED QUIET)

# For some reason it seems like we need to do this after libigl
find_package(Boost COMPONENTS system filesystem date_time REQUIRED)

add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

set(PYBIND11_CPP_STANDARD -std=c++11)
#add_definitions(-DPYBIND11_PYTHON_VERSION=2.7)
#set(PYBIND11_PYTHON_VERSION 2.7)
#set(PYTHON_LIBRARIES /usr/lib/x86_64-linux-gnu/libpython2.7.so)
set(PYTHON_EXECUTABLE /usr/bin/python2.7)
add_subdirectory(pybind11)

message("Pybind11: " ${PYBIND11_INCLUDE_DIR})

add_subdirectory(cxxopts)

include_directories(${PYBIND11_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
#include_directories(eigen_cereal/include)

add_subdirectory(libgsf)
add_subdirectory(libxtf)
add_subdirectory(eigen_cereal)
add_subdirectory(data_tools)
add_subdirectory(bathy_maps)
add_subdirectory(sonar_tracing)
add_subdirectory(pydata_tools)
add_subdirectory(pybathy_maps)
add_subdirectory(pysonar_tracing)
