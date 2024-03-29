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

option(AUVLIB_WITH_GSF "Enable reading and processing gsf data"    ON)
option(AUVLIB_USE_LIBIGL_EIGEN "Use libigl's version of Eigen"     ON)
option(AUVLIB_USE_LIBIGL_GLFW "Use libigl's version of glfw"       ON)
option(AUVLIB_USE_LIBIGL_TINYXML "Use libigl's version of tinyxml" ON)
option(AUVLIB_USE_PYTHON3 "Use python3 instead of python2"         ON)
option(AUVLIB_EXPORT_BUILD "Export build dir configs"              OFF)
option(AUVLIB_WITH_CERES "Use the ceres library, required for refraction tracing" OFF)

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
option(LIBIGL_WITH_XML               "Use XML"            ON)
#option(LIBIGL_EXPORT_TARGETS         "Export libigl CMake targets" ON)

find_package(OpenCV REQUIRED core highgui imgproc)
find_package(OpenGL REQUIRED)
if (AUVLIB_USE_LIBIGL_TINYXML)
  set(TinyXML2_LIBRARIES tinyxml2)
else()
  #find_package(TinyXML2 REQUIRED)
  set(TinyXML2_LIBRARIES -ltinyxml2)
endif()

set(ENV{LIBIGL_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/libigl)
find_package(LIBIGL REQUIRED QUIET)
include(${CMAKE_CURRENT_BINARY_DIR}/embree/embree-config.cmake)

if (AUVLIB_USE_LIBIGL_GLFW)
  #add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libigl/external/glfw) # this might be needed on windows?
  set(GLFW3_LIBRARY glfw)
else()
  find_package(glfw3 REQUIRED)
endif()

if (AUVLIB_USE_LIBIGL_EIGEN)
  set(EIGEN3_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/libigl/external/eigen)
else()
  find_package(Eigen3 REQUIRED)
endif()

if(MSVC)
  if (AUVLIB_USE_PYTHON3)
    set(PYTHON_EXECUTABLE C:/Python38-x64/python.exe)
  else()
    #set(PYTHON_EXECUTABLE C:/Python27/python.exe)
    set(PYTHON_EXECUTABLE C:/Python27-x64/python.exe)
  endif()
else()
  if (AUVLIB_USE_PYTHON3)
    set(PYTHON_EXECUTABLE /usr/bin/python3)
  else()
    set(PYTHON_EXECUTABLE /usr/bin/python2.7)
  endif()
endif()

if (MSVC)
  set(Boost_USE_STATIC_LIBS ON CACHE BOOL "use static libraries from Boost")
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_DEBUG_RUNTIME OFF)
endif()
# For some reason it seems like we need to do this after libigl
find_package(Boost COMPONENTS system filesystem date_time REQUIRED)

add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

set(PYBIND11_CPP_STANDARD -std=c++11)
#add_definitions(-DPYBIND11_PYTHON_VERSION=2.7)
#set(PYBIND11_PYTHON_VERSION 2.7)
#set(PYTHON_LIBRARIES /usr/lib/x86_64-linux-gnu/libpython2.7.so)

add_subdirectory(pybind11)

message("Pybind11: " ${PYBIND11_INCLUDE_DIR})

add_subdirectory(cxxopts)

include_directories(${PYBIND11_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/cereal/include ${TinyXML2_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
#include_directories(eigen_cereal/include)

include(cmake/glad.cmake)
if(AUVLIB_WITH_GSF)
  add_subdirectory(libgsf)
endif()
add_subdirectory(libxtf)
add_subdirectory(eigen_cereal)
add_subdirectory(data_tools)
add_subdirectory(bathy_maps)
add_subdirectory(sonar_tracing)
add_subdirectory(pydata_tools)
add_subdirectory(pybathy_maps)
add_subdirectory(pysonar_tracing)
