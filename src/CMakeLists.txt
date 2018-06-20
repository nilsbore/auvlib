cmake_minimum_required(VERSION 2.8)
project(gpgs_slam_project)

#add_definitions(-std=c++11 -O3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -O3")

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

include_directories(${CERES_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(eigen_cereal/include)

add_subdirectory(eigen_cereal)
add_subdirectory(sparse_gp)
add_subdirectory(libgsf)
add_subdirectory(data_tools)
add_subdirectory(gpgs_slam)
add_subdirectory(apps)
