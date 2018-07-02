cmake_minimum_required(VERSION 2.8)
project(gpgs_example_project)

#add_definitions(-std=c++11 -O3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -O3")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

find_package(SparseGP REQUIRED)
find_package(DataTools REQUIRED)
find_package(GPGSSlam REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

include_directories(include ${CERES_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(../src/eigen_cereal/include)
include_directories(../libigl/include)
include_directories(../libigl/external/glad/include)

add_executable(example_slam src/example_slam.cpp)
target_link_libraries(example_slam submaps gpgs_cost_function gpgs_visualization gpgs_multi_visualizer gpgs_igl_visualizer data_structures)
