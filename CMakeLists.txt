cmake_minimum_required(VERSION 2.8)

add_definitions(-std=c++11)

project(helloworld)

#find_package(Gflags REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

include_directories(include ${CERES_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_library(rbf_kernel src/sparse_gp/rbf_kernel.cpp)

add_library(probit_noise src/sparse_gp/probit_noise.cpp)

# helloworld
add_executable(helloworld src/helloworld.cpp)
target_link_libraries(helloworld ${CERES_LIBRARIES})

add_executable(pose_graph_3d src/pose_graph_3d.cpp)
target_link_libraries(pose_graph_3d ${CERES_LIBRARIES})

add_executable(test_gaussian_process src/test_gaussian_process.cpp)
target_link_libraries(test_gaussian_process rbf_kernel probit_noise ${CERES_LIBRARIES})
