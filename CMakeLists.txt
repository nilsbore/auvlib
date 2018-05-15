cmake_minimum_required(VERSION 2.8)

add_definitions(-std=c++11 -O3)

project(helloworld)

#find_package(Gflags REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

include_directories(include ${CERES_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(rbf_kernel src/sparse_gp/rbf_kernel.cpp)

add_library(probit_noise src/sparse_gp/probit_noise.cpp)

add_library(gaussian_noise src/sparse_gp/gaussian_noise.cpp)

add_library(octave_convenience src/sparse_gp/octave_convenience.cpp)

add_library(submaps src/data_tools/submaps.cpp)
target_link_libraries(submaps ${PCL_LIBRARIES})

add_library(gpgs_transforms src/gpgs_slam/transforms.cpp)

add_library(gpgs_cost_function src/gpgs_slam/cost_function.cpp)
target_link_libraries(gpgs_cost_function gpgs_transforms ${CERES_LIBRARIES} rbf_kernel gaussian_noise submaps)

add_library(gpgs_visualization src/gpgs_slam/visualization.cpp)
target_link_libraries(gpgs_visualization gpgs_transforms ${CERES_LIBRARIES} rbf_kernel gaussian_noise ${PCL_LIBRARIES} ${OpenCV_LIBS} submaps)


# helloworld
add_executable(helloworld src/helloworld.cpp)
target_link_libraries(helloworld ${CERES_LIBRARIES})

add_executable(pose_graph_3d src/pose_graph_3d.cpp)
target_link_libraries(pose_graph_3d ${CERES_LIBRARIES})

add_executable(test_gaussian_process src/test_gaussian_process.cpp)
target_link_libraries(test_gaussian_process rbf_kernel gaussian_noise probit_noise octave_convenience submaps ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(visualize_pointclouds src/visualize_pointclouds.cpp)
target_link_libraries(visualize_pointclouds ${PCL_LIBRARIES})

add_executable(visualize_process src/visualize_process.cpp)
target_link_libraries(visualize_process rbf_kernel gaussian_noise probit_noise octave_convenience submaps ${PCL_LIBRARIES})

add_executable(register_process src/register_process.cpp)
target_link_libraries(register_process submaps gpgs_cost_function gpgs_visualization)

add_executable(register_process_ceres src/register_process_ceres.cpp)
#target_link_libraries(register_process_ceres rbf_kernel gaussian_noise probit_noise octave_convenience submaps ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(register_process_ceres submaps gpgs_cost_function gpgs_visualization)
