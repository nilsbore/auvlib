cmake_minimum_required(VERSION 2.8)

#add_definitions(-std=c++11 -O3)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -O3")

#add_definitions(-DIGL_VIEWER_WITH_NANOGUI=1)

project(helloworld)

#find_package(Gflags REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

include_directories(include libigl/include libigl/external/glad/include ${CERES_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(glad libigl/external/glad/src/glad.c)
#target_include_directories(glad PUBLIC include)

add_library(rbf_kernel src/sparse_gp/rbf_kernel.cpp)

add_library(probit_noise src/sparse_gp/probit_noise.cpp)

add_library(gaussian_noise src/sparse_gp/gaussian_noise.cpp)

add_library(octave_convenience src/sparse_gp/octave_convenience.cpp)

add_library(sparse_gp src/sparse_gp/sparse_gp.cpp)
target_link_libraries(sparse_gp rbf_kernel gaussian_noise octave_convenience)

add_library(submaps src/data_tools/submaps.cpp)
target_link_libraries(submaps ${PCL_LIBRARIES})

add_library(data_structures src/data_tools/data_structures.cpp)
target_link_libraries(data_structures sparse_gp rbf_kernel gaussian_noise)

add_library(gpgs_transforms src/gpgs_slam/transforms.cpp)

add_library(navi_data src/data_tools/navi_data.cpp)
target_link_libraries(navi_data gpgs_transforms ${PCL_LIBRARIES})

add_library(gpgs_cost_function src/gpgs_slam/cost_function.cpp)
target_link_libraries(gpgs_cost_function gpgs_transforms ${CERES_LIBRARIES} sparse_gp rbf_kernel gaussian_noise submaps)

add_library(gpgs_visualization src/gpgs_slam/visualization.cpp)
target_link_libraries(gpgs_visualization gpgs_transforms ${CERES_LIBRARIES} sparse_gp rbf_kernel gaussian_noise ${PCL_LIBRARIES} ${OpenCV_LIBS} submaps)

add_library(gpgs_multi_visualizer src/gpgs_slam/multi_visualizer.cpp)
target_link_libraries(gpgs_multi_visualizer gpgs_visualization gpgs_transforms ${CERES_LIBRARIES} sparse_gp rbf_kernel gaussian_noise ${PCL_LIBRARIES} ${OpenCV_LIBS} submaps)

add_library(gpgs_igl_visualizer src/gpgs_slam/igl_visualizer.cpp)
target_link_libraries(gpgs_igl_visualizer gpgs_visualization gpgs_transforms ${CERES_LIBRARIES} sparse_gp rbf_kernel gaussian_noise ${OpenCV_LIBS} submaps glad ${GLFW3_LIBRARY} ${OPENGL_LIBRARY} ${OPENGL_glu_LIBRARY} -lpthread)

# helloworld
add_executable(helloworld src/helloworld.cpp)
target_link_libraries(helloworld ${CERES_LIBRARIES})

add_executable(parse_navi_data src/parse_navi_data.cpp)
target_link_libraries(parse_navi_data navi_data sparse_gp gpgs_transforms gpgs_igl_visualizer data_structures ${PCL_LIBRARIES})

add_executable(pose_graph_3d src/pose_graph_3d.cpp)
target_link_libraries(pose_graph_3d ${CERES_LIBRARIES})

add_executable(test_gaussian_process src/test_gaussian_process.cpp)
target_link_libraries(test_gaussian_process sparse_gp rbf_kernel gaussian_noise probit_noise octave_convenience submaps ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(visualize_pointclouds src/visualize_pointclouds.cpp)
target_link_libraries(visualize_pointclouds ${PCL_LIBRARIES})

add_executable(visualize_process src/visualize_process.cpp)
target_link_libraries(visualize_process sparse_gp rbf_kernel gaussian_noise probit_noise octave_convenience submaps ${PCL_LIBRARIES})

add_executable(register_process src/register_process.cpp)
target_link_libraries(register_process sparse_gp submaps gpgs_cost_function gpgs_visualization)

add_executable(register_process_ceres src/register_process_ceres.cpp)
#target_link_libraries(register_process_ceres rbf_kernel gaussian_noise probit_noise octave_convenience submaps ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(register_process_ceres submaps gpgs_cost_function gpgs_visualization)

add_executable(slam_process_ceres src/slam_process_ceres.cpp)
target_link_libraries(slam_process_ceres submaps gpgs_cost_function gpgs_visualization gpgs_multi_visualizer gpgs_igl_visualizer data_structures)

add_executable(visualize_slam_process src/visualize_slam_process.cpp)
#target_link_libraries(visualize_slam_process submaps gpgs_cost_function gpgs_visualization gpgs_multi_visualizer gpgs_igl_visualizer)
target_link_libraries(visualize_slam_process gpgs_visualization gpgs_igl_visualizer data_structures)

add_executable(test_fast_derivatives test/test_fast_derivatives.cpp)
target_link_libraries(test_fast_derivatives submaps gpgs_cost_function gpgs_visualization gpgs_multi_visualizer gpgs_transforms)

add_executable(test_igl src/test_igl.cpp)
target_link_libraries(test_igl glad ${GLFW3_LIBRARY} ${OPENGL_LIBRARY} ${OPENGL_glu_LIBRARY} -lpthread sparse_gp rbf_kernel gaussian_noise probit_noise octave_convenience submaps)
