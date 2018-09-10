project(gpgs_slam)

#include_directories(../../libigl/include ../../libigl/external/glad/include)

#include_directories(../../libigl/external/glad/include)
#add_library(glad ../../libigl/external/glad/src/glad.c)
#target_include_directories(glad PUBLIC include)

add_library(gpgs_cost_function src/cost_function.cpp)

add_library(gpgs_visualization src/visualization.cpp)

add_library(gpgs_multi_visualizer src/multi_visualizer.cpp)

add_library(gpgs_igl_visualizer src/igl_visualizer.cpp)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(gpgs_cost_function PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(gpgs_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(gpgs_multi_visualizer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(gpgs_igl_visualizer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../libigl/include>
    #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../libigl/external/glad/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_link_libraries(gpgs_cost_function data_transforms ${CERES_LIBRARIES} sparse_gp rbf_kernel gaussian_noise submaps)
target_link_libraries(gpgs_visualization data_transforms ${CERES_LIBRARIES} sparse_gp rbf_kernel gaussian_noise ${PCL_LIBRARIES} ${OpenCV_LIBS} submaps)
target_link_libraries(gpgs_multi_visualizer gpgs_visualization data_transforms ${CERES_LIBRARIES} sparse_gp rbf_kernel gaussian_noise ${PCL_LIBRARIES} ${OpenCV_LIBS} submaps)
#target_link_libraries(gpgs_igl_visualizer gpgs_visualization data_transforms ${CERES_LIBRARIES} sparse_gp rbf_kernel gaussian_noise ${OpenCV_LIBS} submaps glad ${GLFW3_LIBRARY} ${OPENGL_LIBRARY} ${OPENGL_glu_LIBRARY} -lpthread)
target_link_libraries(gpgs_igl_visualizer gpgs_visualization data_transforms ${CERES_LIBRARIES} sparse_gp rbf_kernel gaussian_noise ${OpenCV_LIBS} submaps igl::core igl::opengl_glfw -lpthread)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS gpgs_cost_function gpgs_visualization
                gpgs_multi_visualizer gpgs_igl_visualizer EXPORT GPGSSlamConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT GPGSSlamConfig DESTINATION share/GPGSSlam/cmake)

# This makes the project importable from the build directory
export(TARGETS data_transforms gpgs_cost_function gpgs_visualization
               gpgs_multi_visualizer gpgs_igl_visualizer FILE GPGSSlamConfig.cmake)
